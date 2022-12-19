/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "firmware-sdk/sensor_aq.h"
#include "ei_device_psoc62.h"
#include "ei_microphone.h"
#include "sensor_aq_none.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cyhal_clock.h"
#include "cyhal_pdmpcm.h"
#include <stdint.h>
#include <stdlib.h>

/* AUDIO SYSTEM CONSTANTS */
/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ          24576000u
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             64u
/* PDM/PCM Pins */
#define PDM_CLK     P10_4
#define PDM_DATA    P10_5
/* this is variable, received from studio */
#define PDM_DEFAULT_FREQ_HZ (16000UL)

/* Microphone takes about 100ms settling time */
#define MICROPHONE_SETTLE_TIME 300 /* triple this to be safe */

/* Size of the ping-pong buffers in bytes (allocated on stack).
 * Must by multiple of sizeof(microphone_sample_t) and multiple of 2.
 * Using ping-pong buffers enables us to write one buffer to flash,
 * while sampling in the other.
 */
#define TOTAL_BUFFER_SIZE       (32000U)
#define TOTAL_BUFFER_SAMPLES    (TOTAL_BUFFER_SIZE / sizeof(microphone_sample_t))
#define SINGLE_BUFFER_SIZE      (TOTAL_BUFFER_SIZE / 2)
#define SINGLE_BUFFER_SAMPLES   (SINGLE_BUFFER_SIZE / sizeof(microphone_sample_t))

/* LOCAL VARIABLES */
static cyhal_clock_t audio_clock;
/* PDM interface object */
static cyhal_pdm_pcm_t pdm_pcm;
/* Basic configuration */
static cyhal_pdm_pcm_cfg_t pdm_pcm_cfg = {
            .sample_rate     = PDM_DEFAULT_FREQ_HZ,
            .decimation_rate = DECIMATION_RATE,
            .mode            = CYHAL_PDM_PCM_MODE_LEFT,
            .word_length     = sizeof(microphone_sample_t) * 8,  /* bits */
            .left_gain       = 20,   /* dB */
            .right_gain      = 0,   /* dB */
};

/* Sampling related variables */
static volatile bool pdm_pcm_flag = false;
static int16_t *readyBuffer;
static int16_t bufOne[SINGLE_BUFFER_SIZE];
static int16_t bufTwo[SINGLE_BUFFER_SIZE];
/* CBOR variables */
static uint32_t headerOffset;
static uint32_t collected_bytes;

/* Inference variables */
/** Status and control struct for inferencing struct */
typedef struct {
    microphone_sample_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;
static inference_t inference;

/* sample_aq definitions */
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*);
static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin);

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/****************************** SIGNING RELATED FUNCTIONS ***************************************************/

static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    return 0;
}

/****************************** PDM RELATED FUNCTIONS *******************************************************/

bool ei_microphone_pdm_init(void)
{
    cyhal_clock_t pll_clock;

    /* Initialize the PLL */
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);
    cyhal_clock_free(&pll_clock);

    /* Initialize the audio subsystem clock HF[1] */
    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);

    return true;
}

static bool pdm_configure(uint32_t sample_rate, cyhal_pdm_pcm_event_callback_t pdm_callback)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    // set sample rate
    pdm_pcm_cfg.sample_rate = sample_rate;

    // set PDM configuration
    result = cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    if(result != CY_RSLT_SUCCESS) {
        ei_printf("ERR: failed to configure PDM (0x%04lx)\n", result);
        return false;
    }

    // register callback
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_callback, NULL);
    // callback called when async operation is complete (all data received)
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, false);

    // start PDM
    result = cyhal_pdm_pcm_start(&pdm_pcm);
    if(result != CY_RSLT_SUCCESS)
    {
        ei_printf("ERR: PDM interface init failed (0x%04lx)!\n", result);
        return false;
    }
    return true;
}

void ingestion_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    static bool ping_pong = false;

    /* Sampling was started in ei_microphone_sample_start on bufOne,
     * therefore first time we hit our ping_pong scheme, we need to
     * set readyBuffer point to bufone and start sampling with bufTwo.
     *
     */
    if(ping_pong)
    {
        /* Write next frame to ping buffer */
        cyhal_pdm_pcm_read_async(&pdm_pcm, bufOne, SINGLE_BUFFER_SAMPLES);
        /* Setup the pong buffer to be read */
        readyBuffer = bufTwo;
    }
    else
    {
        /* Write next frame to pong buffer */
        cyhal_pdm_pcm_read_async(&pdm_pcm, bufTwo, SINGLE_BUFFER_SAMPLES);
        /* Setup the ping buffer to be read */
        readyBuffer = bufOne;
    }

    ping_pong = !ping_pong;
    pdm_pcm_flag = true;
}

/****************************** INGESTION RELATED FUNCTIONS *************************************************/

static void ingestion_process(uint32_t n_bytes)
{
    EiDevicePSoC62* dev = static_cast<EiDevicePSoC62*>(EiDevicePSoC62::get_device());
    EiDeviceMemory* mem = dev->get_memory();

    if(readyBuffer != NULL) {
        mem->write_sample_data((const uint8_t *)readyBuffer, headerOffset + collected_bytes, n_bytes);
    }

    collected_bytes += n_bytes;
}

static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(size_t i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

static bool create_header(void)
{
    int ret;
    EiDevicePSoC62* dev = static_cast<EiDevicePSoC62*>(EiDevicePSoC62::get_device());
    EiDeviceMemory* mem = dev->get_memory();
    const char *device_name = dev->get_device_id().c_str();
    const char *device_type = dev->get_device_type().c_str();
    float interval_ms = dev->get_sample_interval_ms();

    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    sensor_aq_payload_info payload = {
        device_name,
        device_type,
        interval_ms,
        { { "audio", "wav" } }
    };

    ret = sensor_aq_init(&ei_mic_ctx, &payload, NULL, true);
    if (ret != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t*)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);

    end_of_header_ix += ref_size;

    // Write to blockdevice
    ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if (ret != (int)end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

bool ei_microphone_sample_start(void)
{
    EiDevicePSoC62* dev = static_cast<EiDevicePSoC62*>(EiDevicePSoC62::get_device());
    EiDeviceMemory* mem = dev->get_memory();
    cy_rslt_t result;
    uint32_t required_samples, required_bytes;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    required_samples = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    // Round to even number of samples for word align flash write
    if(required_samples & 1) {
        required_samples++;
    }

    required_bytes = required_samples * sizeof(microphone_sample_t);
    collected_bytes = 0;

    if(required_bytes > mem->get_available_sample_bytes()) {
        ei_printf("ERR: Sample length is too long. Maximum allowed is %lu ms at 16000 Hz.\r\n", 
            ((mem->get_available_sample_bytes() / (16000 * sizeof(microphone_sample_t))) * 1000));
        return false;
    }

    dev->set_state(eiStateErasingFlash);

    // Minimum delay of 2000 ms for daemon
    uint32_t delay_time_ms = ((required_bytes / mem->block_size) + 1) * mem->block_erase_time;
    ei_printf("Starting in %lu ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);

    if(mem->erase_sample_data(0, required_bytes) != (required_bytes)) {
        return false;
    }

    // if erasing took less than 2 seconds, wait additional time
    if(delay_time_ms < 2000) {
        ei_sleep(2000 - delay_time_ms);
    }

    pdm_configure((uint32_t)(1000.f / dev->get_sample_interval_ms()), ingestion_isr_handler);

    create_header();

    // discard first mic data, because it takes about 100ms for the mic to settle
    cyhal_pdm_pcm_read_async(&pdm_pcm, bufOne, SINGLE_BUFFER_SAMPLES);
    ei_sleep(MICROPHONE_SETTLE_TIME);
    cyhal_pdm_pcm_abort_async(&pdm_pcm);
    // enable PDM async sampling
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    // now start normal data collection
    result = cyhal_pdm_pcm_read_async(&pdm_pcm, bufOne, SINGLE_BUFFER_SAMPLES);
    if(result != CY_RSLT_SUCCESS) {
        ei_printf("ERR: no audio data!\n");
    }

    ei_printf("Sampling...\n");
    pdm_pcm_flag = false;
    dev->set_state(eiStateSampling);

    while (collected_bytes < required_bytes) {
        if(pdm_pcm_flag) {
            pdm_pcm_flag = false;
            ingestion_process(SINGLE_BUFFER_SIZE);
        }
    }

    cyhal_pdm_pcm_abort_async(&pdm_pcm);
    cyhal_pdm_pcm_stop(&pdm_pcm);
    cyhal_pdm_pcm_free(&pdm_pcm);

    // we collect multiply of SINGLE_BUFFER_SAMPLES, if user requested less we have to adjust collected_bytes
    if(collected_bytes > required_bytes) {
        collected_bytes = required_bytes;
    }

    ei_printf("Done sampling, total bytes collected: %lu\n", collected_bytes);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", collected_bytes + headerOffset);
    ei_printf("[1/1] Uploading file to Edge Impulse OK (took 0 ms.)\n");
    ei_printf("OK\n");

    return true;
}

/****************************** INFERENCE RELATED FUNCTIONS *************************************************/

void inference_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    /* swap inference buffers */
    inference.buf_select ^= 1;

    /* Write next frame to ping buffer */
    cyhal_pdm_pcm_read_async(&pdm_pcm, inference.buffers[inference.buf_select], inference.n_samples);

    /* mark buffer ready */
    inference.buf_ready = 1;
}

int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr)
{
    /* buf_select is currently used buffer, so get data from the opposite one */
    inference.buf_ready = 0;

    return ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
}

bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    EiDevicePSoC62* dev = static_cast<EiDevicePSoC62*>(EiDevicePSoC62::get_device());
    cy_rslt_t result;

    inference.buffers[0] = (microphone_sample_t*)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[0] == NULL) {
        ei_printf("ERR: Can't allocate 1st buffer (%lu bytes)\n", n_samples * sizeof(microphone_sample_t));
        return false;
    }

    inference.buffers[1] = (microphone_sample_t*)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[1] == NULL) {
        ei_printf("ERR: Can't allocate 2nd buffer (%lu bytes)\n", n_samples * sizeof(microphone_sample_t));
        // 1st buffer is alrerady allocated here, so we have to release it
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    pdm_configure((uint32_t)(1000.0f / dev->get_sample_interval_ms()), inference_isr_handler);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);

    result = cyhal_pdm_pcm_read_async(&pdm_pcm, inference.buffers[0], n_samples);
    if(result != CY_RSLT_SUCCESS) {
        ei_printf("ERR: no audio data!\n");
        return false;
    }

    return true;
}

bool ei_microphone_inference_is_recording(void)
{
    return inference.buf_ready == 0;
}

void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

bool ei_microphone_inference_end(void)
{
    cyhal_pdm_pcm_abort_async(&pdm_pcm);
    cyhal_pdm_pcm_stop(&pdm_pcm);
    cyhal_pdm_pcm_free(&pdm_pcm);

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);

    return true;
}
