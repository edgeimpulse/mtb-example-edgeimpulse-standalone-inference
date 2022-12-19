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

#include <stdint.h>
#include <stdlib.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "mtb_thermistor_ntc_gpio.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_environment_sensor.h"
#include "ei_device_psoc62.h"

/***************************************
*            Constants
****************************************/
#define PIN_THERM_VDD   (CYBSP_A0)
#define PIN_THERM_OUT1  (CYBSP_A1)
#define PIN_THERM_OUT2  (CYBSP_A2)
#define PIN_THERM_GND   (CYBSP_A3)
/* Resistance of the reference resistor */
#define THERM_R_REF     (float)(10000)
/* Beta constant of the NCP18XH103F03RB thermistor is 3380 Kelvin */
#define THERM_B_CONST   (float)(3380)
/* Resistance of the thermistor is 10K at 25 degrees C
 * R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
 * R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855
 */
#define THERM_R_INFINITY (float)(0.1192855)
/* Needed for computation */
#define ABSOLUTE_ZERO   (float)(-273.15)
/* Desired sample rate for Thermistor, as it is a very slow changing sensor,
 * we can sample faster and report the current value without quailty issues.
 */
#define THERM_SAMPLE_FREQ_HZ (uint32)(125000)
#define ADC_ISR_PRIORITY 6

/***************************************
 *        Local variables
 **************************************/
static float env_data[ENV_AXIS_SAMPLED];
cyhal_adc_t adc;
mtb_thermistor_ntc_gpio_t thermistor;
mtb_thermistor_ntc_gpio_cfg_t thermistor_cfg = {
    .r_ref = THERM_R_REF,
    .b_const = THERM_B_CONST,
    .r_infinity = THERM_R_INFINITY,
};
/* Flag: 0 - sample reference, 1 - sample thermistor */
static volatile bool sample_ref_voltage = false;
/* Variable to store sample between ISR and callback */
static volatile int32_t adc_sample = 0;
/* Variables to store the latest adc samples before computation */
static volatile uint32_t voltage_ref = 0;
static volatile uint32_t voltage_therm = 0;


bool ei_environment_sensor_async_trigger(void)
{
    cy_rslt_t result;
    bool ret = false;

    if(sample_ref_voltage) {
        cyhal_gpio_write(PIN_THERM_VDD, 0u);
        cyhal_gpio_write(PIN_THERM_GND, 1u);
    }
    else {
        cyhal_gpio_write(PIN_THERM_VDD, 1u);
        cyhal_gpio_write(PIN_THERM_GND, 0u);
    }

    result = cyhal_adc_read_async(&adc, 1, (int32_t*)&adc_sample);
    if(result == CY_RSLT_SUCCESS) {
        ret = true;
    }

    return ret;
}

void ei_environment_sensor_adc_cb(void *callback_arg, cyhal_adc_event_t event)
{
    if(sample_ref_voltage) {
        voltage_ref = adc_sample;
        /* We need the voltage across the thermistor too */
        ei_environment_sensor_async_trigger();
    }
    else {
        voltage_therm = adc_sample;
        /* We no longer need voltage applied */
        cyhal_gpio_write(PIN_THERM_VDD, 0u);
    }
    /* Indicate what are we sampling next */
    sample_ref_voltage = !sample_ref_voltage;
}

bool ei_environment_sensor_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    bool ret = false;
    static const cyhal_adc_channel_config_t ADC_CHAN_CONFIG = {
        .enabled = true,
        .enable_averaging = false,
        .min_acquisition_ns = 1000u /* 1000ns per Infineon recommendation */
    };
    cyhal_adc_channel_t adc_channel;

    memset(&thermistor, 0, sizeof(thermistor));
    ei_printf("\nInitializing Thermistor\n");

    /* GPIO for reference resistor */
    result = cyhal_gpio_init(PIN_THERM_VDD, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0u);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cyhal_gpio_init(PIN_THERM_GND, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0u);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* ADC init */
    result = cyhal_adc_init(&adc, PIN_THERM_OUT1, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    result = cyhal_adc_channel_init_diff(&adc_channel, &adc, PIN_THERM_OUT1, CYHAL_ADC_VNEG, &ADC_CHAN_CONFIG);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    if (result == CY_RSLT_SUCCESS) {
        ei_printf("Thermistor is online\n");
    }
    else {
        ei_printf("ERR: Thermistor init failed (0x%04x)!\n", result);
        return false;
    }

    /* Register async event to workaround the race condition between ADC and Timer */
    cyhal_adc_register_callback(&adc, ei_environment_sensor_adc_cb, NULL);
    cyhal_adc_enable_event(&adc, CYHAL_ADC_ASYNC_READ_COMPLETE, ADC_ISR_PRIORITY, true);

    /* Register as fusion sensor */
    if(ei_add_sensor_to_fusion_list(environment_sensor) == false) {
        ei_printf("ERR: failed to register Environment sensor!\n");
    }
    else {
        ret = true;
        /* Trigger one initial thermistor read to settle the sensor */
        ei_environment_sensor_async_trigger();
    }

    return ret;
}

float *ei_fusion_environment_sensor_read_data(int n_samples)
{
    /* Calculate thermistor reference as if MTB_THERMISTOR_NTC_WIRING_VIN_R_NTC_GND is set */
    float rThermistor = (THERM_R_REF * voltage_therm) / ((float)(voltage_ref));

    /* Calculate temperature */
    float temperature = (THERM_B_CONST / (logf(rThermistor / THERM_R_INFINITY))) + ABSOLUTE_ZERO;

    /* Store for sensor fusion */
    env_data[0] = temperature;

    /* We do not know if all samples are in, thus we trigger ADC again and in
     * the worst case, the ADC samples one extra data point that is not needed
     */
    ei_environment_sensor_async_trigger();

    return env_data;
}
