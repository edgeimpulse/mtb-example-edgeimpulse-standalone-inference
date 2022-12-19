/*
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "ei_device_psoc62.h"
#include "ei_at_handlers.h"
#include "ei_flash_memory.h"
#include "ei_inertial_sensor.h"
#include "ei_environment_sensor.h"
#include "ei_microphone.h"
#include "ei_run_impulse.h"
#include "cyhal_clock.h"
#include "cyhal_gpio.h"
#include "cyhal_uart.h"

/******
 *
 * @brief EdgeImpulse PSoC63 firmware. See README.md for more information
 *
 ******/

/***************************************
*            Constants
****************************************/
#define UART_CLEAR_SCREEN   "\x1b[2J\x1b[;H"

/***************************************
*            Static Variables
****************************************/
static ATServer *at;
EiDevicePSoC62 *eidev;

void cy_err(int result)
{
    if(result != CY_RSLT_SUCCESS) {
        int err_type = CY_RSLT_GET_TYPE(result);
        int err_module = CY_RSLT_GET_MODULE(result);
        int err_code = CY_RSLT_GET_CODE(result);

        ei_printf("err type = 0x%X module = 0x%X code = 0x%X\n\r", err_type, err_module, err_code);
    }
}

void board_init(void)
{
    cy_rslt_t result;
    cyhal_clock_t system_clock, pll_clock;

    /* Make sure firmware starts after debugger */
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
        Cy_SysLib_Delay(400u);
    }

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize the PLL */
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[1]);
    cyhal_clock_set_frequency(&pll_clock, 150000000, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);
    cyhal_clock_free(&pll_clock);

    /* Initialize the system clock (HFCLK0) */
    cyhal_clock_reserve(&system_clock, &CYHAL_CLOCK_HF[0]);
    cyhal_clock_set_source(&system_clock, &pll_clock);
    cyhal_clock_free(&system_clock);

    /* Do not disable the FLL on PSoC63
    cyhal_clock_reserve(&fll_clock, &CYHAL_CLOCK_FLL);
    cyhal_clock_set_enabled(&fll_clock, false, true);
    cyhal_clock_free(&fll_clock);
    */

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

int main(void)
{
    char uart_data;
    uint32_t led_state = CYBSP_LED_STATE_OFF;

    board_init();
    ei_inertial_sensor_init();
    ei_environment_sensor_init();
    ei_microphone_pdm_init();
    /* Make sure the environmental sensor is settled by sampling more from it */
    ei_environment_sensor_async_trigger();
    ei_printf(UART_CLEAR_SCREEN);

    eidev =  static_cast<EiDevicePSoC62*>(EiDeviceInfo::get_device());
    at = ei_at_init(eidev);
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();
    eidev->set_state(eiStateFinished);

    while(1)
    {
        if(cyhal_uart_getc(&cy_retarget_io_uart_obj, (uint8_t*)&uart_data, 5) == CY_RSLT_SUCCESS) {
            /* Controlling inference */
            if(is_inference_running() && uart_data == 'b') {
                ei_stop_impulse();
                at->print_prompt();
                continue;
            }
            at->handle(uart_data);
        }

        if(is_inference_running()) {
            ei_run_impulse();
        }
    }
}
