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

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_flash_memory.h"

/******
 *
 * @brief The external NOR Flash memory on the PSoC62 43012 development kit has
 * 		  64 MBytes, with sectors of 256kBytes and 512 bytes programming page.
 *
 ******/

uint32_t EiFlashMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
	cy_rslt_t result;

    if(address + num_bytes > this->memory_size) {
        num_bytes = this->memory_size - address;
    }

    result = cy_serial_flash_qspi_read(address, num_bytes, data);
    if(result != CY_RSLT_SUCCESS) {
       num_bytes = 0; /* Inform the caller that we could not read any bytes */
    }

    return num_bytes;
}

uint32_t EiFlashMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
	cy_rslt_t result;
    uint32_t offset = 0;
    uint32_t n_bytes = 0;
    uint32_t bytes_to_write = num_bytes;

    do {
        if(bytes_to_write > FLASH_PAGE_SIZE) {
            n_bytes = FLASH_PAGE_SIZE;
            bytes_to_write -= FLASH_PAGE_SIZE;
        }
        else {
            n_bytes = bytes_to_write;
            bytes_to_write = 0;
        }

        /* If write overflows page, split up in 2 writes */
        if((((address+offset) & 0x000000ff) + n_bytes) > FLASH_PAGE_SIZE) {
            int diff = FLASH_PAGE_SIZE - ((address+offset) & 0xFF);

            result = cy_serial_flash_qspi_write(address + offset, diff, ((uint8_t *)data + offset));
            if(result != CY_RSLT_SUCCESS) {
                num_bytes  =0 ; /* Inform the caller that we could not write any bytes */
                break;
            }

            /* Update index pointers */
            n_bytes -= diff;
            offset += diff;
            bytes_to_write += n_bytes;
        }
        else {
            result = cy_serial_flash_qspi_write(address + offset, n_bytes, ((uint8_t *)data + offset));
            if(result != CY_RSLT_SUCCESS) {
                num_bytes -= bytes_to_write; /* Partial success, inform the user of the amount written */
                break;
            }
            offset += n_bytes;
        }
    } while(bytes_to_write);

    return num_bytes;
}

uint32_t EiFlashMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
	cy_rslt_t result;

    /**
     * Address can point to the middle of sector, but num_bytes may be reaching
     *  part of the last sector
     * +-------+-------+-------+-------+
     * |       |       |       |       |
     * +-------+-------+-------+-------+
     *     ^
     *     address
     *     <-----num_bytes--------->
     */
    uint32_t first_block_offset = address & 0x00000fff;
    uint32_t bytes_to_erase = num_bytes + first_block_offset;
    int num_blocks = bytes_to_erase < this->block_size ? 1 : ceil(float(bytes_to_erase) / this->block_size);

    for(int i=0; i<num_blocks; i++) {
        result = cy_serial_flash_qspi_erase(address + i * this->block_size, this->block_size);
        if(result != CY_RSLT_SUCCESS) {
            num_bytes = i * this->block_size; /* Inform the caller of the partial success */
            break;
        }
    }

    return num_bytes;
}

EiFlashMemory::EiFlashMemory(uint32_t config_size):
    EiDeviceMemory(config_size, FLASH_ERASE_TIME, FLASH_SIZE, FLASH_SECTOR_SIZE)
{
	cy_rslt_t result;

    result = cy_serial_flash_qspi_init(smifMemConfigs[QSPI_MEM_SLOT_NUM],
    			CYBSP_QSPI_D0, CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3,
				NC, NC, NC, NC, CYBSP_QSPI_SCK, CYBSP_QSPI_SS,
				QSPI_BUS_FREQUENCY_HZ);
	CY_ASSERT(result == CY_RSLT_SUCCESS);
}
