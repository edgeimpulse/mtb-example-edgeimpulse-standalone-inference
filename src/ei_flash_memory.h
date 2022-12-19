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

#ifndef EI_FLASH_MEMORY_H
#define EI_FLASH_MEMORY_H

#include "firmware-sdk/ei_device_memory.h"

extern "C" {
	#include "cy_pdl.h"
	#include "cyhal.h"
	#include "cybsp.h"
	#include <cycfg_qspi_memslot.h>
	#include <cy_serial_flash_qspi.h>
};

#define QSPI_MEM_SLOT_NUM       (0u)		 /* QSPI slot to use */
#define QSPI_BUS_FREQUENCY_HZ   (50000000lu) /* 50 Mhz */
/*
  Flash Related Parameter Define
*/
#define FLASH_ERASE_TIME    600 // Typical time is 520ms + 15% buffer
#define FLASH_SIZE          0x4000000   // 64 MB
#define FLASH_SECTOR_SIZE   0x40000     // 256K Sector size
#define FLASH_PAGE_SIZE     0x0200      // 512 Byte Page size
#define FLASH_BLOCK_NUM     (FLASH_SIZE / SECTOR_SIZE)

class EiFlashMemory : public EiDeviceMemory {
protected:
    uint32_t read_data(uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t erase_data(uint32_t address, uint32_t num_bytes);

public:
    EiFlashMemory(uint32_t config_size);
};

#endif /* EI_FLASH_MEMORY_H */
