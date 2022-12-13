# Edge Impulse Example: standalone inferencing for Infineon PSoC6

This code example runs an exported impulse on any Infineon PSoC 6 series MCU and demonstrates:
* Running the classifier from a trained model in the [EdgeImpulse](https://edgeimpulse.com) Online Studio
* Using a features data set from the [EdgeImpulse Online Studio](https://studio.edgeimpulse.com)

See the documentation at [Running your impulse locally](https://docs.edgeimpulse.com/docs/running-your-impulse-locally-1).

## Requirements

### Hardware

* This example can work with any PSOC 6 series MCU and development board
* Tested with [PSoC 62 BLE pioneer kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-062-ble/) (`CY8CKIT-062-BLE`)
* This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

**Note:** The PSoC&trade; 6 Bluetooth&reg; LE pioneer kit (CY8CKIT-062-BLE) and the PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. The ModusToolbox&trade; software requires KitProg3. Before using this code example, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the [Firmware Loader](https://github.com/Infineon/Firmware-loader) GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".

### Software

* Install ModusToolbox SDK and IDE

## Supported kits (make variable 'TARGET')

- [PSoC&trade; 6 Wi-Fi Bluetooth® prototyping kit](https://www.cypress.com/CY8CPROTO-062-4343W) (`CY8CPROTO-062-4343W`) – Default value of `TARGET`
- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit](https://www.cypress.com/CY8CKIT-062-WiFi-BT) (`CY8CKIT-062-WIFI-BT`)
- [PSoC&trade; 6 Bluetooth&reg; LE pioneer kit](https://www.cypress.com/CY8CKIT-062-BLE) (`CY8CKIT-062-BLE`)
- [PSoC&trade; 6 Bluetooth&reg; LE prototyping kit](https://www.cypress.com/CY8CPROTO-063-BLE) (`CY8CPROTO-063-BLE`)
- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; pioneer kit](https://www.cypress.com/CY8CKIT-062S2-43012) (`CY8CKIT-062S2-43012`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; pioneer kit](https://www.cypress.com/CYW9P62S1-43438EVB-01) (`CYW9P62S1-43438EVB-01`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; pioneer kit](https://www.cypress.com/CYW9P62S1-43012EVB-01) (`CYW9P62S1-43012EVB-01`)
- [PSoC&trade; 62S3 Wi-Fi Bluetooth&reg; prototyping kit](https://www.cypress.com/CY8CPROTO-062S3-4343W) (`CY8CPROTO-062S3-4343W`)
- [PSoC&trade; 64 "Secure Boot" Wi-Fi Bluetooth&reg; pioneer kit](https://www.cypress.com/CY8CKIT-064B0S2-4343W) (`CY8CKIT-064B0S2-4343W`)
- [PSoC&trade; 62S4 pioneer kit](https://www.cypress.com/CY8CKIT-062S4) (`CY8CKIT-062S4`)
- [PSoC&trade; 62S2 evaluation kit](https://www.cypress.com/CY8CEVAL-062S2) (`CY8CEVAL-062S2`, `CY8CEVAL-062S2-LAI-4373M2`, `CY8CEVAL-062S2-MUR-43439M2`)
- [PSoC&trade; 64 "Secure Boot" prototyping kit](https://www.cypress.com/CY8CPROTO-064B0S3) (`CY8CPROTO-064B0S3`)
- [PSoC&trade; 64 "Secure Boot" prototyping kit](https://www.cypress.com/CY8CPROTO-064S1-SB) (`CY8CPROTO-064S1-SB`)

## Building the application

### Get the Edge Impulse SDK

Unzip the deployed `C/C++ library` from your Edge Impulse project and copy only the folders to the `ei-model` directory of this project:

   ```
   example-standalone-inferencing-psoc62/PSoC62_App
   ├─ LICENSE
   ├─ Infineon-EULA.txt
   ├─ Makefile
   ├─ makefile.init
   ├─ deps
   ├─ libs
   ├─ README.md
   ├─ main.cpp
   └─ ei-model
        ├─ edge-impulse-sdk
        ├─ model-parameters
        └─ tflite-model  
   ```

### Compile

Initialize PSoC63 BSP. This step is optional, because all required libraries are included in this repo.

   ```bash
   make getlibs
   ```

Compile in terminal as shown below (or use ModusToolbox IDE).

   ```bash
   make build
   ```

### Flash

Connect the PSoC6 BLE pioneer kit to your computer and execute the command below.

   ```bash
   make flash
   ```

### Run

Use screen, minicom or terminal in ModusToolbox IDE to set up a serial connection over USB. The following UART settings are used: 115200 baud, 8N1.

   ```bash
   minicom -D /dev/cu.usbmodem11203
   ```

Where `/dev/cu.usbmodem11203` needs to be changed to the actual port where the PSoC6 BLE pioneer kit is connected on your system.
