# LR1110 modem application example

## 1. Description

The LR1110 modem application example project contains several simple examples highlighting LoRa Basic Modem-E features.

### 1.1. Simple LoRaWAN Class A/C application

This application joins automatically the LoRa Network server then sends uplinks periodically with the interval defined by APP_TX_DUTYCYCLE

Please read the [application documentation](Docs/apps.LoRaWAN.md) for more details.

### 1.2. Simple GNSS example

The application executes a periodic GNSS scan.

Please read the [application documentation](Docs/apps.gnss_test.md) for more details.

### 1.3. Simple Wi-Fi example

The modem executes a Wi-Fi scan periodically according to the Wi-Fi settings defined in the application.

Please read the [application documentation](Docs/apps.wifi_test.md) for more details.

### 1.4. Simple Tx continuous app

The modem starts to send continuous wave.

Please read the [application documentation](Docs/apps.tx_continuous.md) for more details.

### 1.5. Simple LoRaWAN clock synchronization example

This applications joins automatically the LoRaWAN Network Server. The LR1110 then gets it clocks synchronized with the Application Layer Clock Synchronization service (ALC Sync).

Please read the [application documentation](Docs/apps.clock_sync.md) for more details.

### 1.6. UART-based firmware update application example

This applications updates the LR1110 firmware. The new firmware is received from the UART.

Please read the [application documentation](Docs/apps.uart_firmware_update.md) for more details.

### 1.7. Tracker example

The app joins automatically the network then periodically:

- Perform a Wi-Fi Scan
- Perform a GNSS scan, the ALC Sync is needed
- Stream the scan results

Please read the [application documentation](Docs/apps.Tracker.md) for more details.

### 1.8. Simple LoRaWAN Data Streaming application

This application joins automatically the LoRa Network Server then streams data periodically.

Please read the [application documentation](Docs/apps.stream.md) for more details.

### 1.9. Large File Upload example

The app joins automatically and starts the upload data using the Large File Upload service.

Please read the [application documentation](Docs/apps.large_file_upload.md) for more details.## 2. Requirements

### 2.1. Hardware

The example applications are designed to run with the LR1110 Evaluation Kit hardware, namely:

* NUCLEO-L476RG development board
* LR1110 shield
* Touchscreen (not used in these applications)


### 2.2. LR1110 Firmware

The applications require that the LR1110 runs the Modem-E firmware version 1.1.7 or later. To update the Modem-E to the latest firmware version please use the updater tool application: https://github.com/Lora-net/lr1110_updater_tool/.

Applications usually display the detected LR1110 Firmware version in the serial console when they start, here the LoRaWAN showing version 1.1.7 (0x10107) of the Modem-E firmware:

```
###### ===== LoRa Basics Modem-E LoRaWAN Class A/C demo application ==== ######

APP VERSION : 1.5.0

INFO : ###### ===== LR1110 MODEM-E RESET 555 ==== ######

INFO : ###### ===== LR1110 MODEM-E VERSION ==== ######

LORAWAN     : 0X103
FIRMWARE    : 0X10107
BOOTLOADER  : 0X21DF6500
CLASS       : A
REGION      : EU868
```

## 3. Usage

Connect the NUCLEO board to a host PC via an USB cable. The USB connection will provide power for the LR1110 Evaluation Kit as well as serve as a serial communication link for the example application.

Use a terminal application configured with the following settings:
- Speed: 921600 baud
- Data bits: 8s
- Stop bits: 1
- Parity: None

Applications use the serial link to display information messages. The GNSS application also use this link to get the date from the user.

To use the Semtech Join Server keys derivation algorithm: update the USE_SEMTECH_JOIN_SERVER definition

To use the LR1110 modem production keys: update the USE_PRODUCTION_KEYS definition

## 4. Build & Install

To build the example application for the STM32L476RG controller of the NUCLEO development board, you will need:

* either a Keil MDK for Cortex-M commercial license, MDK-Lite will not work (https://www2.keil.com/mdk5)
* or the GNU Arm Embedded Toolchain (https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

To build the example applications, you can either use Keil MDK or the GNU Arm Embedded Toolchain.

### 4.1. Building with Keil MDK

Use the Keil project `lr1110_modem_application_example.uvprojx` located in the `MDK-ARM` directory.

### 4.2. Building with the GNU Arm Embedded Toolchain

1. Run `make` from the `gcc` directory with the target application name as an argument:

```
$ make APP=lorawan
```

Note: the supported application names are `tracker`, `lorawan`, `gnss`, `wifi`, `clock_sync`, `tx_continuous`, `stream`, `read_internal_log`, `uart_firmware_update` and `large_file_upload`.

2. The application binary file, for example `wifi.bin`, is created in the `gcc/build` directory.
3. Copy the binary file to the STM32 microcontroller, either using the host OS copy facility or a dedicated tool like the STM32 ST-Link utility.
