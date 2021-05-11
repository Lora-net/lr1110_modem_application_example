# LR1110 modem demo application

## 1. Description

The LR1110 modem demo application project contains several simple examples highlighting lr1110 modem features :

Simple LoRaWAN Class A application EU868/US915/AS915/CN470/IN865/KR920/RU864 MHz:
-	The app joins automatically the LoRa Network server then sends uplinks periodically with the interval defined by APP_TX_DUTYCYCLE
Simple GNSS example :
-	The unix date in ASCII is asked to the user through a terminal, once the date is received by the modem, it executes a GNSS scan periodically according to the gnss settings defined in the application
Simple Wi-Fi example :
-	The modem executes a Wi-Fi scan periodically according to the Wi-Fi settings defined in the application
Simple Tx continuous app :
-	The modem starts to send continuous wave
Tracker example :
-	The app joins automatically the network then periodically :
	- Perform a Wi-Fi Scan
	- Perform a GNSS scan, the alcsync is needed
	- Stream the scan results

## 2. Usage
	
-	Uart baudrate is set to 921600 bauds
-	To use the Semtech Join Server keys derivation algorythm : update the USE_SEMTECH_JOIN_SERVER definition
-	To use the LR1110 modem production keys : update the USE_PRODUCTION_KEYS definition

## 3. Build & Install

To build the host softwares, proceed as follow:

-	Use the Keil project lr1110_modem_application_example.uvprojx located in MDK-ARM
-	Execute the makefile (in gcc folder):
	- Install arm-none-eabi-gcc : https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
	- Make -j
	- lr1110_modem_application_example.bin is created in the gcc/build folder

## 4. Changelog

### V 1.4.1 ###

-	update lr1110 modem driver on v3.0.1 version

### V 1.4.0 ###

-	Modem-E V1.1.7 support
-	Add region AU915, AS923, CN470, IN865, KR920, RU864
-	Add Class C - Add LED toggle for example
-	Support LR1110 Modem-E driver v3.0.0
-	Add read/write tracker parameters over LoRaWAN in tracker application
- 	Refactor examples
-	Minor bug fixes

### V 1.3.0 ###

-	Fix the SNR reading which was wrong
-	Fix RNG module
-	Merge LoRaWAN commissioning files for tracker and lorawan app
-	Update LR1110 Modem-E driver
-   Remove infinite loop in HardFault Handler / Error Handler / Hal mcu panic and replaced by hal_mcu_reset

### V 1.2.0 ###

-	Update the LR1110 modem drivers
-	Work on LR1110 modem 1.0.7 firmware version

### V 1.1.0 ###

-	Update the LR1110 modem drivers
-	Work on LR1110 modem 1.0.5 firmware version
-	Add GNSS assistance position update by DAS

### V 1.0.0 ###

Initial release