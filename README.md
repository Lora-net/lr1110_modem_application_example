# LR1110 modem demo application

## 1. Description

The LR1110 modem demo application project contains several simple examples highlighting lr1110 modem features :

Simple LoRaWAN Class A application 868/915 MHz:
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

-	use the Keil project, or
-	execute the makefile (in gcc folder)

## 4. Changelog

### V 1.2.0 ###

-	Update the LR1110 modem drivers
-	Work on LR1110 modem 1.0.7 firmware version
-	Fix RNG module
-	Code format

### V 1.1.0 ###

-	Update the LR1110 modem drivers
-	Work on LR1110 modem 1.0.5 firmware version
-	Add GNSS assistance position update by DAS

### V 1.0.0 ###

Initial release