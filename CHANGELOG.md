# LR1110 modem application example changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.5.0] 2021-08-16

### Added
* Documenation about every examples
* Clock sync application example
* Stream application example
* Large file upload application example
* uart firmware update application example

### Changed

* Refactor all application examles
* Tracker example update
* LoRaWAN example update
* Modem-E drivers update to v3.0.1

### Fixed

* Makefile

## [v1.4.1] 2021-05-11

### Changed

* update lr1110 modem driver on v3.0.1 version

## [v1.4.0] 2021-05-04

### Added

* Modem-E V1.1.7 support
* Add region AU915, AS923, CN470, IN865, KR920, RU864
* Add Class C - Add LED toggle for example
* Support LR1110 Modem-E driver v3.0.0
* Add read/write tracker parameters over LoRaWAN in tracker application

### Changed

* Refactor examples

### Fixed

* Minor bug fixes

## [v1.3.0] 2020-10-29

### Added

* Update the LR1110 modem drivers

### Changed

* Merge LoRaWAN commissioning files for tracker and lorawan app

### Fixed

 * Fix the SNR reading which was wrong
 * Fix RNG module
 * Remove infinite loop in HardFault Handler / Error Handler / Hal mcu panic and replaced by hal_mcu_reset

## [v1.2.0] 2020-10-12

### Added

* Update the LR1110 modem drivers
* Work on LR1110 modem 1.0.7 firmware version

## [v1.1.0] 2020-09-17

### Added

* Update the LR1110 modem drivers
* Work on LR1110 modem 1.0.5 firmware version
* Add GNSS assistance position update by DAS

## [v1.0.0] 2020-09-16

### Added

* Initial release
