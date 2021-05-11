/*!
 * @file      board-config.h
 *
 * @brief     board specific pinout
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Defines the time required for the TCXO to wakeup [ms].
 */
#if defined( LR1110EVK )
#define BOARD_TCXO_WAKEUP_TIME 5
#else
#define BOARD_TCXO_WAKEUP_TIME 0
#endif

/*!
 * @brief Board MCU pins definitions
 */

/* LR1110 */
#define RADIO_RESET PA_0
#define RADIO_MOSI PA_7
#define RADIO_MISO PA_6
#define RADIO_SCLK PA_5
#define RADIO_NSS PA_8
#define RADIO_BUSY PB_3
#define RADIO_EVENT PB_4

#define LNA_PON PB_0

/* Sensors */
#define I2C_SCL PB_8
#define I2C_SDA PB_9

#define ACC_INT1 PA_9

/* LED */
#define LED_RX PC_0
#define LED_TX PC_1
#define LED_SCAN PB_5

#define OSC_LSE_IN PC_14
#define OSC_LSE_OUT PC_15

#define OSC_HSE_IN PH_0
#define OSC_HSE_OUT PH_1

#define SWCLK PA_14
#define SWDAT PA_13

#define UART_TX PA_2
#define UART_RX PA_3

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __BOARD_CONFIG_H__

/* --- EOF ------------------------------------------------------------------ */
