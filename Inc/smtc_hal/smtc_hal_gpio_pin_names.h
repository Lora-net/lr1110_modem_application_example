/*!
 * \file      smtc_hal_gpio_pin_names.h
 *
 * \brief     Defines NucleoL073 platform pin names
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

#ifndef __SMTC_HAL_GPIO_PIN_NAMES_H__
#define __SMTC_HAL_GPIO_PIN_NAMES_H__

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \brief HAL Gpio pin name
 */
typedef enum hal_gpio_pin_names_e
{
    // GPIOA
    PA_0  = 0x00,
    PA_1  = 0x01,
    PA_2  = 0x02,
    PA_3  = 0x03,
    PA_4  = 0x04,
    PA_5  = 0x05,
    PA_6  = 0x06,
    PA_7  = 0x07,
    PA_8  = 0x08,
    PA_9  = 0x09,
    PA_10 = 0x0A,
    PA_11 = 0x0B,
    PA_12 = 0x0C,
    PA_13 = 0x0D,
    PA_14 = 0x0E,
    PA_15 = 0x0F,
    // GPIOB
    PB_0  = 0x10,
    PB_1  = 0x11,
    PB_2  = 0x12,
    PB_3  = 0x13,
    PB_4  = 0x14,
    PB_5  = 0x15,
    PB_6  = 0x16,
    PB_7  = 0x17,
    PB_8  = 0x18,
    PB_9  = 0x19,
    PB_10 = 0x1A,
    PB_11 = 0x1B,
    PB_12 = 0x1C,
    PB_13 = 0x1D,
    PB_14 = 0x1E,
    PB_15 = 0x1F,
    // GPIOC
    PC_0  = 0x20,
    PC_1  = 0x21,
    PC_2  = 0x22,
    PC_3  = 0x23,
    PC_4  = 0x24,
    PC_5  = 0x25,
    PC_6  = 0x26,
    PC_7  = 0x27,
    PC_8  = 0x28,
    PC_9  = 0x29,
    PC_10 = 0x2A,
    PC_11 = 0x2B,
    PC_12 = 0x2C,
    PC_13 = 0x2D,
    PC_14 = 0x2E,
    PC_15 = 0x2F,
    // GPIOD
    PD_2 = 0x32,
    // GPIOH
    PH_0 = 0x70,
    PH_1 = 0x71,
    // ADC internal channels
    ADC_TEMP = 0xF0,
    ADC_VREF = 0xF1,
    ADC_VBAT = 0xF2,
    // Arduino interface names
    A0  = PA_0,
    A1  = PA_1,
    A2  = PA_4,
    A3  = PB_0,
    A4  = PC_1,
    A5  = PC_0,
    D0  = PA_3,
    D1  = PA_2,
    D2  = PA_10,
    D3  = PB_3,
    D4  = PB_5,
    D5  = PB_4,
    D6  = PB_10,
    D7  = PA_8,
    D8  = PA_9,
    D9  = PC_7,
    D10 = PB_6,
    D11 = PA_7,
    D12 = PA_6,
    D13 = PA_5,
    D14 = PB_9,
    D15 = PB_8,
    // MCU peripherals
    UART1_RX    = PA_10,
    UART1_TX    = PA_9,
    UART2_RX    = PA_3,
    UART2_TX    = PA_2,
    I2C_SCL     = PB_8,
    I2C_SDA     = PB_9,
    SPI1_MOSI   = PA_7,
    SPI1_MISO   = PA_6,
    SPI1_SCLK   = PA_5,
    PWM_OUT     = PB_3,
    OSC_LSE_IN  = PC_14,
    OSC_LSE_OUT = PC_15,
    OSC_HSE_IN  = PH_0,
    OSC_HSE_OUT = PH_1,
    SWCLK       = PA_14,
    SWDAT       = PA_13,
    // Not connected
    NC = -1
} hal_gpio_pin_names_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_GPIO_PIN_NAMES_H__
