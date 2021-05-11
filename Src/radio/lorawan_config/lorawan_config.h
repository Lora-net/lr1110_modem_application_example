/*!
 * @file      lorawan_config.h
 *
 * @brief     LoRaWAN configuration definition
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

#ifndef LORAWAN_CONFIG_H
#define LORAWAN_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr1110_modem_board.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief LoRaWAN ETSI duty cycle control enable/disable
 *
 * @remark Please note that ETSI mandates duty cycled transmissions. Set to false only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON LR1110_MODEM_DUTY_CYCLE_ENABLE

/*!
 * @brief LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON false

#if defined( USE_LORAWAN_CLASS_A )
    #define LORAWAN_CLASS_USED LR1110_LORAWAN_CLASS_A
#elif defined( USE_LORAWAN_CLASS_C )
    #define LORAWAN_CLASS_USED LR1110_LORAWAN_CLASS_C
#else
    #define LORAWAN_CLASS_USED LR1110_LORAWAN_CLASS_A
#endif

/*!
 * @brief Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED
    
/*!
 * @brief LoRaWAN confirmed messages
 */   
#if defined( USE_REGION_EU868 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_EU868
#elif defined( USE_REGION_US915 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_US915
#elif defined( USE_REGION_AU915 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_AU915
#elif defined( USE_REGION_AS923_GRP1 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_AS923_GRP1
#elif defined( USE_REGION_CN470 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_CN470
#elif defined( USE_REGION_AS923_GRP2 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_AS923_GRP2
#elif defined( USE_REGION_AS923_GRP3 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_AS923_GRP3
#elif defined( USE_REGION_IN865 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_IN865
 #elif defined( USE_REGION_KR920 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_KR920
#elif defined( USE_REGION_RU864 )
    #define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_RU864
#else
    #error No region selected: define a region
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Lorawan default init
 *
 * @param [in] region LoRaWAN region to use \ref lr1110_modem_regions_t
 * @param [in] lorawan_class LoRaWAN class to use \ref lr1110_modem_classes_t
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lorawan_init( lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class );

#ifdef __cplusplus
}
#endif

#endif  // LORAWAN_CONFIG_H

/* --- EOF ------------------------------------------------------------------ */
