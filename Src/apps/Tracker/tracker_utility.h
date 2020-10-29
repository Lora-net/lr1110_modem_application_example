/*!
 * \file      tracker_utility.h
 *
 * \brief     Tracker utility definition
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

#ifndef __TRACKER_UTILITY_H__
#define __TRACKER_UTILITY_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>
#include <stdbool.h>
#include "wifi_scan.h"
#include "gnss_scan.h"
#include "lr1110_modem_lorawan.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define WIFI_SINGLE_BEACON_LEN              0x07
 
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct
{
    /* Time variables */
    uint32_t timestamp;
    bool     has_date; /* Indicates if a date has been set */

    /* LoRaWAN Parameters */
    uint8_t  dev_eui[8];
    uint8_t  join_eui[8];
    uint8_t  app_key[16];
    uint8_t  chip_eui[8];
    uint32_t lorawan_pin;

    /* Modem version information */
    lr1110_modem_version_t modem_version;

    /* GNSS Parameters */
    gnss_settings_t gnss_settings;

    /* WiFi Parameters */
    wifi_settings_t wifi_settings;

    /* Application Parameters */
    bool     accelerometer_used;
    uint32_t app_scan_interval;
    uint32_t app_keep_alive_frame_interval;
    uint8_t  accelerometer_move_history;
    bool     send_alive_frame;
    bool     stream_done;

    /* Results values */
    uint16_t               lorawan_payload_len;
    uint8_t                lorawan_payload[500];
    uint32_t               next_frame_ctn;
    uint16_t nav_message_len;
    uint8_t nav_message[259];
    uint8_t nb_detected_satellites;
    wifi_scan_all_result_t wifi_result;
    int16_t                accelerometer_x;
    int16_t                accelerometer_y;
    int16_t                accelerometer_z;
    int16_t                tout;
    uint32_t               charge;
}tracker_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Init the Tracker context
 *
 * \param [in] dev_eui LoRaWAN Device Eui
 * \param [in] join_eui LoRaWAN Join Eui
 * \param [in] app_key LoRaWAN Application Key
 */
void tracker_init_app_ctx( uint8_t *dev_eui, uint8_t *join_eui, uint8_t *app_key );

#ifdef __cplusplus
}
#endif

#endif  // __TRACKER_UTILITY_H__

/* --- EOF ------------------------------------------------------------------ */
