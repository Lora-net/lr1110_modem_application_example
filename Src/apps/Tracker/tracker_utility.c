/*!
 * \file      tracker_utility.c
 *
 * \brief     tracker utility implementation.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "lr1110-modem-board.h"
#include "tracker_utility.h"
#include "main_tracker.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * \brief Radio hardware and global parameters
 */
extern lr1110_t lr1110;

/*!
 * \brief Tracker context structure
 */
tracker_ctx_t tracker_ctx;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void tracker_init_app_ctx( uint8_t *dev_eui, uint8_t *join_eui, uint8_t *app_key)
{
    /* LoRaWAN Parameter */
    memcpy(tracker_ctx.dev_eui,dev_eui,8);
    memcpy(tracker_ctx.join_eui,join_eui,8);
    memcpy(tracker_ctx.app_key,app_key,16);

    /* GNSS Parameters */
    tracker_ctx.gnss_settings.enabled                   = true;
    tracker_ctx.gnss_settings.constellation_to_use      = LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK ;
    tracker_ctx.gnss_settings.scan_type                 = ASSISTED_MODE;
    tracker_ctx.gnss_settings.search_mode               = LR1110_MODEM_GNSS_OPTION_DEFAULT;
    /* Set default position to Semtech France */
    tracker_ctx.gnss_settings.assistance_position.latitude  = 45.208;
    tracker_ctx.gnss_settings.assistance_position.longitude = 5.781;

    /* Wi-Fi Parameters */
    tracker_ctx.wifi_settings.enabled       = true;
    tracker_ctx.wifi_settings.channels      = 0x3FFF;  // by default enable all channels
    tracker_ctx.wifi_settings.types         = LR1110_MODEM_WIFI_TYPE_SCAN_B;
    tracker_ctx.wifi_settings.scan_mode     = LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PACKET;
    tracker_ctx.wifi_settings.nbr_retrials  = WIFI_NBR_RETRIALS_DEFAULT;
    tracker_ctx.wifi_settings.max_results   = WIFI_MAX_RESULTS_DEFAULT;
    tracker_ctx.wifi_settings.timeout       = WIFI_TIMEOUT_IN_MS_DEFAULT;
    tracker_ctx.wifi_settings.result_format = LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL;

    /* Application Parameters */
    tracker_ctx.accelerometer_used  = false;
    tracker_ctx.app_scan_interval   = TRACKER_SCAN_INTERVAL;
    tracker_ctx.app_keep_alive_frame_interval  = TRACKER_KEEP_ALIVE_FRAME_INTERVAL;   
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
