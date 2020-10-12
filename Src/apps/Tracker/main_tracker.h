/*!
 * \file      main_tracker.h
 *
 * \brief     lr1110 Modem Tracker Application definition
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

#ifndef __MAIN_TRACKER_H__
#define __MAIN_TRACKER_H__

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
 * \brief Defines the application scan interval
 * when device has moved. 60s, value in [ms].
 */
#define TRACKER_SCAN_INTERVAL 60000

/*!
 * Defines the watchdog timeout application
 * when device has moved. 180s, value in [ms].
 */
#define WATCHDOG_TIMEOUT 180000

/*!
 * \brief Defines the application keep alive frame interval
 * when device doesn't move. 1800s (30min), value in [ms].
 */
#define TRACKER_KEEP_ALIVE_FRAME_INTERVAL 1800000

/*!
 * \brief Defines the application data transmission duty cycle counter.
 * when device doesn't move.
 */
#define TRACKER_APP_TX_LOW_DUTYCYCLE_CTN TRACKER_KEEP_ALIVE_FRAME_INTERVAL / TRACKER_SCAN_INTERVAL

/*!
 * \brief Use or not the LoRaWAN production Keys.
 */
#define USE_PRODUCTION_KEYS 1

/*!
 * \brief Use or not the Semtech join server.
 */
#define USE_SEMTECH_JOIN_SERVER 1

/*!
 * Defines the application firmware version
 */
 
/*!
 * \brief Defines the application firmware version
 */
#define TRACKER_MAJOR_APP_VERSION 1
#define TRACKER_MINOR_APP_VERSION 1
#define TRACKER_SUB_MINOR_APP_VERSION 0


/*!
 * \brief LoRaWAN application tag
 */
#define TAG_NAV 7
#define TAG_WIFI_SCAN 8
#define TAG_ACCELEROMETER 9
#define TAG_CHARGE 10

/*!
 * \brief LoRaWAN stream application port
 */
#define LORAWAN_STREAM_APP_PORT 199

/*!
 * \brief LoRaWAN port used to the gnss push solver messages
 */
#define GNSS_PUSH_SOLVER_MSG_PORT 197

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __MAIN_TRACKER_H__

/* --- EOF ------------------------------------------------------------------ */
