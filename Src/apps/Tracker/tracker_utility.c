/*!
 * @file      tracker_utility.c
 *
 * @brief     tracker utility implementation.
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

#include <time.h>
#include "lorawan_config.h"
#include "lr1110_modem_board.h"
#include "tracker_utility.h"
#include "main_tracker.h"
#include "lorawan_commissioning.h"

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
 * @brief Radio hardware and global parameters
 */
extern lr1110_t lr1110;

/*!
 * @brief Tracker context structure
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

void tracker_init_app_ctx( uint8_t* dev_eui, uint8_t* join_eui, uint8_t* app_key )
{
    /* LoRaWAN Parameter */
    memcpy( tracker_ctx.dev_eui, dev_eui, 8 );
    memcpy( tracker_ctx.join_eui, join_eui, 8 );
    memcpy( tracker_ctx.app_key, app_key, 16 );
    tracker_ctx.lorawan_region          = LORAWAN_REGION_USED;
    tracker_ctx.use_semtech_join_server = USE_SEMTECH_JOIN_SERVER;
    tracker_ctx.lorawan_adr_profile     = LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED;

    /* GNSS Parameters */
    tracker_ctx.gnss_settings.enabled              = true;
    tracker_ctx.gnss_settings.constellation_to_use = LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK;

    tracker_ctx.gnss_settings.scan_type   = ASSISTED_MODE;
    tracker_ctx.gnss_settings.search_mode = LR1110_MODEM_GNSS_OPTION_DEFAULT;
    /* Set default position to Semtech France */
    tracker_ctx.gnss_settings.assistance_position.latitude  = 45.208;
    tracker_ctx.gnss_settings.assistance_position.longitude = 5.781;
    tracker_ctx.gnss_scan_if_wifi_not_good_enough           = false;

    /* Wi-Fi Parameters */
    tracker_ctx.wifi_settings.enabled       = true;
    tracker_ctx.wifi_settings.channels      = 0x3FFF;  // by default enable all channels
    tracker_ctx.wifi_settings.types         = LR1110_MODEM_WIFI_TYPE_SCAN_B;
    tracker_ctx.wifi_settings.scan_mode     = LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PKT;
    tracker_ctx.wifi_settings.nbr_retrials  = WIFI_NBR_RETRIALS_DEFAULT;
    tracker_ctx.wifi_settings.max_results   = WIFI_MAX_RESULTS_DEFAULT;
    tracker_ctx.wifi_settings.timeout       = WIFI_TIMEOUT_IN_MS_DEFAULT;
    tracker_ctx.wifi_settings.result_format = LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL;

    /* Application Parameters */
    tracker_ctx.accelerometer_used            = false;
    tracker_ctx.app_scan_interval             = TRACKER_SCAN_INTERVAL;
    tracker_ctx.app_keep_alive_frame_interval = TRACKER_KEEP_ALIVE_FRAME_INTERVAL;
}

uint8_t tracker_parse_cmd( uint8_t* payload, uint8_t* buffer_out )
{
    uint8_t nb_elements         = 0;
    uint8_t nb_elements_index   = 0;
    uint8_t payload_index       = 0;
    uint8_t output_buffer_index = 1;
    uint8_t tag                 = 0;
    uint8_t len                 = 0;
    uint8_t res_size            = 0;
    bool    reset_board_asked   = false;

    nb_elements = payload[payload_index++];

    buffer_out[0] = 0;  // ensure that byte 0 is set to 0 at the beggining.

    if( nb_elements > 0 )
    {
        while( nb_elements_index < nb_elements )
        {
            tag = payload[payload_index++];
            len = payload[payload_index++];

            switch( tag )
            {
            case GET_FW_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_FW_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_FW_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = TRACKER_MAJOR_APP_VERSION;
                buffer_out[output_buffer_index++] = TRACKER_MINOR_APP_VERSION;
                buffer_out[output_buffer_index++] = TRACKER_SUB_MINOR_APP_VERSION;

                payload_index += GET_FW_VERSION_LEN;
                break;
            }

            case GET_STACK_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_STACK_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_STACK_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.modem_version.lorawan >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.modem_version.lorawan;

                payload_index += GET_STACK_VERSION_LEN;
                break;
            }

            case GET_MODEM_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.modem_version.firmware >> 16;
                buffer_out[output_buffer_index++] = tracker_ctx.modem_version.firmware >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.modem_version.firmware;

                payload_index += GET_MODEM_VERSION_LEN;
                break;
            }

            case GET_MODEM_STATUS_CMD:
            {
                lr1110_modem_status_t modem_status;
                lr1110_modem_get_status( &lr1110, &modem_status );

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_STATUS_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_STATUS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.has_date;
                buffer_out[output_buffer_index++] = modem_status;

                payload_index += GET_MODEM_STATUS_LEN;
                break;
            }

            case GET_MODEM_DATE_CMD:
            {
                uint32_t date = lr1110_modem_board_get_systime_from_gps( &lr1110 );

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_DATE_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_DATE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = date >> 24;
                buffer_out[output_buffer_index++] = date >> 16;
                buffer_out[output_buffer_index++] = date >> 8;
                buffer_out[output_buffer_index++] = date;

                payload_index += GET_MODEM_DATE_LEN;
                break;
            }

            case GET_LORAWAN_PIN_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_PIN_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_PIN_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_pin >> 24;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_pin >> 16;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_pin >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_pin;

                payload_index += GET_LORAWAN_PIN_LEN;
                break;
            }

            case SET_LORAWAN_DEVEUI_CMD:
            {
                memcpy( tracker_ctx.dev_eui, payload + payload_index, SET_LORAWAN_DEVEUI_LEN );
                tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_DEVEUI_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_DEVEUI_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.dev_eui, SET_LORAWAN_DEVEUI_LEN );
                output_buffer_index += SET_LORAWAN_DEVEUI_LEN;

                lr1110_modem_set_dev_eui( &lr1110, tracker_ctx.dev_eui );
                /* do a derive key to have the new pin code */
                lr1110_modem_derive_keys( &lr1110 );
                lr1110_modem_get_pin( &lr1110, &tracker_ctx.lorawan_pin );

                payload_index += SET_LORAWAN_DEVEUI_LEN;
                break;
            }

            case GET_LORAWAN_DEVEUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_DEVEUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_DEVEUI_ANSWER_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.dev_eui, GET_LORAWAN_DEVEUI_ANSWER_LEN );
                output_buffer_index += GET_LORAWAN_DEVEUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_DEVEUI_LEN;
                break;
            }

            case GET_LORAWAN_CHIP_EUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_CHIP_EUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_CHIP_EUI_ANSWER_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.chip_eui, GET_LORAWAN_CHIP_EUI_ANSWER_LEN );
                output_buffer_index += GET_LORAWAN_CHIP_EUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_CHIP_EUI_LEN;
                break;
            }

            case SET_LORAWAN_JOINEUI_CMD:
            {
                memcpy( tracker_ctx.join_eui, payload + payload_index, SET_LORAWAN_JOINEUI_LEN );
                tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_JOINEUI_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_JOINEUI_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.join_eui, SET_LORAWAN_JOINEUI_LEN );
                output_buffer_index += SET_LORAWAN_JOINEUI_LEN;

                /* do a derive key to have the new pin code */
                lr1110_modem_set_join_eui( &lr1110, tracker_ctx.join_eui );
                lr1110_modem_derive_keys( &lr1110 );
                lr1110_modem_get_pin( &lr1110, &tracker_ctx.lorawan_pin );

                payload_index += SET_LORAWAN_JOINEUI_LEN;
                break;
            }

            case GET_LORAWAN_JOINEUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOINEUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOINEUI_ANSWER_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.join_eui, GET_LORAWAN_JOINEUI_ANSWER_LEN );
                output_buffer_index += GET_LORAWAN_JOINEUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_JOINEUI_LEN;
                break;
            }

            case SET_LORAWAN_APPKEY_CMD:
            {
                memcpy( tracker_ctx.app_key, payload + payload_index, SET_LORAWAN_APPKEY_LEN );
                tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_APPKEY_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_APPKEY_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.app_key, SET_LORAWAN_APPKEY_LEN );
                output_buffer_index += SET_LORAWAN_APPKEY_LEN;

                payload_index += SET_LORAWAN_APPKEY_LEN;
                break;
            }

            case GET_LORAWAN_APPKEY_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_APPKEY_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_APPKEY_ANSWER_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.app_key, GET_LORAWAN_APPKEY_ANSWER_LEN );
                output_buffer_index += GET_LORAWAN_APPKEY_ANSWER_LEN;

                payload_index += GET_LORAWAN_APPKEY_LEN;
                break;
            }

            case GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_CMD:
            {
                uint16_t nb_uplink_mobile_static;
                uint16_t nb_uplink_reset;
                lr1110_modem_get_connection_timeout_status( &lr1110, &nb_uplink_mobile_static, &nb_uplink_reset );

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_ANSWER_LEN;
                buffer_out[output_buffer_index++] = nb_uplink_reset >> 8;
                buffer_out[output_buffer_index++] = nb_uplink_reset;

                payload_index += GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_LEN;
                break;
            }

            case SET_GNSS_ENABLE_CMD:
            {
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.gnss_settings.enabled = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_ENABLE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_ENABLE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.enabled;
                }
                else
                {
                    tracker_ctx.gnss_settings.enabled = 1;
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_ENABLE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_ENABLE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.enabled;
                }
                payload_index += SET_GNSS_ENABLE_LEN;
                break;
            }

            case GET_GNSS_ENABLE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_ENABLE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_ENABLE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.enabled;

                payload_index += GET_GNSS_ENABLE_LEN;
                break;
            }

            case SET_GNSS_CONSTELLATION_CMD:
            {
                if( payload[payload_index] <= 2 )
                {
                    tracker_ctx.gnss_settings.constellation_to_use = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.constellation_to_use;
                }
                else
                {
                    tracker_ctx.gnss_settings.constellation_to_use = 3;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.constellation_to_use;
                }

                payload_index += SET_GNSS_CONSTELLATION_LEN;
                break;
            }

            case GET_GNSS_CONSTELLATION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_CONSTELLATION_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_CONSTELLATION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.constellation_to_use;

                payload_index += GET_GNSS_CONSTELLATION_LEN;
                break;
            }

            case SET_GNSS_SCAN_TYPE_CMD:
            {
                if( ( payload[payload_index] >= 1 ) && ( payload[payload_index] <= 3 ) )
                {
                    tracker_ctx.gnss_settings.scan_type = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_SCAN_TYPE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_SCAN_TYPE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.scan_type;
                }
                else
                {
                    /* NAck the CMD */
                    /* Clip the value */
                    if( payload[payload_index] < 1 )
                    {
                        tracker_ctx.gnss_settings.scan_type = 1;
                    }
                    else
                    {
                        tracker_ctx.gnss_settings.scan_type = 3;
                    }
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_SCAN_TYPE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_SCAN_TYPE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.scan_type;
                }
                payload_index += SET_GNSS_SCAN_TYPE_LEN;
                break;
            }

            case GET_GNSS_SCAN_TYPE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_SCAN_TYPE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_SCAN_TYPE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.scan_type;

                payload_index += GET_GNSS_SCAN_TYPE_LEN;
                break;
            }

            case SET_GNSS_SEARCH_MODE_CMD:
            {
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.gnss_settings.search_mode = ( lr1110_modem_gnss_search_mode_t ) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_SEARCH_MODE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_SEARCH_MODE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.search_mode;
                }
                else
                {
                    tracker_ctx.gnss_settings.search_mode = ( lr1110_modem_gnss_search_mode_t ) 1;
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_SEARCH_MODE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_SEARCH_MODE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.search_mode;
                }

                payload_index += SET_GNSS_SEARCH_MODE_LEN;
                break;
            }

            case GET_GNSS_SEARCH_MODE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_SEARCH_MODE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_SEARCH_MODE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.search_mode;

                payload_index += GET_GNSS_SEARCH_MODE_LEN;
                break;
            }

            case GET_GNSS_LAST_ALMANAC_UPDATE_CMD:
            {
                uint32_t oldest_almanac_date = 0;
                uint32_t newest_almanac_date = 0;

                /* get the dates form the Modem-E */
                lr1110_modem_get_almanac_dates( &lr1110, &oldest_almanac_date, &newest_almanac_date );

                if( oldest_almanac_date > tracker_ctx.last_almanac_update )
                {
                    tracker_ctx.last_almanac_update = oldest_almanac_date;
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_ALMANAC_UPDATE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_ALMANAC_UPDATE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.last_almanac_update >> 24;
                buffer_out[output_buffer_index++] = tracker_ctx.last_almanac_update >> 16;
                buffer_out[output_buffer_index++] = tracker_ctx.last_almanac_update >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.last_almanac_update;

                payload_index += GET_GNSS_LAST_ALMANAC_UPDATE_LEN;
                break;
            }

            case GET_GNSS_LAST_NB_SV_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_NB_SV_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_NB_SV_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.last_nb_detected_satellites;

                payload_index += GET_GNSS_LAST_NB_SV_LEN;
                break;
            }

            case SET_WIFI_ENABLE_CMD:
            {
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.wifi_settings.enabled = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_ENABLE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_ENABLE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.enabled;
                }
                else
                {
                    tracker_ctx.wifi_settings.enabled = 1;
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_ENABLE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_ENABLE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.enabled;
                }

                payload_index += SET_WIFI_ENABLE_LEN;
                break;
            }

            case GET_WIFI_ENABLE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_ENABLE_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_ENABLE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.enabled;

                payload_index += GET_WIFI_ENABLE_LEN;
                break;
            }

            case SET_WIFI_CHANNELS_CMD:
            {
                uint16_t wifi_channels;

                wifi_channels = ( uint16_t ) payload[payload_index] << 8;
                wifi_channels += payload[payload_index + 1];

                if( wifi_channels <= 0x3FFF )
                {
                    tracker_ctx.wifi_settings.channels = wifi_channels;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_CHANNELS_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_CHANNELS_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.channels >> 8;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.channels;
                }
                else
                {
                    tracker_ctx.wifi_settings.channels = 0x3FFF;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_CHANNELS_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_CHANNELS_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.channels >> 8;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.channels;
                }

                payload_index += SET_WIFI_CHANNELS_LEN;
                break;
            }

            case GET_WIFI_CHANNELS_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_CHANNELS_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_CHANNELS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.channels >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.channels;

                payload_index += GET_WIFI_CHANNELS_LEN;
                break;
            }

            case SET_WIFI_TYPE_CMD:
            {
                if( ( payload[payload_index] >= 1 ) && ( payload[payload_index] <= 2 ) )
                {
                    tracker_ctx.wifi_settings.types = ( lr1110_modem_wifi_signal_type_scan_t ) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_TYPE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_TYPE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.types;
                }
                else
                {
                    /* Clip the value */
                    if( payload[payload_index] < 1 )
                    {
                        tracker_ctx.wifi_settings.types = ( lr1110_modem_wifi_signal_type_scan_t ) 1;
                    }
                    else
                    {
                        tracker_ctx.wifi_settings.types = ( lr1110_modem_wifi_signal_type_scan_t ) 2;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_TYPE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_TYPE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.types;
                }
                payload_index += SET_WIFI_TYPE_LEN;
                break;
            }

            case GET_WIFI_TYPE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_TYPE_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_TYPE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.types;

                payload_index += GET_WIFI_TYPE_LEN;
                break;
            }

            case SET_WIFI_SCAN_MODE_CMD:
            {
                if( ( payload[payload_index] >= 1 ) && ( payload[payload_index] <= 2 ) )
                {
                    tracker_ctx.wifi_settings.scan_mode = ( lr1110_modem_wifi_mode_t ) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_SCAN_MODE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_SCAN_MODE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.scan_mode;
                }
                else
                {
                    /* Clip the value */
                    if( payload[payload_index] < 1 )
                    {
                        tracker_ctx.wifi_settings.scan_mode = ( lr1110_modem_wifi_mode_t ) 1;
                    }
                    else
                    {
                        tracker_ctx.wifi_settings.scan_mode = ( lr1110_modem_wifi_mode_t ) 2;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_SCAN_MODE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_SCAN_MODE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.scan_mode;
                }
                payload_index += SET_WIFI_SCAN_MODE_LEN;
                break;
            }

            case GET_WIFI_SCAN_MODE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_SCAN_MODE_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_SCAN_MODE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.scan_mode;

                payload_index += GET_WIFI_SCAN_MODE_LEN;
                break;
            }

            case SET_WIFI_RETRIALS_CMD:
            {
                tracker_ctx.wifi_settings.nbr_retrials = payload[payload_index];

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_WIFI_RETRIALS_CMD;
                buffer_out[output_buffer_index++] = SET_WIFI_RETRIALS_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.nbr_retrials;

                payload_index += SET_WIFI_RETRIALS_LEN;
                break;
            }

            case GET_WIFI_RETRIALS_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_RETRIALS_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_RETRIALS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.nbr_retrials;

                payload_index += GET_WIFI_RETRIALS_LEN;
                break;
            }

            case SET_WIFI_MAX_RESULTS_CMD:
            {
                if( ( payload[payload_index] >= 1 ) && ( payload[payload_index] <= 32 ) )
                {
                    tracker_ctx.wifi_settings.max_results = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_MAX_RESULTS_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_MAX_RESULTS_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.max_results;
                }
                else
                {
                    /* Clip the value */
                    if( payload[payload_index] < 1 )
                    {
                        tracker_ctx.wifi_settings.max_results = 1;
                    }
                    else
                    {
                        tracker_ctx.wifi_settings.max_results = 32;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_MAX_RESULTS_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_MAX_RESULTS_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.max_results;
                }

                payload_index += SET_WIFI_MAX_RESULTS_LEN;
                break;
            }

            case GET_WIFI_MAX_RESULTS_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_MAX_RESULTS_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_MAX_RESULTS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.max_results;

                payload_index += GET_WIFI_MAX_RESULTS_LEN;
                break;
            }

            case SET_WIFI_TIMEOUT_CMD:
            {
                uint16_t wifi_timeout = 0;

                wifi_timeout = ( uint16_t ) payload[payload_index] << 8;
                wifi_timeout += payload[payload_index + 1];

                if( ( wifi_timeout >= 20 ) && ( wifi_timeout <= 5000 ) )
                {
                    tracker_ctx.wifi_settings.timeout = wifi_timeout;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_TIMEOUT_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_TIMEOUT_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.timeout >> 8;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.timeout;
                }
                else
                {
                    /* Clip the value */
                    if( payload[payload_index] < 20 )
                    {
                        tracker_ctx.wifi_settings.timeout = 20;
                    }
                    else
                    {
                        tracker_ctx.wifi_settings.timeout = 5000;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_TIMEOUT_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_TIMEOUT_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.timeout >> 8;
                    buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.timeout;
                }
                payload_index += SET_WIFI_TIMEOUT_LEN;
                break;
            }

            case GET_WIFI_TIMEOUT_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_TIMEOUT_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_TIMEOUT_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.timeout >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.timeout;

                payload_index += GET_WIFI_TIMEOUT_LEN;
                break;
            }

            case GET_WIFI_LAST_NB_MAC_ADDRESS_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_LAST_NB_MAC_ADDRESS_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_LAST_NB_MAC_ADDRESS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.last_nb_detected_mac_address;

                payload_index += GET_WIFI_LAST_NB_MAC_ADDRESS_LEN;
                break;
            }

            case SET_USE_ACCELEROMETER_CMD:
            {
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.accelerometer_used = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_CMD;
                    buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.accelerometer_used;
                }
                else
                {
                    /* Clip the value */
                    tracker_ctx.accelerometer_used = 1;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_CMD;
                    buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.accelerometer_used;
                }

                payload_index += SET_USE_ACCELEROMETER_LEN;
                break;
            }

            case GET_USE_ACCELEROMETER_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_USE_ACCELEROMETER_CMD;
                buffer_out[output_buffer_index++] = GET_USE_ACCELEROMETER_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.accelerometer_used;

                payload_index += GET_USE_ACCELEROMETER_LEN;
                break;
            }

            case SET_APP_SCAN_INTERVAL_CMD:
            {
                uint16_t app_duty_cycle = 0;

                app_duty_cycle = ( uint16_t ) payload[payload_index] << 8;
                app_duty_cycle += payload[payload_index + 1];

                if( ( app_duty_cycle >= 10 ) && ( app_duty_cycle <= 1800 ) )
                {
                    tracker_ctx.app_scan_interval = app_duty_cycle * 1000;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_SCAN_INTERVAL_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_SCAN_INTERVAL_LEN;
                    buffer_out[output_buffer_index++] = ( tracker_ctx.app_scan_interval / 1000 ) >> 8;
                    buffer_out[output_buffer_index++] = ( tracker_ctx.app_scan_interval / 1000 );
                }
                else
                {
                    /* Clip the value */
                    if( payload[payload_index] < 10 )
                    {
                        tracker_ctx.app_scan_interval = 10 * 1000;
                    }
                    else
                    {
                        tracker_ctx.app_scan_interval = 1800 * 1000;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_SCAN_INTERVAL_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_SCAN_INTERVAL_LEN;
                    buffer_out[output_buffer_index++] = ( tracker_ctx.app_scan_interval / 1000 ) >> 8;
                    buffer_out[output_buffer_index++] = ( tracker_ctx.app_scan_interval / 1000 );
                }

                payload_index += SET_APP_SCAN_INTERVAL_LEN;
                break;
            }

            case GET_APP_SCAN_INTERVAL_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_SCAN_INTERVAL_CMD;
                buffer_out[output_buffer_index++] = GET_APP_SCAN_INTERVAL_ANSWER_LEN;
                buffer_out[output_buffer_index++] = ( tracker_ctx.app_scan_interval / 1000 ) >> 8;
                buffer_out[output_buffer_index++] = ( tracker_ctx.app_scan_interval / 1000 );

                payload_index += GET_APP_SCAN_INTERVAL_LEN;
                break;
            }

            case SET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD:
            {
                uint16_t app_low_duty_cycle = 0;

                app_low_duty_cycle = ( uint16_t ) payload[payload_index] << 8;
                app_low_duty_cycle += payload[payload_index + 1];

                if( ( app_low_duty_cycle >= 10 ) && ( app_low_duty_cycle <= 1440 ) )
                {
                    tracker_ctx.app_keep_alive_frame_interval = app_low_duty_cycle * 60 * 1000;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN;
                    buffer_out[output_buffer_index++] = ( ( tracker_ctx.app_keep_alive_frame_interval / 60000 ) >> 8 );
                    buffer_out[output_buffer_index++] = ( tracker_ctx.app_keep_alive_frame_interval / 60000 );
                }
                else
                {
                    /* Clip the value */
                    if( payload[payload_index] < 10 )
                    {
                        tracker_ctx.app_keep_alive_frame_interval = 10 * 60 * 1000;
                    }
                    else
                    {
                        tracker_ctx.app_keep_alive_frame_interval = 1440 * 60 * 1000;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN;
                    buffer_out[output_buffer_index++] = ( ( tracker_ctx.app_keep_alive_frame_interval / 60000 ) >> 8 );
                    buffer_out[output_buffer_index++] = ( tracker_ctx.app_keep_alive_frame_interval / 60000 );
                }

                payload_index += SET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN;
                break;
            }

            case GET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD;
                buffer_out[output_buffer_index++] = GET_APP_KEEP_ALINE_FRAME_INTERVAL_ANSWER_LEN;
                buffer_out[output_buffer_index++] = ( ( tracker_ctx.app_keep_alive_frame_interval / 60000 ) >> 8 );
                buffer_out[output_buffer_index++] = ( tracker_ctx.app_keep_alive_frame_interval / 60000 );

                payload_index += GET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN;
                break;
            }

            case SET_LORAWAN_REGION_CMD:
            {
                if( ( payload[payload_index] == 1 ) || ( payload[payload_index] == 3 ) )
                {
                    tracker_ctx.lorawan_region                  = ( lr1110_modem_regions_t ) payload[payload_index];
                    tracker_ctx.lorawan_parameters_have_changed = true;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.lorawan_region;
                }
                else
                {
                    /* Don't change the value of the region in this case */
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.lorawan_region;
                }

                payload_index += SET_LORAWAN_REGION_LEN;
                break;
            }

            case GET_LORAWAN_REGION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_REGION_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_REGION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_region;

                payload_index += GET_LORAWAN_REGION_LEN;
                break;
            }

            case SET_LORAWAN_JOIN_SERVER_CMD:
            {
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.use_semtech_join_server         = payload[payload_index];
                    tracker_ctx.lorawan_parameters_have_changed = true;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.use_semtech_join_server;
                }
                else
                {
                    /* Don't change the value of the usage of the usage of join server in this case */
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.use_semtech_join_server;
                }

                payload_index += SET_LORAWAN_JOIN_SERVER_LEN;
                break;
            }

            case GET_LORAWAN_JOIN_SERVER_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOIN_SERVER_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOIN_SERVER_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.use_semtech_join_server;

                payload_index += GET_LORAWAN_JOIN_SERVER_LEN;
                break;
            }

            case SET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_CMD:
            {
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.gnss_scan_if_wifi_not_good_enough = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_scan_if_wifi_not_good_enough;
                }
                else
                {
                    /* Clip the value */
                    tracker_ctx.gnss_scan_if_wifi_not_good_enough = 0;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.gnss_scan_if_wifi_not_good_enough;
                }

                payload_index += SET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_LEN;
                break;
            }

            case GET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_scan_if_wifi_not_good_enough;

                payload_index += GET_GNSS_ONLY_IF_WIFI_GOOD_ENOUGH_LEN;
                break;
            }

            case SET_LORAWAN_ADR_PROFILE_CMD:
            {
                if( payload[payload_index] <= 3 )
                {
                    tracker_ctx.lorawan_adr_profile = ( lr1110_modem_adr_profiles_t ) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_ADR_PROFILE_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_ADR_PROFILE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.lorawan_adr_profile;
                }
                else
                {
                    /* Clip the value */
                    tracker_ctx.lorawan_adr_profile = LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_ADR_PROFILE_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_ADR_PROFILE_LEN;
                    buffer_out[output_buffer_index++] = tracker_ctx.lorawan_adr_profile;
                }

                payload_index += SET_LORAWAN_ADR_PROFILE_LEN;
                break;
            }

            case GET_LORAWAN_ADR_PROFILE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_ADR_PROFILE_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_ADR_PROFILE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_adr_profile;

                payload_index += GET_LORAWAN_ADR_PROFILE_LEN;
                break;
            }

            case GET_APP_ACCUMULATED_CHARGE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_ACCUMULATED_CHARGE_CMD;
                buffer_out[output_buffer_index++] = GET_APP_ACCUMULATED_CHARGE_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.charge >> 24;
                buffer_out[output_buffer_index++] = tracker_ctx.charge >> 16;
                buffer_out[output_buffer_index++] = tracker_ctx.charge >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.charge;

                payload_index += GET_APP_ACCUMULATED_CHARGE_ANSWER_LEN;
                break;
            }

            case RESET_APP_ACCUMULATED_CHARGE_CMD:
            {
                tracker_ctx.charge = 0;

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = RESET_APP_ACCUMULATED_CHARGE_CMD;
                buffer_out[output_buffer_index++] = RESET_APP_ACCUMULATED_CHARGE_LEN;

                payload_index += RESET_APP_ACCUMULATED_CHARGE_LEN;
                break;
            }
            case GET_APP_SYSTEM_SANITY_CHECK_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_SYSTEM_SANITY_CHECK_CMD;
                buffer_out[output_buffer_index++] = GET_APP_SYSTEM_SANITY_CHECK_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.system_sanity_check;

                payload_index += GET_APP_SYSTEM_SANITY_CHECK_LEN;
                break;
            }

            case GET_APP_TRACKER_SETTINGS_CMD:
            {
                uint32_t oldest_almanac_date = 0;
                uint32_t newest_almanac_date = 0;
                uint16_t nb_uplink_mobile_static;
                uint16_t nb_uplink_reset;

                /* get the dates form the Modem-E */
                lr1110_modem_get_almanac_dates( &lr1110, &oldest_almanac_date, &newest_almanac_date );
                lr1110_modem_get_connection_timeout_status( &lr1110, &nb_uplink_mobile_static, &nb_uplink_reset );

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_CMD;
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_ANSWER_LEN;

                /* All settings pattern version */
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_VERSION;
                /* Modem-E firmware version */
                buffer_out[output_buffer_index++] = tracker_ctx.modem_version.firmware >> 16;
                buffer_out[output_buffer_index++] = tracker_ctx.modem_version.firmware >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.modem_version.firmware;
                /* LoRaWAN ADR profile */
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_adr_profile;
                /* LoRaWAN Join Sever usage */
                buffer_out[output_buffer_index++] = tracker_ctx.use_semtech_join_server;
                /* GNSS enable */
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.enabled;
                /* GNSS Antenna sel */
                buffer_out[output_buffer_index++] = 0;
                /* GNSS Assistance position */
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_settings.assistance_position.latitude * 10000000 ) ) >> 24;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_settings.assistance_position.latitude * 10000000 ) ) >> 16;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_settings.assistance_position.latitude * 10000000 ) ) >> 8;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_settings.assistance_position.latitude * 10000000 ) );
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_settings.assistance_position.longitude * 10000000 ) ) >> 24;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_settings.assistance_position.longitude * 10000000 ) ) >> 16;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_settings.assistance_position.longitude * 10000000 ) ) >> 8;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_settings.assistance_position.longitude * 10000000 ) );
                /* GNSS Constellation to use */
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.constellation_to_use;
                /* GNSS Search mode */
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_settings.search_mode;
                /* GNSS oldest almanac */
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 24;
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 16;
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 8;
                buffer_out[output_buffer_index++] = oldest_almanac_date;
                /* GNSS newest almanac */
                buffer_out[output_buffer_index++] = newest_almanac_date >> 24;
                buffer_out[output_buffer_index++] = newest_almanac_date >> 16;
                buffer_out[output_buffer_index++] = newest_almanac_date >> 8;
                buffer_out[output_buffer_index++] = newest_almanac_date;
                /* Perform GNSS scan only if Wi-Fi scan is not good enough */
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_scan_if_wifi_not_good_enough;
                /* Wi-Fi enable */
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.enabled;
                /* Wi-Fi channles */
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.channels >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.channels;
                /* Wi-Fi max results */
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.max_results;
                /* Wi-Fi scan mode */
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.scan_mode;
                /* Wi-Fi timeout */
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.timeout >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.timeout;
                /* Wi-Fi types */
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.types;
                /* Wi-Fi nbr_retrials */
                buffer_out[output_buffer_index++] = tracker_ctx.wifi_settings.nbr_retrials;
                /* App accelerometer used */
                buffer_out[output_buffer_index++] = tracker_ctx.accelerometer_used;
                /* App scan interval */
                buffer_out[output_buffer_index++] = ( tracker_ctx.app_scan_interval / 1000 ) >> 8;
                buffer_out[output_buffer_index++] = ( tracker_ctx.app_scan_interval / 1000 );
                /* App keep alive interval */
                buffer_out[output_buffer_index++] = ( ( tracker_ctx.app_keep_alive_frame_interval / 60000 ) >> 8 );
                buffer_out[output_buffer_index++] = ( tracker_ctx.app_keep_alive_frame_interval / 60000 );
                /* App internal log enable */
                buffer_out[output_buffer_index++] = 0;
                /* App internal log remaining memory space */
                buffer_out[output_buffer_index++] = 0;
                /* App Reset counter */
                buffer_out[output_buffer_index++] = 0;
                buffer_out[output_buffer_index++] = 0;
                buffer_out[output_buffer_index++] = 0;
                buffer_out[output_buffer_index++] = 0;
                /* Last sv number detected */
                buffer_out[output_buffer_index++] = nb_uplink_reset >> 8;
                buffer_out[output_buffer_index++] = nb_uplink_reset;
                /* Last sv number detected */
                buffer_out[output_buffer_index++] = tracker_ctx.last_nb_detected_satellites;
                /* Last mac address number detected */
                buffer_out[output_buffer_index++] = tracker_ctx.last_nb_detected_mac_address;
                /* Sanity check bit mask */
                buffer_out[output_buffer_index++] = tracker_ctx.system_sanity_check;

                payload_index += GET_APP_TRACKER_SETTINGS_LEN;

                break;
            }

            case SET_APP_RESET_CMD:
            {
                reset_board_asked = true;
                payload_index += SET_APP_RESET_LEN;

                break;
            }

            default:
                payload_index += len;

                break;
            }

            nb_elements_index++;
        }
    }

    if( reset_board_asked == true )
    {
        hal_mcu_reset( );
    }

    if( output_buffer_index > 1 )  // if > 1 it means there is something to send
    {
        res_size = output_buffer_index;
    }

    return res_size;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
