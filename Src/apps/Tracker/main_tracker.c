/*!
 * @file      main_tracker.c
 *
 * @brief     lr1110 Modem Tracker Application implementation
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

#include <stdio.h>
#include <time.h>
#include "main_tracker.h"
#include "lr1110_modem_board.h"
#include "lorawan_commissioning.h"
#include "lorawan_config.h"
#include "wifi_scan.h"
#include "gnss_scan.h"
#include "tracker_utility.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief Default datarate when device is static
 */
#define LORAWAN_STATIC_DATARATE LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED

/*!
 * @brief Default datarate when device is mobile
 */
#define LORAWAN_MOBILE_DATARATE LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER

/*!
 * @brief Use the accelerometer movement in application
 *
 * @remark see ACCELEROMETER_MOUNTED definition
 */
#define USE_ACCELEROMETER false

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Tracker context structure
 */
extern tracker_ctx_t tracker_ctx;

/*!
 * @brief Radio hardware and global parameters
 */
extern lr1110_t lr1110;

/*!
 * @brief ADR custom list when ADR is set to custom
 */
uint8_t adr_custom_list[16] = { 0x05, 0x05, 0x05, 0x04, 0x04, 0x04, 0x03, 0x03,
                                0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00 };

/*!
 * @brief Timer to handle the state of LED TX
 */
static timer_event_t led_tx_timer;

/*!
 * @brief Timer to handle the state of LED RX
 */
static timer_event_t led_rx_timer;

/*!
 * @brief Stream done counter \note only for trace/debug
 */
static uint32_t stream_cnt = 0;

/*!
 * @brief Downlink counter \note only for trace/debug
 */
static uint32_t downlink_cnt = 0;

/*!
 * @brief Device states
 */
static enum edevice_state {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_COLLECT_DATA,
    DEVICE_STATE_SLEEP
} device_state = DEVICE_STATE_INIT;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Prints the provided buffer in HEX
 *
 * @param [in] buffer Buffer to be printed
 * @param [in] size Buffer size to be printed
 */
static void print_hex_buffer( const uint8_t* buffer, uint8_t size );

/*!
 * @brief Prints the LoRaWAN keys
 *
 * @param [in] dev_eui Device EUI to be printed
 * @param [in] join_eui Join EUI to be printed
 * @param [in] app_key Application Key to be printed
 * @param [in] pin pin code
 */
static void print_lorawan_keys( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key,
                                const uint32_t pin );

/*!
 * @brief Executes the network Join request
 */
static void join_network( void );

/*!
 * @brief   add payload of the frame
 *
 * @returns  [true : payload could be add, false : error]
 */
static bool add_payload_in_streaming_fifo( uint8_t* payload, uint16_t len );

/*!
 * @brief   Check if the next scan is possible
 *
 * @returns  [true : scan is possible, false : scan is not possible]
 */
static bool is_next_scan_possible( void );

/*!
 * @brief   Check if the tracker is in static mode
 *
 * @returns  [true : tracker is static, false : tracker in movement]
 */
static bool is_tracker_in_static_mode( void );

/*!
 * @brief Function executed on Led TX Timeout event
 */
static void on_led_tx_timer_event( void* context );

/*!
 * @brief Function executed on Led RX Timeout event
 */
static void on_led_rx_timer_event( void* context );

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*!
 * @brief Network Joined event callback
 */
static void lr1110_modem_network_joined( void );

/*!
 * @brief Join Fail event callback
 */
static void lr1110_modem_join_fail( void );

/*!
 * @brief Alarm event callback
 */
static void lr1110_modem_alarm( void );

/*!
 * @brief Down data event callback.
 *
 * @param [in] rssi    rssi in signed value in dBm + 64
 * @param [in] snr     snr signed value in 0.25 dB steps
 * @param [in] flags   rx flags \see down_data_flag_t
 * @param [in] port    LoRaWAN port
 * @param [in] payload Received buffer pointer
 * @param [in] size    Received buffer size
 */
static void lr1110_modem_down_data( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port,
                                    const uint8_t* payload, uint8_t size );

/*!
 * @brief Stream done event callback
 *
 * @param [in] mute    modem mute status \ref lr1110_modem_mute_t
 */
static void lr1110_modem_mute( lr1110_modem_mute_t mute );

/*!
 * @brief Set conf event callback
 *
 * @param [in] info_tag    modem mute status \ref lr1110_modem_mute_t
 */
static void lr1110_modem_set_conf( uint8_t info_tag );

/*!
 * @brief Stream done event callback
 */
static void lr1110_modem_stream_done( void );

/*!
 * @brief Time updated by application layer clock synchronisation event callback
 */
static void lr1110_modem_time_updated_alc_sync( lr1110_modem_alc_sync_state_t alc_sync_state );

/*!
 * @brief Automatic switch from mobile to static ADR when connection timeout occurs event callback
 */
static void lr1110_modem_adr_mobile_to_static( void );

/*!
 * @brief New link ADR request event callback
 */
static void lr1110_modem_new_link_adr( void );

/*!
 * @brief No event exists event callback
 */
static void lr1110_modem_no_event( void );

/*!
 * @brief Parse the received downlink
 *
 * @param [in] port LoRaWAN port
 * @param [in] payload Payload Buffer
 * @param [in] size Payload size
 */
static void parse_frame( uint8_t port, const uint8_t* payload, uint8_t size );

/*!
 * @brief GNSS default init
 */
static lr1110_modem_response_code_t gnss_init( void );

/*!
 * @brief Convert lr1110 modem status to string
 *
 * @param [in] modem_status modem status \ref lr1110_modem_status_t
 */
static void modem_status_to_string( lr1110_modem_status_t modem_status );

/*!
 * @brief Run and call the necessary functions for the Wi-Fi scan
 *
 * @param [in] wifi_settings Wi-Fi settings used for the scan \ref wifi_settings_t
 * @param [out] wifi_result Wi-Fi result structure where are stored the results
 */
static void tracker_run_wifi_scan( const wifi_settings_t* wifi_settings, wifi_scan_selected_result_t* wifi_result );

/*!
 * @brief Return the number max of sattelites to scan according to the LoRaWAN parameters
 *
 * @returns the number max of sattelites to scan
 */
static uint8_t tracker_get_gnss_next_nb_sat( void );

/*!
 * @brief Run and call the necessary function for the GNSS scan
 *
 * @param [in] gnss_settings GNSS settings used for the scan \ref gnss_settings_t
 * @param [out] capture_result Structure containing the capture result, \ref gnss_scan_single_result_t
 */
static void tracker_run_gnss_scan( const gnss_settings_t* gnss_settings, gnss_scan_single_result_t* capture_result );

/*!
 * @brief build results payload in TLV format and stream it
 *
 * @param [in] payload_mask The mask representing which data to stream, \see tracker_payload_mask_t
 */
static void build_and_stream_payload( tracker_payload_mask_t payload_mask );

/*!
 * @brief build tracker settings payload in TLV format and stream it
 *
 * @param [in] buffer Buffer containing the tracker settings
 * @param [in] len Len of the buffer
 */
static void build_and_stream_tracker_settings( uint8_t* buffer, uint8_t len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_event_t         lr1110_modem_event;

    uint8_t dev_eui[LORAWAN_DEVICE_EUI_LEN] = LORAWAN_DEVICE_EUI;
    uint8_t join_eui[LORAWAN_JOIN_EUI_LEN]  = LORAWAN_JOIN_EUI;
    uint8_t app_key[LORAWAN_APP_KEY_LEN]    = LORAWAN_APP_KEY;

    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Board is initialized */
    leds_blink( LED_ALL_MASK, 100, 2, true );

    HAL_DBG_TRACE_MSG( "\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 Modem Tracker demo application ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "APP VERSION : %d.%d.%d\r\n\r\n", TRACKER_MAJOR_APP_VERSION, TRACKER_MINOR_APP_VERSION,
                          TRACKER_SUB_MINOR_APP_VERSION );

    /* Init LR1110 modem event */
    lr1110_modem_event.reset                 = lr1110_modem_reset_event;
    lr1110_modem_event.alarm                 = lr1110_modem_alarm;
    lr1110_modem_event.joined                = lr1110_modem_network_joined;
    lr1110_modem_event.join_fail             = lr1110_modem_join_fail;
    lr1110_modem_event.down_data             = lr1110_modem_down_data;
    lr1110_modem_event.set_conf              = lr1110_modem_set_conf;
    lr1110_modem_event.mute                  = lr1110_modem_mute;
    lr1110_modem_event.gnss_scan_done        = lr1110_modem_gnss_scan_done;
    lr1110_modem_event.wifi_scan_done        = lr1110_modem_wifi_scan_done;
    lr1110_modem_event.stream_done           = lr1110_modem_stream_done;
    lr1110_modem_event.time_updated_alc_sync = lr1110_modem_time_updated_alc_sync;
    lr1110_modem_event.adr_mobile_to_static  = lr1110_modem_adr_mobile_to_static;
    lr1110_modem_event.new_link_adr          = lr1110_modem_new_link_adr;
    lr1110_modem_event.no_event              = lr1110_modem_no_event;

    if( lr1110_modem_board_init( &lr1110, &lr1110_modem_event ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LR1110 BOARD INIT FAIL ==== ######\r\n\r\n" );
    }

    /* LR1110 modem version */
    lr1110_modem_get_version( &lr1110, &tracker_ctx.modem_version );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM VERSION ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", tracker_ctx.modem_version.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", tracker_ctx.modem_version.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n", tracker_ctx.modem_version.bootloader );

#if( USE_PRODUCTION_KEYS == 1 )
    modem_response_code = lr1110_modem_get_dev_eui( &lr1110, dev_eui );
    modem_response_code = lr1110_modem_get_join_eui( &lr1110, join_eui );
#endif

    /* Init the LoRaWAN keys set in Commissioning_tracker_ctx.h or using the production keys and init the global
     * context */
    tracker_init_app_ctx( dev_eui, join_eui, app_key );

    /* Basic LoRaWAN configuration */
    if( lorawan_init( LORAWAN_REGION_USED, LORAWAN_CLASS_USED ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LORAWAN INIT ERROR ==== ######\r\n\r\n" );
    }

    if( gnss_init( ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== GNSS INIT ERROR ==== ######\r\n\r\n" );
    }

    /* Set Keys */
    modem_response_code = lr1110_modem_set_dev_eui( &lr1110, dev_eui );
    modem_response_code = lr1110_modem_set_join_eui( &lr1110, join_eui );
    /* do a derive keys to get the pin code */
    lr1110_modem_derive_keys( &lr1110 );
    lr1110_modem_get_pin( &lr1110, &tracker_ctx.lorawan_pin );
    lr1110_modem_get_chip_eui( &lr1110, tracker_ctx.chip_eui );

    /* Init the software watchdog */
    hal_mcu_init_software_watchdog( tracker_ctx.app_scan_interval * 3 );

    /* Init Leds timer */
    timer_init( &led_tx_timer, on_led_tx_timer_event );
    timer_set_value( &led_tx_timer, 25 );
    timer_init( &led_rx_timer, on_led_rx_timer_event );
    timer_set_value( &led_rx_timer, 25 );

    /* Init tracker context volatile parameters */
    tracker_ctx.has_date                   = false;
    tracker_ctx.accelerometer_move_history = 1;
    tracker_ctx.stream_done                = true;
    tracker_ctx.voltage                    = hal_mcu_get_vref_level( );
    tracker_ctx.accelerometer_used         = USE_ACCELEROMETER;
    tracker_ctx.use_semtech_join_server    = USE_SEMTECH_JOIN_SERVER;
    tracker_ctx.system_sanity_check        = ( tracker_system_sanity_check_mask_t ) 0;
    tracker_ctx.payload_mask               = TRACKER_ALL_PAYLOAD;

    while( 1 )
    {
        /* Process Event */
        if( lr1110.event.callback != NULL )
        {
            lr1110_modem_event_process( &lr1110 );
        }

        switch( device_state )
        {
        case DEVICE_STATE_INIT:
        {
            HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM INIT ==== ######\r\n\r\n" );

            if( tracker_ctx.use_semtech_join_server == true )
            {
                modem_response_code = lr1110_modem_derive_keys( &lr1110 );
            }
            else
            {
                modem_response_code = lr1110_modem_set_app_key( &lr1110, app_key );
            }

            device_state = DEVICE_STATE_JOIN;

            break;
        }

        case DEVICE_STATE_JOIN:
        {
            /* Display used keys */
            print_lorawan_keys( dev_eui, join_eui, app_key, tracker_ctx.lorawan_pin );

            join_network( );

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_COLLECT_DATA:
        {
#if( ACCELEROMETER_MOUNTED == 1 )
            /* Create a movevment history on 8 bits and update this value only if the stream is done */
            if( tracker_ctx.stream_done == true )
            {
                tracker_ctx.accelerometer_move_history =
                    ( tracker_ctx.accelerometer_move_history << 1 ) + is_accelerometer_detected_moved( );
            }
#endif

            /* Check if scan can be launched */
            if( is_next_scan_possible( ) == true )
            {
                lr1110_modem_adr_profiles_t  adr_profile;
                lr1110_modem_lorawan_state_t lorawan_state;

                /* Reload the software watchdog */
                hal_mcu_reset_software_watchdog( );

                /* Adapt the ADR following the acceleromer movement */
                if( tracker_ctx.send_alive_frame == true )  // means device is static
                {
                    lr1110_modem_get_adr_profile( &lr1110, &adr_profile );
                    if( adr_profile != LORAWAN_STATIC_DATARATE )
                    {
                        HAL_DBG_TRACE_MSG( "Set ADR to LORAWAN_STATIC_DATARATE\n\r\n\r" );
                        lr1110_modem_set_adr_profile( &lr1110, LORAWAN_STATIC_DATARATE, adr_custom_list );
                    }
                }
                else  // means device is mobile
                {
                    lr1110_modem_get_adr_profile( &lr1110, &adr_profile );
                    if( adr_profile != LORAWAN_MOBILE_DATARATE )
                    {
                        HAL_DBG_TRACE_MSG( "Set ADR to LORAWAN_MOBILE_DATARATE\n\r\n\r" );
                        lr1110_modem_set_adr_profile( &lr1110, LORAWAN_MOBILE_DATARATE, adr_custom_list );
                    }
                }

                /* Led start for user notification */
                leds_on( LED_TX_MASK );
                timer_start( &led_tx_timer );

                /* Activate the partial low power mode */
                hal_mcu_partial_sleep_enable( true );

                /* Reset flag and counter */
                tracker_ctx.send_alive_frame = false;
                tracker_ctx.next_frame_ctn   = 0;

                /*  WIFI SCAN */
                if( tracker_ctx.wifi_settings.enabled == true )
                {
                    HAL_DBG_TRACE_INFO( "*** Wi-Fi Scan *** \n\r\n\r" );

                    tracker_run_wifi_scan( &tracker_ctx.wifi_settings, &tracker_ctx.wifi_result );

                    tracker_ctx.last_nb_detected_mac_address = tracker_ctx.wifi_result.nbr_results;
                }

                /*  GNSS SCAN */
                if( ( tracker_ctx.gnss_scan_if_wifi_not_good_enough == false ) ||
                    ( ( tracker_ctx.gnss_scan_if_wifi_not_good_enough == true ) &&
                      ( tracker_ctx.wifi_result.nbr_results < 6 ) ) )
                {
                    if( tracker_ctx.gnss_settings.enabled == true )
                    {
                        HAL_DBG_TRACE_INFO( "*** Gnss Scan ***\n\r\n\r" );

                        if( tracker_ctx.has_date == true )
                        {
                            struct tm epoch_time;

                            /* Timestamp scan */
                            tracker_ctx.timestamp = lr1110_modem_board_get_systime_from_gps( &lr1110 );

                            memcpy( &epoch_time, localtime( &tracker_ctx.timestamp ), sizeof( struct tm ) );
                            HAL_DBG_TRACE_PRINTF( "Date : %d-%d-%d %d:%d:%d.000 \r\n", epoch_time.tm_year + 1900,
                                                  epoch_time.tm_mon + 1, epoch_time.tm_mday, epoch_time.tm_hour,
                                                  epoch_time.tm_min, epoch_time.tm_sec );

                            /* Get the next nb sat value */
                            tracker_ctx.gnss_settings.nb_sat = tracker_get_gnss_next_nb_sat( );

                            tracker_run_gnss_scan( &tracker_ctx.gnss_settings, &tracker_ctx.gnss_scan_result );

                            /* Update the last_nb_detected_satellites field */
                            tracker_ctx.last_nb_detected_satellites =
                                tracker_ctx.gnss_scan_result.nb_detected_satellites;
                        }
                        else
                        {
                            HAL_DBG_TRACE_MSG( "Wait application layer clock synchronisation\r\n\r\n" );
                        }
                    }
                }
                else
                {
                    HAL_DBG_TRACE_MSG( "Wi-Fi Scan result good enough, don't perform GNSS scan\n\r" );
                }
                /*  SENSORS DATA */
#if( ACCELEROMETER_MOUNTED == 1 )
                HAL_DBG_TRACE_INFO( "*** sensors collect ***\n\r\n\r" );

                /* Acceleration */
                acc_read_raw_data( );
                tracker_ctx.accelerometer_x = acc_get_raw_x( );
                tracker_ctx.accelerometer_y = acc_get_raw_y( );
                tracker_ctx.accelerometer_z = acc_get_raw_z( );
                HAL_DBG_TRACE_PRINTF( "Acceleration [mg]: X=%4.2f mg | Y=%4.2f mg | Z=%4.2f mg \r\n",
                                      ( double ) tracker_ctx.accelerometer_x, ( double ) tracker_ctx.accelerometer_y,
                                      ( double ) tracker_ctx.accelerometer_z );

                /* Move history */
                HAL_DBG_TRACE_PRINTF( "Move history : %d\r\n", tracker_ctx.accelerometer_move_history );

                /* Temperature */
                tracker_ctx.tout = hal_mcu_get_temperature( ) * 100;
                HAL_DBG_TRACE_PRINTF( "Temperature : %d *C\r\n", tracker_ctx.tout / 100 );
#endif
                /* Modem charge */
                lr1110_modem_get_charge( &lr1110, &tracker_ctx.charge );
                HAL_DBG_TRACE_PRINTF( "Charge value : %d mAh\r\n", tracker_ctx.charge );

                /* Board voltage charge */
                lr1110_modem_get_lorawan_state( &lr1110, &lorawan_state );
                if( lorawan_state == LR1110_MODEM_LORAWAN_IDLE )
                {
                    tracker_ctx.voltage = hal_mcu_get_vref_level( );
                    HAL_DBG_TRACE_PRINTF( "Board voltage : %d mV\r\n", tracker_ctx.voltage );
                }
                else
                {
                    HAL_DBG_TRACE_WARNING( "TX ongoing don't collect the board voltage\r\n" );
                }

                /* Build the payload and stream it */
                build_and_stream_payload( tracker_ctx.payload_mask );

                /* Send tracker setting if it has been asked by the application server */
                if( tracker_ctx.tracker_settings_payload_len > 0 )
                {
                    build_and_stream_tracker_settings( tracker_ctx.tracker_settings_payload,
                                                       tracker_ctx.tracker_settings_payload_len );
                    tracker_ctx.tracker_settings_payload_len = 0;
                }

                device_state = DEVICE_STATE_SEND;
            }
            else
            {
                if( tracker_ctx.stream_done == true )
                {
                    if( tracker_ctx.next_frame_ctn >=
                        ( tracker_ctx.app_keep_alive_frame_interval / tracker_ctx.app_scan_interval ) )
                    {
                        HAL_DBG_TRACE_INFO( "Send a keep alive frame\r\n" );
                        tracker_ctx.send_alive_frame = true;
                    }
                    else
                    {
                        HAL_DBG_TRACE_PRINTF(
                            "Device is static next keep alive frame in %d sec\r\n",
                            ( ( tracker_ctx.app_keep_alive_frame_interval / tracker_ctx.app_scan_interval ) -
                              tracker_ctx.next_frame_ctn ) *
                                ( tracker_ctx.app_scan_interval / 1000 ) );
                        tracker_ctx.next_frame_ctn++;
                    }
                    device_state = DEVICE_STATE_CYCLE;
                }
                else
                {
                    device_state = DEVICE_STATE_SEND;
                }
            }

            break;
        }
        case DEVICE_STATE_SEND:
        {
            if( tracker_ctx.stream_done == false )
            {
                lr1110_modem_stream_status_t stream_status;

                /* Stream previous payload if it's not terminated */
                lr1110_modem_stream_status( &lr1110, LORAWAN_STREAM_APP_PORT, &stream_status );
                HAL_DBG_TRACE_PRINTF( "Streaming ongoing %d bytes remaining %d bytes free \r\n", stream_status.pending,
                                      stream_status.free );
            }

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            /* Reload the software watchdog */
            hal_mcu_reset_software_watchdog( );

            device_state = DEVICE_STATE_SLEEP;

            /* Schedule next packet transmission */
            do
            {
                modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, tracker_ctx.app_scan_interval / 1000 );
            } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );

            HAL_DBG_TRACE_PRINTF( "lr1110_modem_set_alarm_timer : %d s\r\n\r\n", tracker_ctx.app_scan_interval / 1000 );

            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            /* go in low power */
            if( lr1110_modem_board_read_event_line( &lr1110 ) == false )
            {
                hal_mcu_low_power_handler( );
            }
#if( ACCELEROMETER_MOUNTED == 1 )
            /* Wake up from static mode thanks the accelerometer ? */
            if( ( get_accelerometer_irq1_state( ) == true ) && ( is_tracker_in_static_mode( ) == true ) )
            {
                /* Stop the LR1110 current modem alarm */
                do
                {
                    modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, 0 );
                } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );

                device_state = DEVICE_COLLECT_DATA;
            }
#endif
            break;
        }
        default:
        {
            device_state = DEVICE_STATE_INIT;

            break;
        }
        }
    }
}

void _Error_Handler( int line )
{
    /* User can add his own implementation to report the HAL error return state */
    HAL_DBG_TRACE_ERROR( "Error Handler : %s\n", __FUNCTION__ );

    // reset the board
    hal_mcu_reset( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void tracker_run_wifi_scan( const wifi_settings_t* wifi_settings, wifi_scan_selected_result_t* wifi_result )
{
    static wifi_scan_all_results_t capture_result;

    if( wifi_execute_scan( &lr1110, wifi_settings, &capture_result ) == WIFI_SCAN_SUCCESS )
    {
        lr1110_modem_display_wifi_scan_results( &capture_result );

        lr1110_modem_wifi_scan_select_results( &capture_result, wifi_result );

        if( wifi_result->nbr_results != 0 )
        {
            /* Update the system_sanity_check bit field */
            tracker_ctx.system_sanity_check |= TRACKER_WIFI_SCAN_SUCCESSFUL_ONCE;
        }
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "Wi-Fi Scan error\n\r" );
        wifi_result->nbr_results = 0;  // reset MAC addr detected
    }
}

static uint8_t tracker_get_gnss_next_nb_sat( void )
{
    uint8_t next_tx_max_payload = 0;

    /* Adapt the nb_sat value in function of the number of byte available for the next lorawan payload */
    lr1110_modem_get_next_tx_max_payload( &lr1110, &next_tx_max_payload );

    if( next_tx_max_payload <= 51 )
    {
        return 8;
    }
    else
    {
        return 12;
    }
}

static void tracker_run_gnss_scan( const gnss_settings_t* gnss_settings, gnss_scan_single_result_t* capture_result )
{
    uint8_t gnss_status;

    gnss_status = gnss_scan_execute( &lr1110, gnss_settings, capture_result );

    if( gnss_status == GNSS_SCAN_SUCCESS )
    {
        gnss_scan_display_results( capture_result );

        if( capture_result->nb_detected_satellites != 0 )
        {
            /* Update the system_sanity_check bit field */
            tracker_ctx.system_sanity_check |= TRACKER_GNSS_SCAN_SUCCESSFUL_ONCE;
        }
    }
    else
    {
        if( gnss_status == GNSS_SCAN_NO_TIME )
        {
            HAL_DBG_TRACE_MSG( "GNSS Scan error: No time\n\r" );
        }
        else
        {
            HAL_DBG_TRACE_MSG( "GNSS Scan error\n\r" );
        }
    }
}

static void build_and_stream_payload( tracker_payload_mask_t payload_mask )
{
    /* BUILD THE PAYLOAD IN TLV FORMAT */
    tracker_ctx.lorawan_payload_len = 0;  // reset the payload len

    HAL_DBG_TRACE_MSG( "\r\nIs added in the stream FiFo:\r\n" );

    if( payload_mask & TRACKER_GNSS_PAYLOAD )
    {
        if( ( tracker_ctx.gnss_scan_result.is_valid_nav_message == true ) &&
            ( tracker_ctx.gnss_settings.enabled == true ) )
        {
            HAL_DBG_TRACE_MSG( " - NAV message :" );

            tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TAG_NAV;  // GNSS PATCH TAG
            tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] =
                tracker_ctx.gnss_scan_result.nav_message_size - 1;  // GNSS LEN
            memcpy( tracker_ctx.lorawan_payload + tracker_ctx.lorawan_payload_len,
                    tracker_ctx.gnss_scan_result.nav_message + 1, tracker_ctx.gnss_scan_result.nav_message_size - 1 );
            tracker_ctx.lorawan_payload_len += tracker_ctx.gnss_scan_result.nav_message_size - 1;

            /* Push the NAV message in the FiFo stream */
            add_payload_in_streaming_fifo( tracker_ctx.lorawan_payload, tracker_ctx.lorawan_payload_len );

            tracker_ctx.lorawan_payload_len = 0;  // reset the payload len
        }
    }

    if( payload_mask & TRACKER_WIFI_PAYLOAD )
    {
        if( ( tracker_ctx.wifi_result.nbr_results >= 2 ) && ( tracker_ctx.wifi_settings.enabled == true ) )
        {
            /* Add Wi-Fi scan */
            uint8_t wifi_index = 0;
            HAL_DBG_TRACE_MSG( " - WiFi scan : " );
            tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TAG_WIFI_SCAN;  // Wi-Fi TAG
            tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] =
                tracker_ctx.wifi_result.nbr_results * WIFI_SINGLE_BEACON_LEN;  // Wi-Fi Len

            wifi_index = tracker_ctx.lorawan_payload_len;
            for( uint8_t i = 0; i < tracker_ctx.wifi_result.nbr_results; i++ )
            {
                tracker_ctx.lorawan_payload[wifi_index] = tracker_ctx.wifi_result.results[i].rssi;
                memcpy( &tracker_ctx.lorawan_payload[wifi_index + 1], tracker_ctx.wifi_result.results[i].mac_address,
                        6 );
                wifi_index += WIFI_SINGLE_BEACON_LEN;
            }
            tracker_ctx.lorawan_payload_len += tracker_ctx.wifi_result.nbr_results * WIFI_SINGLE_BEACON_LEN;

            /* Push the Wi-Fi data in the FiFo stream */
            add_payload_in_streaming_fifo( tracker_ctx.lorawan_payload, tracker_ctx.lorawan_payload_len );

            tracker_ctx.lorawan_payload_len = 0;  // reset the payload len
        }
    }

    if( payload_mask & TRACKER_SENSORS_PAYLOAD )
    {
#if( ACCELEROMETER_MOUNTED == 1 )
        /* Send sensors value */
        HAL_DBG_TRACE_MSG( " - sensors value : " );
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TAG_ACCELEROMETER;  // Accelerometer TAG
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = 9;                  // Accelerometer LEN

        /* Accelemrometer movement history  */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accelerometer_move_history;

        /* Acceleromter data */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accelerometer_x >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accelerometer_x;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accelerometer_y >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accelerometer_y;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accelerometer_z >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accelerometer_z;

        /* Temperature */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.tout >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.tout;
#endif
        /* Charge */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TAG_CHARGE;  // Charge TAG
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = 4;           // Charge LEN
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.charge >> 24;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.charge >> 16;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.charge >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.charge;

        /* Board voltage */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TAG_VOLTAGE;  // Voltage TAG
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = 2;            // Voltage LEN
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.voltage >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.voltage;

        /* Push the sensor values in the FiFo stream */
        add_payload_in_streaming_fifo( tracker_ctx.lorawan_payload, tracker_ctx.lorawan_payload_len );

        tracker_ctx.lorawan_payload_len = 0;  // reset the payload len
    }
}

static void build_and_stream_tracker_settings( uint8_t* buffer, uint8_t len )
{
    /* Send sensors value */
    HAL_DBG_TRACE_MSG( " - Tracker settings : " );
    tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TAG_TRACKER_SETTINGS;  // Tracker settings TAG
    tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = len;  // Tracker settings LEN is variable

    memcpy( tracker_ctx.lorawan_payload + 2, buffer, len );
    tracker_ctx.lorawan_payload_len += len;

    /* Push the sensor values in the FiFo stream */
    add_payload_in_streaming_fifo( tracker_ctx.lorawan_payload, tracker_ctx.lorawan_payload_len );

    tracker_ctx.lorawan_payload_len = 0;  // reset the payload len
}

static lr1110_modem_response_code_t gnss_init( void )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    modem_response_code =
        lr1110_modem_gnss_set_assistance_position( &lr1110, &tracker_ctx.gnss_settings.assistance_position );

    return modem_response_code;
}

static void print_hex_buffer( const uint8_t* buffer, uint8_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
            HAL_DBG_TRACE_PRINTF( "\r\n" );
            newline = 0;
        }

        HAL_DBG_TRACE_PRINTF( "%02X ", buffer[i] );

        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
}

static void print_lorawan_keys( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key, uint32_t pin )
{
    HAL_DBG_TRACE_PRINTF( "DevEui      : %02X", dev_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", dev_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
    HAL_DBG_TRACE_PRINTF( "AppEui      : %02X", join_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", join_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
    if( tracker_ctx.use_semtech_join_server )
    {
        HAL_DBG_TRACE_MSG( "AppKey      : Semtech join server used\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "AppKey      : %02X", app_key[0] );
        for( int i = 1; i < 16; i++ )
        {
            HAL_DBG_TRACE_PRINTF( "-%02X", app_key[i] );
        }
        HAL_DBG_TRACE_PRINTF( "\r\n" );
    }

    HAL_DBG_TRACE_PRINTF( "Pin         : %08X\r\n\r\n", pin );
}

static void join_network( void )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    /* Starts the join procedure */
    modem_response_code = lr1110_modem_join( &lr1110 );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_INFO( "###### ===== JOINING ==== ######\r\n\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "###### ===== JOINING CMD ERROR ==== ######\r\n\r\n" );
    }
}

static bool add_payload_in_streaming_fifo( uint8_t* payload, uint16_t len )
{
    lr1110_modem_stream_status_t stream_status;

    /* Push the Payload in the FiFo stream */
    lr1110_modem_stream_status( &lr1110, LORAWAN_STREAM_APP_PORT, &stream_status );
    if( stream_status.free > len )
    {
        tracker_ctx.stream_done = false;
        HAL_DBG_TRACE_MSG( "add in streaming FiFo\r\n" );
        lr1110_modem_send_stream_data( &lr1110, LORAWAN_STREAM_APP_PORT, payload, len );

        return true;
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "Not enought space, need = %d bytes - free = %d bytes\r\n", len, stream_status.free );

        return false;
    }
}

static bool is_next_scan_possible( void )
{
    if( ( ( tracker_ctx.accelerometer_move_history != 0 ) || ( tracker_ctx.send_alive_frame == true ) ||
          ( tracker_ctx.accelerometer_used == 0 ) ) &&
        ( tracker_ctx.stream_done == true ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool is_tracker_in_static_mode( void )
{
    if( ( tracker_ctx.accelerometer_move_history == 0 ) && ( tracker_ctx.accelerometer_used == 1 ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void on_led_tx_timer_event( void* context )
{
    timer_stop( &led_tx_timer );
    /* Switch LED TX OFF */
    leds_off( LED_TX_MASK );
}

static void on_led_rx_timer_event( void* context )
{
    timer_stop( &led_rx_timer );
    /* Switch LED RX OFF */
    leds_off( LED_RX_MASK );
}

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM RESET %lu ==== ######\r\n\r\n", reset_count );

    if( lr1110_modem_board_is_ready( ) == true )
    {
        /* System reset */
        hal_mcu_reset( );
    }
    else
    {
        lr1110_modem_board_set_ready( true );
    }
}

static void lr1110_modem_network_joined( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== JOINED ==== ######\r\n\r\n" );

    /* Set the ADR profile once joined */
    lr1110_modem_set_adr_profile( &lr1110, LORAWAN_DEFAULT_DATARATE, adr_custom_list );

    /* Init the stream once joined */
    lr1110_modem_stream_init( &lr1110, 0, LR1110_MODEM_SERVICES_ENCRYPTION_DISABLE );
}

static void lr1110_modem_join_fail( void ) { HAL_DBG_TRACE_INFO( "###### ===== JOIN FAIL ==== ######\r\n\r\n" ); }

static void modem_status_to_string( lr1110_modem_status_t modem_status )
{
    HAL_DBG_TRACE_MSG( "Modem status : " );

    if( ( modem_status & LR1110_LORAWAN_CRASH ) == LR1110_LORAWAN_CRASH )
    {
        HAL_DBG_TRACE_MSG( "CRASH " );
    }
    if( ( modem_status & LR1110_LORAWAN_MUTE ) == LR1110_LORAWAN_MUTE )
    {
        HAL_DBG_TRACE_MSG( "MUTE " );
    }
    if( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED )
    {
        HAL_DBG_TRACE_MSG( "JOINED " );
    }
    if( ( modem_status & LR1110_LORAWAN_SUSPEND ) == LR1110_LORAWAN_SUSPEND )
    {
        HAL_DBG_TRACE_MSG( "SUSPEND " );
    }
    if( ( modem_status & LR1110_LORAWAN_UPLOAD ) == LR1110_LORAWAN_UPLOAD )
    {
        HAL_DBG_TRACE_MSG( "UPLOAD " );
    }
    if( ( modem_status & LR1110_LORAWAN_JOINING ) == LR1110_LORAWAN_JOINING )
    {
        HAL_DBG_TRACE_MSG( "JOINING " );
    }
    if( ( modem_status & LR1110_LORAWAN_STREAM ) == LR1110_LORAWAN_STREAM )
    {
        HAL_DBG_TRACE_MSG( "STREAM " );
    }

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
}

void lr1110_modem_alarm( void )
{
    lr1110_modem_status_t        modem_status;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    HAL_DBG_TRACE_INFO( "###### ===== LR1110 ALARM ==== ######\r\n\r\n" );

    modem_response_code = lr1110_modem_get_status( &lr1110, &modem_status );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        modem_status_to_string( modem_status );

        if( ( modem_status & LR1110_LORAWAN_CRASH ) == LR1110_LORAWAN_CRASH )
        {
            device_state = DEVICE_STATE_CYCLE;
        }
        else if( ( ( modem_status & LR1110_LORAWAN_MUTE ) == LR1110_LORAWAN_MUTE ) ||
                 ( ( modem_status & LR1110_LORAWAN_SUSPEND ) == LR1110_LORAWAN_SUSPEND ) )
        {
            device_state = DEVICE_STATE_CYCLE;
        }
        else if( ( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED ) ||
                 ( ( modem_status & LR1110_LORAWAN_STREAM ) == LR1110_LORAWAN_STREAM ) ||
                 ( ( modem_status & LR1110_LORAWAN_UPLOAD ) == LR1110_LORAWAN_UPLOAD ) )
        {
            device_state = DEVICE_COLLECT_DATA;
        }
        else if( ( modem_status & LR1110_LORAWAN_JOINING ) == LR1110_LORAWAN_JOINING )
        {
            /* Network not joined yet. Wait */
            device_state = DEVICE_STATE_CYCLE;
        }
        else
        {
            HAL_DBG_TRACE_WARNING( "Unknow modem status %d\r\n\r\n", modem_status );
            device_state = DEVICE_STATE_CYCLE;
        }
    }
}

static void lr1110_modem_down_data( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port,
                                    const uint8_t* payload, uint8_t size )
{
    HAL_DBG_TRACE_INFO( "\r\n###### ===== DOWNLINK FRAME %lu ==== ######\r\n\r\n", downlink_cnt++ );

    HAL_DBG_TRACE_PRINTF( "RX WINDOW   : %d\r\n", flags );

    HAL_DBG_TRACE_PRINTF( "RX PORT     : %d\r\n", port );

    if( size != 0 )
    {
        HAL_DBG_TRACE_MSG( "RX DATA     : " );
        print_hex_buffer( payload, size );
    }

    HAL_DBG_TRACE_PRINTF( "RX RSSI     : %d\r\n", rssi );
    HAL_DBG_TRACE_PRINTF( "RX SNR      : %d\r\n\r\n", snr );

    /* Update the system_sanity_check bit field */
    tracker_ctx.system_sanity_check |= TRACKER_DOWNLINK_SUCCESSFUL_ONCE;

    leds_on( LED_RX_MASK );
    timer_start( &led_rx_timer );

    parse_frame( port, payload, size );
}

static void lr1110_modem_mute( lr1110_modem_mute_t mute )
{
    if( mute == LR1110_MODEM_UNMUTED )
    {
        HAL_DBG_TRACE_INFO( "###### ===== MODEM UNMUTED ==== ######\r\n\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_INFO( "###### ===== MODEM MUTED ==== ######\r\n\r\n" );
    }
}

static void lr1110_modem_set_conf( uint8_t infor_tag )
{
    HAL_DBG_TRACE_INFO( "###### ===== MODEM SET CONF %02X ==== ######\r\n\r\n", infor_tag );
}

static void lr1110_modem_stream_done( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== STREAM DONE nb %d ==== ######\r\n\r\n", stream_cnt++ );

    tracker_ctx.stream_done = true;
}

static void lr1110_modem_time_updated_alc_sync( lr1110_modem_alc_sync_state_t alc_sync_state )
{
    HAL_DBG_TRACE_INFO( "###### ===== APPLICATION LAYER CLOCK SYNC EVENT ==== ######\r\n\r\n" );

    /* Update the system_sanity_check bit field */
    tracker_ctx.system_sanity_check |= TRACKER_DOWNLINK_SUCCESSFUL_ONCE;

    if( alc_sync_state == LR1110_MODEM_ALC_SYNC_SYNCHRONIZED )
    {
        HAL_DBG_TRACE_MSG( "CLOCK SYNC STATE SYNCHRONIZED\r\n\r\n" );
        /* Notify user that the date has been received */
        leds_blink( LED_RX_MASK, 100, 4, true );
        tracker_ctx.has_date = true;
    }
    else
    {
        HAL_DBG_TRACE_MSG( "CLOCK SYNC STATE DESYNCHRONIZED\r\n\r\n" );
        /* Notify user that the date has been received */
        leds_blink( LED_TX_MASK, 100, 4, true );
        tracker_ctx.has_date = false;
    }
}

static void lr1110_modem_adr_mobile_to_static( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== ADR HAS SWITCHED FROM MOBILE TO STATIC ==== ######\r\n\r\n" );
}

static void lr1110_modem_new_link_adr( void ) { HAL_DBG_TRACE_INFO( "###### ===== NEW LINK ADR ==== ######\r\n\r\n" ); }

static void lr1110_modem_no_event( void ) { HAL_DBG_TRACE_INFO( "###### ===== NO EVENT ==== ######\r\n\r\n" ); }

static void parse_frame( uint8_t port, const uint8_t* payload, uint8_t size )
{
    switch( port )
    {
    case GNSS_PUSH_SOLVER_MSG_PORT:
    {
        HAL_DBG_TRACE_INFO( "###### ===== GNSS PUSH SOLVER MSG ==== ######\r\n\r\n" );

        lr1110_modem_gnss_push_solver_msg( &lr1110, payload, size );

        break;
    }
    case TRACKER_REQUEST_MSG_PORT:
    {
        uint8_t tag           = 0;
        uint8_t len           = 0;
        uint8_t payload_index = 0;

        while( payload_index < size )
        {
            tag = payload[payload_index++];
            len = payload[payload_index++];

            switch( tag )
            {
            case GET_APP_TRACKER_SETTINGS_CMD:
            {
                uint8_t settings_buffer[240];
                memcpy( settings_buffer, payload + payload_index, len );

                HAL_DBG_TRACE_INFO(
                    "###### ===== TRACKER CONFIGURATION SETTINGS PAYLOAD RECEIVED ==== ######\r\n\r\n" );

                tracker_ctx.tracker_settings_payload_len =
                    tracker_parse_cmd( settings_buffer, tracker_ctx.tracker_settings_payload );

                break;
            }
            case GET_MODEM_DATE_CMD:
            {
                uint8_t  buffer[6];
                uint32_t modem_date;

                modem_date = lr1110_modem_board_get_systime_from_gps( &lr1110 );

                buffer[0] = GET_MODEM_DATE_CMD;
                buffer[1] = GET_MODEM_DATE_ANSWER_LEN;
                buffer[2] = modem_date >> 24;
                buffer[3] = modem_date >> 16;
                buffer[4] = modem_date >> 8;
                buffer[5] = modem_date;

                /* Use the emergency TX to reduce the latency */
                lr1110_modem_emergency_tx( &lr1110, port, LR1110_MODEM_UPLINK_UNCONFIRMED, buffer, 6 );

                HAL_DBG_TRACE_INFO( "###### ===== SEND MODEM-E DATE IN EMERGENCY TX ==== ######\r\n\r\n" );

                break;
            }
            case SET_RX_LED_CMD:
            {
                /* Wait the end of the rx led timer */
                HAL_Delay( 30 );

                if( payload[payload_index] == 1 )
                {
                    leds_on( LED_RX_MASK );
                }
                if( payload[payload_index] == 2 )
                {
                    leds_off( LED_RX_MASK );
                }

                payload_index += len;
                break;
            }
            default:
                payload_index += len;
                break;
            }
        }
    }
    default:
        break;
    }
}

/* --- EOF ------------------------------------------------------------------ */
