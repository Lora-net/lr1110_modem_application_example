/*!
 * @file      lorawan_config.c
 *
 * @brief     LoRaWAN configuration implementation
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
#include "lorawan_config.h"
#include "lr1110_modem_board.h"
#include "utilities.h"
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

lr1110_modem_response_code_t lorawan_init( lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class )
{
    lr1110_modem_dm_info_fields_t dm_info_fields;
    lr1110_modem_response_code_t  modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    modem_response_code |= lr1110_modem_set_class( &lr1110, lorawan_class );

    if( lorawan_class == LR1110_LORAWAN_CLASS_A )
    {
        HAL_DBG_TRACE_MSG( "CLASS       : A\r\n" );
    }
    if( lorawan_class == LR1110_LORAWAN_CLASS_C )
    {
        HAL_DBG_TRACE_MSG( "CLASS       : C\r\n" );
    }

    modem_response_code |= lr1110_modem_set_region( &lr1110, region );
    if( region == LR1110_LORAWAN_REGION_EU868 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : EU868\r\n\r\n" );
        modem_response_code |= lr1110_modem_activate_duty_cycle( &lr1110, LORAWAN_DUTYCYCLE_ON );
    }
    if( region == LR1110_LORAWAN_REGION_US915 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : US915\r\n\r\n" );
    }
    if( region == LR1110_LORAWAN_REGION_AS923_GRP1 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP1\r\n\r\n" );
    }
    if( region == LR1110_LORAWAN_REGION_CN470 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : CN470\r\n\r\n" );
    }
    if( region == LR1110_LORAWAN_REGION_AS923_GRP2 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP2\r\n\r\n" );
    }
    if( region == LR1110_LORAWAN_REGION_AS923_GRP3 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP3\r\n\r\n" );
    }
    if( region == LR1110_LORAWAN_REGION_IN865 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : IN865\r\n\r\n" );
    }
    if( region == LR1110_LORAWAN_REGION_KR920 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : KR920\r\n\r\n" );
    }
    if( region == LR1110_LORAWAN_REGION_RU864 )
    {
        HAL_DBG_TRACE_MSG( "REGION      : RU864\r\n\r\n" );
    }

    /* Set DM info field */
    dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_CHARGE;
    dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_GNSS_ALMANAC_STATUS;
    dm_info_fields.dm_info_field[2] = LR1110_MODEM_DM_INFO_TYPE_TEMPERATURE;
    dm_info_fields.dm_info_length   = 3;

    modem_response_code |= lr1110_modem_set_dm_info_field( &lr1110, &dm_info_fields );

    modem_response_code |= lr1110_modem_set_dm_info_interval( &lr1110, LR1110_MODEM_REPORTING_INTERVAL_IN_DAY, 1 );

    modem_response_code |= lr1110_modem_set_alc_sync_mode( &lr1110, LR1110_MODEM_ALC_SYNC_MODE_ENABLE );

    return modem_response_code;
}

/* --- EOF ------------------------------------------------------------------ */
