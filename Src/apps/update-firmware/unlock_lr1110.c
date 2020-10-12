/*!
 * \file      unlock_lr1110.c
 *
 * \brief     lr1110 unlock implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* Change this by the right firmware to unlock the lr1110 */
#include "lr1110_trx_0303_prod.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief Unlock the chip
 */
void lr1110_unlock_chip( const uint32_t* data, const uint16_t length );

/*!
 * \brief Reset event callback
 *
 * \param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * \brief Main application entry point.
 */
int main( void )
{
    // Target board initialization
    hal_mcu_init( );

    hal_mcu_init_periph( );

    lr1110_unlock_chip(flash,FLASH_COUNT);
    
    while( 1 )
        ;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void _Error_Handler( int line )
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while( 1 )
    {
        HAL_DBG_TRACE_ERROR( "%s\n", __FUNCTION__ );
    }
    /* USER CODE END Error_Handler_Debug */
}

/* Switch LR1110 prod in dev mode - need special flash witch chip eui */
void lr1110_unlock_chip( const uint32_t* data, const uint16_t length )
{
    lr1110_bootloader_version_t version;

    // Switch en DFU Mode
    lr1110_modem_hal_enter_dfu( &lr1110 );

    lr1110_bootloader_get_version( &lr1110, &version );

    HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", version.hw, version.type, version.fw );

    if( ( version.hw >= 0x21 ) && ( version.type == 0xDF ) && ( version.fw == 0x6500 ) )
    {
        HAL_DBG_TRACE_MSG( "UNLOCK THE CHIP\n\r" );

        lr1110_bootloader_erase_flash( &lr1110 );
        lr1110_bootloader_write_flash_full( &lr1110, 0, data, length );
        lr1110_modem_hal_reset( &lr1110 );
        HAL_Delay( 200 );
        lr1110_modem_hal_reset( &lr1110 );

        lr1110_bootloader_get_version( &lr1110, &version );
        HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", version.hw, version.type, version.fw );
        HAL_Delay( 200 );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "CAN'T UNLOCK THE CHIP\n\r" );
    }
}

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM RESET %lu ==== ######\r\n\r\n", reset_count );

    if( lr1110_modem_board_is_ready( ) == true )
    {
        // System reset
        hal_mcu_reset( );
    }
    else
    {
        lr1110_modem_board_set_ready( true );
    }
}

/* --- EOF ------------------------------------------------------------------ */
