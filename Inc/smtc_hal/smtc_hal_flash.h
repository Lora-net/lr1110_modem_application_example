/*!
 * @file      smtc_hal_flash.h
 *
 * @brief     Board specific package FLASH API definition.
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

#ifndef SMTC_HAL_FLASH_H
#define SMTC_HAL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define ADDR_FLASH_PAGE_SIZE ( ( uint32_t ) 0x00000800 ) /* Size of Page = 2 KBytes */

#define FLASH_BYTE_EMPTY_CONTENT ( ( uint8_t ) 0xFF )
#define FLASH_PAGE_EMPTY_CONTENT ( ( uint64_t ) 0xFFFFFFFFFFFFFFFF )

#define FLASH_USER_START_PAGE ( 1 ) /* Start nb page of user Flash area */

#define FLASH_USER_END_ADDR \
    ( ADDR_FLASH_PAGE( FLASH_USER_END_PAGE ) + ADDR_FLASH_PAGE_SIZE - 1 ) /* End @ of user Flash area */
#define FLASH_USER_END_PAGE ( 509 )                                       /* End nb page of user Flash area */

#define FLASH_USER_INTERNAL_LOG_CTX_START_PAGE ( FLASH_USER_END_PAGE + 1 )

#define FLASH_USER_INTERNAL_LOG_CTX_START_ADDR ADDR_FLASH_PAGE( FLASH_USER_INTERNAL_LOG_CTX_START_PAGE )
#define FLASH_USER_INTERNAL_LOG_CTX_END_ADDR                                             \
    ( ADDR_FLASH_PAGE( FLASH_USER_INTERNAL_LOG_CTX_START_PAGE ) + ADDR_FLASH_PAGE_SIZE - \
      1 ) /* End @ of user ctx Flash area */

#define FLASH_USER_TRACKER_CTX_START_PAGE ( FLASH_USER_END_PAGE + 2 )

#define FLASH_USER_TRACKER_CTX_START_ADDR ADDR_FLASH_PAGE( FLASH_USER_TRACKER_CTX_START_PAGE )
#define FLASH_USER_TRACKER_CTX_END_ADDR                                             \
    ( ADDR_FLASH_PAGE( FLASH_USER_TRACKER_CTX_START_PAGE ) + ADDR_FLASH_PAGE_SIZE - \
      1 ) /* End @ of user tracker ctx Flash area */

/* Base address of the Flash s */
#define ADDR_FLASH_PAGE_0 ( ( uint32_t ) 0x08000000 ) /* Base @ of Page 0, 2 KBytes */
#define ADDR_FLASH_PAGE( page ) ( ADDR_FLASH_PAGE_0 + ( page ) *ADDR_FLASH_PAGE_SIZE )

#define FLASH_OPERATION_MAX_RETRY 4

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initializes the FLASH module and find the first empty page.
 *
 * @returns User flash start address
 */
uint32_t flash_init( void );

/*!
 * @brief Erase a given nb page to the FLASH at the specified address.
 *
 * @param [in] addr FLASH address to start the erase
 * @param [in] nb_page the number of page to erase.
 * @returns status [SMTC_SUCCESS, SMTC_FAIL]
 */
uint8_t flash_erase_page( uint32_t addr, uint8_t nb_page );

/*!
 * @brief Force erasing of a given nb page to the FLASH at the specified address.
 *
 * @param [in] addr FLASH address to start the erase
 * @param [in] nb_page the number of page to erase.
 * @returns status [SMTC_SUCCESS, SMTC_FAIL]
 */
uint8_t flash_force_erase_page( uint32_t addr, uint8_t nb_page );

/*!
 * @brief Writes the given buffer to the FLASH at the specified address.
 *
 * @param [in] addr FLASH address to write to
 * @param [in] buffer Pointer to the buffer to be written.
 * @param [in] size Size of the buffer to be written.
 * @returns status [Real_size_written, FAIL]
 */
uint32_t flash_write_buffer( uint32_t addr, uint8_t* buffer, uint32_t size );

/*!
 * @brief Reads the FLASH at the specified address to the given buffer.
 *
 * @param [in] addr FLASH address to read from
 * @param [out] buffer Pointer to the buffer to be written with read data.
 * @param [in] size Size of the buffer to be read.
 * @returns status [SMTC_SUCCESS, SMTC_FAIL]
 */
void flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size );

/*!
 * @brief Reads the FLASH at the specified address to the given buffer.
 *
 * @returns User flash start address.
 */
uint32_t flash_get_user_start_addr( void );

/*!
 * @brief Set the FLASH user start addr.
 *
 * @param [in] addr User flash start address.
 */
void flash_set_user_start_addr( uint32_t addr );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_FLASH_H

/* --- EOF ------------------------------------------------------------------ */
