/*!
 * \file      smtc_hal_mcu.c
 *
 * \brief     Board specific package MCU API implementation.
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_utils.h"
#include "lr1110-modem-board.h"
#include "smtc_hal.h"

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#endif

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
lr1110_t lr1110;

/*!
 * \brief Low Power options
 */
typedef enum low_power_mode_e
{
    LOW_POWER_ENABLE,
    LOW_POWER_DISABLE,
    LOW_POWER_DISABLE_ONCE
} low_power_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile bool             hal_exit_wait        = false;
static volatile low_power_mode_t hal_lp_current_mode  = LOW_POWER_ENABLE;
static bool                      partial_sleep_enable = false;

/*!
 * \brief Timer to handle the software watchdog
 */
static timer_event_t soft_watchdog;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief init the MCU clock tree
 */
static void hal_mcu_system_clock_config( void );

/*!
 * \brief reinit the MCU clock tree after a stop mode
 */
static void hal_mcu_system_clock_re_config_after_stop( void );

/*!
 * \brief init the GPIO
 */
static void hal_mcu_gpio_init( void );

/*!
 * \brief deinit the GPIO
 */
static void hal_mcu_gpio_deinit( void );

/*!
 * \brief init the power voltage detector
 */
static void hal_mcu_pvd_config( void );

/*!
 * \brief Deinit the MCU
 */
static void hal_mcu_deinit( void );

/*!
 * \brief Initializes MCU after a stop mode
 */
static void hal_mcu_reinit( void );

/*!
 * \brief reinit the peripherals
 */
static void hal_mcu_reinit_periph( void );

/*!
 * \brief deinit the peripherals 
 */
static void hal_mcu_deinit_periph( void );

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
/*!
 * \brief printf
 */
static void vprint( const char* fmt, va_list argp );
#endif

/*!
 * \brief Function executed on software watchdog event
 */
static void on_soft_watchdog_event( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void hal_mcu_critical_section_end( uint32_t* mask ) { __set_PRIMASK( *mask ); }

void hal_mcu_init_periph( void )
{
    // Init TX & RX Leds
    leds_init();
    
    // External supplies
    external_supply_init( LNA_SUPPLY_MASK );

    // LIS2DE12 accelerometer
#if(ACCELEROMETER_MOUNTED == 1)
    accelerometer_init( INT_1 );
#endif
}

static void hal_mcu_reinit_periph( void )
{   
    // Leds
    leds_init( );
    
    // External supplies
    external_supply_init( LNA_SUPPLY_MASK );
}

void hal_mcu_deinit_periph( void )
{
    // Leds
    leds_deinit();
    
    // Disable external supply
    external_supply_deinit( LNA_SUPPLY_MASK );

    hal_mcu_gpio_deinit();
}

void hal_mcu_init( void )
{
    // Initialize MCU HAL library
    HAL_Init( );

    // Initialize clocks
    hal_mcu_system_clock_config( );

    // Initialize GPIOs
    hal_mcu_gpio_init( );

    // Initialize low power timer
    hal_tmr_init( );
    
    // Init power voltage voltage detector
    hal_mcu_pvd_config( );

    // Initialize UART
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_init( HAL_PRINTF_UART_ID, UART_TX, UART_RX);
#endif

    // Initialize SPI
    hal_spi_init( HAL_RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );
    lr1110_modem_board_init_io_context( &lr1110 );
    // Init LR1110 IO
    lr1110_modem_board_init_io( &lr1110 );

    // Initialize RTC
    hal_rtc_init( );

    // Initialize I2C
#if(ACCELEROMETER_MOUNTED == 1)
    hal_i2c_init( HAL_I2C_ID, I2C_SDA, I2C_SCL );
#endif
}

void hal_mcu_disable_irq( void ) { __disable_irq( ); }

void hal_mcu_enable_irq( void ) { __enable_irq( ); }

void hal_mcu_reset( void )
{
    __disable_irq( );

    // Restart system
    NVIC_SystemReset( );
}

void hal_mcu_panic( void )
{
    CRITICAL_SECTION_BEGIN( );

    HAL_DBG_TRACE_ERROR( "%s\n", __FUNCTION__ );
    HAL_DBG_TRACE_ERROR( "PANIC" );

    while( 1 )
    {
    }
}

void hal_mcu_wait_us( const int32_t microseconds )
{
    const uint32_t nb_nop = microseconds * 1000 / 561;
    for( uint32_t i = 0; i < nb_nop; i++ )
    {
        __NOP( );
    }
}

void hal_mcu_init_software_watchdog( uint32_t value )
{
    timer_init( &soft_watchdog, on_soft_watchdog_event );
    timer_set_value( &soft_watchdog, value );
    timer_start( &soft_watchdog );
}

void hal_mcu_set_software_watchdog_value( uint32_t value )
{
    timer_set_value( &soft_watchdog, value );
}

void hal_mcu_start_software_watchdog( void )
{
    timer_start( &soft_watchdog );
}

void hal_mcu_reset_software_watchdog( void )
{
    timer_reset( &soft_watchdog );
}

uint16_t hal_mcu_get_vref_level( void )
{
    return 0;
}

void hal_mcu_disable_low_power_wait( void )
{
    hal_exit_wait       = true;
    hal_lp_current_mode = LOW_POWER_DISABLE;
}

void hal_mcu_enable_low_power_wait( void )
{
    hal_exit_wait       = false;
    hal_lp_current_mode = LOW_POWER_ENABLE;
}

void hal_mcu_disable_once_low_power_wait( void )
{
    hal_exit_wait       = true;
    hal_lp_current_mode = LOW_POWER_DISABLE_ONCE;
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

void hal_mcu_trace_print( const char* fmt, ... )
{
#if HAL_DBG_TRACE == HAL_FEATURE_ON
    va_list argp;
    va_start( argp, fmt );
    vprint( fmt, argp );
    va_end( argp );
#endif
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line
 * number where the assert_param error has occurred. Input          : - file:
 * pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    // User can add his own implementation to report the file name and line
    // number,
    // ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line)

    HAL_DBG_TRACE_PRINTF( "Wrong parameters value: file %s on line %lu\r\n", ( const char* ) file, line );
    // Infinite loop
    while( 1 )
    {
    }
}
#endif

void hal_mcu_partial_sleep_enable( bool enable ) { partial_sleep_enable = enable; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void hal_mcu_system_clock_config( void )
{
    RCC_OscInitTypeDef       rcc_osc_init;
    RCC_ClkInitTypeDef       rcc_clk_init;
    RCC_PeriphCLKInitTypeDef periph_clk_init;

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );
    /* Ensure that MSI is wake-up system clock */ 
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

    // Initializes the CPU, AHB and APB busses clocks
    rcc_osc_init.OscillatorType      = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_HSI;
    rcc_osc_init.MSIState            = RCC_MSI_ON;
    rcc_osc_init.HSEState            = RCC_HSE_OFF;
    rcc_osc_init.HSIState            = RCC_HSI_OFF;
    rcc_osc_init.LSEState            = RCC_LSE_ON;
    rcc_osc_init.LSIState            = RCC_LSI_OFF;
    rcc_osc_init.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    rcc_osc_init.MSIClockRange       = RCC_MSIRANGE_11;
    rcc_osc_init.PLL.PLLState        = RCC_PLL_ON;
    rcc_osc_init.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    rcc_osc_init.PLL.PLLM            = 3;      
    rcc_osc_init.PLL.PLLN            = 10;
    rcc_osc_init.PLL.PLLP            = RCC_PLLP_DIV7;
    rcc_osc_init.PLL.PLLQ            = RCC_PLLQ_DIV2;
    rcc_osc_init.PLL.PLLR            = RCC_PLLR_DIV2;
    if( HAL_RCC_OscConfig( &rcc_osc_init ) != HAL_OK )
    {
    }

    // Initializes the CPU, AHB and APB busses clocks
    rcc_clk_init.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rcc_clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if( HAL_RCC_ClockConfig( &rcc_clk_init, FLASH_LATENCY_1 ) != HAL_OK )
    {
    }

    periph_clk_init.PeriphClockSelection =
        RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_LPTIM1 ;
    periph_clk_init.Lptim1ClockSelection  = RCC_LPTIM1CLKSOURCE_LSE;
    periph_clk_init.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
    periph_clk_init.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if( HAL_RCCEx_PeriphCLKConfig( &periph_clk_init ) != HAL_OK )
    {
    }
    
    // Configure the Systick interrupt time
    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

    // Configure the Systick
    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

/*!
 * \brief  Programmable Voltage Detector (PVD) Configuration
 *         PVD set to level 6 for a threshold around 2.9V.
 * \param  None
 * \retval None
 */
static void hal_mcu_pvd_config( void )
{
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_1;
    sConfigPVD.Mode     = PWR_PVD_MODE_IT_RISING;
    if( HAL_PWR_ConfigPVD( &sConfigPVD ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    // Enable PVD
    HAL_PWR_EnablePVD( );

    // Enable and set PVD Interrupt priority
    HAL_NVIC_SetPriority( PVD_PVM_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( PVD_PVM_IRQn );
}

static void hal_mcu_gpio_init( void )
{
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE( );
    __HAL_RCC_GPIOB_CLK_ENABLE( );
    __HAL_RCC_GPIOC_CLK_ENABLE( );
    __HAL_RCC_GPIOD_CLK_ENABLE( );
    __HAL_RCC_GPIOH_CLK_ENABLE( );

#if( HAL_HW_DEBUG_PROBE == HAL_FEATURE_ON )
    // Enable debug in sleep/stop/standby
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );
#endif
}

static void hal_mcu_gpio_deinit( void )
{
    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE( );
    __HAL_RCC_GPIOB_CLK_DISABLE( );
    __HAL_RCC_GPIOC_CLK_DISABLE( );
    __HAL_RCC_GPIOD_CLK_DISABLE( );
    __HAL_RCC_GPIOE_CLK_DISABLE( );
    __HAL_RCC_GPIOH_CLK_DISABLE( );
}

void HAL_MspInit( void )
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( MemoryManagement_IRQn, 0, 0 );
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( BusFault_IRQn, 0, 0 );
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( UsageFault_IRQn, 0, 0 );
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SVCall_IRQn, 0, 0 );
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( DebugMonitor_IRQn, 0, 0 );
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( PendSV_IRQn, 0, 0 );
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

/**
 * \brief Enters Low Power Stop Mode
 *
 * \note ARM exits the function when waking up
 */
static void hal_mcu_lpm_enter_stop_mode( void )
{
    // Disable IRQ while the MCU is not running on MSI
    CRITICAL_SECTION_BEGIN( );

    if( partial_sleep_enable == true )
    {
        hal_mcu_deinit( );
    }
    else
    {
        hal_mcu_deinit_periph( );
        hal_mcu_deinit( );
    }

    CRITICAL_SECTION_END( );
    // Enter Stop Mode
    HAL_PWREx_EnterSTOP2Mode( PWR_STOPENTRY_WFI );
}

/*!
 * \brief Exists Low Power Stop Mode
 */
static void hal_mcu_lpm_exit_stop_mode( void )
{
    // Disable IRQ while the MCU is not running on MSI
    CRITICAL_SECTION_BEGIN( );

    // Reinitializes the mcu
    hal_mcu_reinit( );

    if( partial_sleep_enable == false )
    {
        // Reinitializes the peripherals
        hal_mcu_reinit_periph( );
    }

    CRITICAL_SECTION_END( );
}

/*!
 * \brief handler low power (TODO: put in a new smtc_hal_lpm with option)
 */
void hal_mcu_low_power_handler( void )
{
#if( HAL_LOW_POWER_MODE == HAL_FEATURE_ON )
    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending
     * and cortex will not enter low power anyway
     */

    hal_mcu_lpm_enter_stop_mode( );
    hal_mcu_lpm_exit_stop_mode( );

    __enable_irq( );
#endif
}

static void hal_mcu_deinit( void )
{
    hal_spi_deinit( HAL_RADIO_SPI_ID );
    lr1110_modem_board_deinit_io( &lr1110 );
    // Disable I2C
#if(ACCELEROMETER_MOUNTED == 1)
    hal_i2c_deinit( HAL_I2C_ID );
#endif
    // Disable UART
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_deinit( HAL_PRINTF_UART_ID );
#endif
}

static void hal_mcu_reinit( void )
{
    // Reconfig needed OSC and PLL
    hal_mcu_system_clock_re_config_after_stop( );

    // Initialize I2C
#if(ACCELEROMETER_MOUNTED == 1)
    hal_i2c_init(HAL_I2C_ID, I2C_SDA, I2C_SCL);
#endif
    
    // Initialize UART
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_init( HAL_PRINTF_UART_ID, UART_TX, UART_RX);
#endif

    // Initialize SPI
    hal_spi_init( HAL_RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );
    // Init LR1110 IO
    lr1110_modem_board_init_io( &lr1110 );
}

static void hal_mcu_system_clock_re_config_after_stop( void )
{
    RCC_OscInitTypeDef       rcc_osc_init;
    RCC_ClkInitTypeDef       rcc_clk_init;
    RCC_PeriphCLKInitTypeDef periph_clk_init;

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );
    /* Ensure that MSI is wake-up system clock */ 
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

    // Initializes the CPU, AHB and APB busses clocks
    rcc_osc_init.OscillatorType      = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    rcc_osc_init.MSIState            = RCC_MSI_ON;
    rcc_osc_init.HSEState            = RCC_HSE_OFF;
    rcc_osc_init.HSIState            = RCC_HSI_OFF;
    rcc_osc_init.LSEState            = RCC_LSE_ON;
    rcc_osc_init.LSIState            = RCC_LSI_OFF;
    rcc_osc_init.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    rcc_osc_init.MSIClockRange       = RCC_MSIRANGE_11;
    rcc_osc_init.PLL.PLLState        = RCC_PLL_ON;
    rcc_osc_init.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    rcc_osc_init.PLL.PLLM            = 3;      
    rcc_osc_init.PLL.PLLN            = 10;
    rcc_osc_init.PLL.PLLP            = RCC_PLLP_DIV7;
    rcc_osc_init.PLL.PLLQ            = RCC_PLLQ_DIV2;
    rcc_osc_init.PLL.PLLR            = RCC_PLLR_DIV2;
    if( HAL_RCC_OscConfig( &rcc_osc_init ) != HAL_OK )
    {
    }

    // Initializes the CPU, AHB and APB busses clocks
    rcc_clk_init.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rcc_clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if( HAL_RCC_ClockConfig( &rcc_clk_init, FLASH_LATENCY_1 ) != HAL_OK )
    {
    }

    periph_clk_init.PeriphClockSelection =
        RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_LPTIM1 ;
    periph_clk_init.Lptim1ClockSelection  = RCC_LPTIM1CLKSOURCE_LSE;
    periph_clk_init.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
    periph_clk_init.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if( HAL_RCCEx_PeriphCLKConfig( &periph_clk_init ) != HAL_OK )
    {
    }
}

#if( HAL_LOW_POWER_MODE == HAL_FEATURE_OFF )
static bool hal_mcu_no_low_power_wait( const int32_t milliseconds )
{
    uint32_t start_time = hal_rtc_get_time_ms( );

    while( hal_rtc_get_time_ms( ) < ( start_time + milliseconds ) )
    {
        // interruptible wait for 100ms
        HAL_Delay( 100 );
        if( hal_exit_wait == true )
        {
            // stop wait/lp function and return immediatly
            hal_exit_wait = false;
            return true;
        }
    }
    return false;
}
#endif

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
static void vprint( const char* fmt, va_list argp )
{
    char string[HAL_PRINT_BUFFER_SIZE];
    if( 0 < vsprintf( string, fmt, argp ) )  // build string
    {
        hal_uart_tx( 2, ( uint8_t* ) string, strlen( string ) );
    }
}
#endif

static void on_soft_watchdog_event( void* context )
{
    HAL_DBG_TRACE_INFO( "###### ===== WATCHDOG RESET ==== ######\r\n\r\n" );
    /* System reset */
    hal_mcu_reset( );
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler( void )
{
    HAL_DBG_TRACE_ERROR( "HardFault_Handler\n\r" );
    while( 1 )
    {
    }
}

/*!
 * \brief  This function handles PVD interrupt request.
 * \param  None
 * \retval None
 */
void PVD_PVM_IRQHandler( void )
{
    HAL_DBG_TRACE_ERROR( "PVD_PVM_IRQHandler\n\r" );
    // Loop inside the handler to prevent the Cortex from using the Flash,
    // allowing the flash interface to finish any ongoing transfer.
    while( __HAL_PWR_GET_FLAG( PWR_FLAG_PVDO ) != RESET )
    {
    }
    // Then reset the board
    hal_mcu_reset( );
}

/* --- EOF ------------------------------------------------------------------ */
