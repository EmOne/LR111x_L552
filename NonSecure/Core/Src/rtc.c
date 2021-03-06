/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */
#include <math.h>
#include <time.h>
#include "stm32l5xx.h"
#include "utilities.h"
#include "delay.h"
#include "main.h"
#include "timer.h"
//#include "systime.h"
#include "gpio.h"
//#include "sysIrqHandlers.h"
#include "lpm-board.h"
//#include "rtc-board.h"
/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm B
  */
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp_IT(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable Calibration
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_1HZ) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_RTCAPB_CLK_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// MCU Wake Up Time
#define MIN_ALARM_DELAY                             3 // in ticks

// sub-second number of bits
#define N_PREDIV_S                                  10

// Synchronous prediv
#define PREDIV_S                                    ( ( 1 << N_PREDIV_S ) - 1 )

// Asynchronous prediv
#define PREDIV_A                                    ( 1 << ( 15 - N_PREDIV_S ) ) - 1

// Sub-second mask definition
#define ALARM_SUBSECOND_MASK                        ( N_PREDIV_S << RTC_ALRMASSR_MASKSS_Pos )

// RTC Time base in us
#define USEC_NUMBER                                 1000000
#define MSEC_NUMBER                                 ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR                               3
#define CONV_NUMER                                  ( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM                                  ( 1 << ( N_PREDIV_S - COMMON_FACTOR ) )

/*!
 * \brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR                           ( ( uint32_t )  366U )
#define DAYS_IN_YEAR                                ( ( uint32_t )  365U )
#define SECONDS_IN_1DAY                             ( ( uint32_t )86400U )
#define SECONDS_IN_1HOUR                            ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE                          ( ( uint32_t )   60U )
#define MINUTES_IN_1HOUR                            ( ( uint32_t )   60U )
#define HOURS_IN_1DAY                               ( ( uint32_t )   24U )

/*!
 * \brief Correction factors
 */
#define  DAYS_IN_MONTH_CORRECTION_NORM              ( ( uint32_t )0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP              ( ( uint32_t )0x445550 )

/*!
 * \brief Calculates ceiling( X / N )
 */
#define DIVC( X, N )                                ( ( ( X ) + ( N ) -1 ) / ( N ) )

/*!
 * RTC timer context
 */
typedef struct
{
    uint32_t        Time;         // Reference time
    RTC_TimeTypeDef CalendarTime; // Reference time in calendar format
    RTC_DateTypeDef CalendarDate; // Reference date in calendar format
}RtcTimerContext_t;

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;

/*!
 * \brief Indicates if the RTC Wake Up Time is calibrated or not
 */
static bool McuWakeUpTimeInitialized = false;

/*!
 * \brief Compensates MCU wakeup time
 */
static int16_t McuWakeUpTimeCal = 0;

/*!
 * Number of days in each month on a normal year
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * \brief RTC Handle
 */
static RTC_HandleTypeDef RtcHandle =
{
    .Instance = NULL,
    .Init =
    {
        .HourFormat = 0,
        .AsynchPrediv = 0,
        .SynchPrediv = 0,
        .OutPut = 0,
        .OutPutPolarity = 0,
        .OutPutType = 0
    },
    .Lock = HAL_UNLOCKED,
    .State = HAL_RTC_STATE_RESET
};

/*!
 * \brief RTC Alarm
 */
static RTC_AlarmTypeDef RtcAlarm;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

/*!
 * \brief Get the current time from calendar in ticks
 *
 * \param [IN] date           Pointer to RTC_DateStruct
 * \param [IN] time           Pointer to RTC_TimeStruct
 * \retval calendarValue Time in ticks
 */
static uint64_t RtcGetCalendarValue( RTC_DateTypeDef* date, RTC_TimeTypeDef* time );

void RtcInit( void )
{
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;

    if( RtcInitialized == false )
    {
        __HAL_RCC_RTC_ENABLE( );

        RtcHandle.Instance            = RTC;
        RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
        RtcHandle.Init.AsynchPrediv   = PREDIV_A;  // RTC_ASYNCH_PREDIV;
        RtcHandle.Init.SynchPrediv    = PREDIV_S;  // RTC_SYNCH_PREDIV;
        RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
        RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
        RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
        HAL_RTC_Init( &RtcHandle );

        date.Year                     = 0;
        date.Month                    = RTC_MONTH_JANUARY;
        date.Date                     = 1;
        date.WeekDay                  = RTC_WEEKDAY_MONDAY;
        HAL_RTC_SetDate( &RtcHandle, &date, RTC_FORMAT_BIN );

        /*at 0:0:0*/
        time.Hours                    = 0;
        time.Minutes                  = 0;
        time.Seconds                  = 0;
        time.SubSeconds               = 0;
        time.TimeFormat               = 0;
        time.StoreOperation           = RTC_STOREOPERATION_RESET;
        time.DayLightSaving           = RTC_DAYLIGHTSAVING_NONE;
        HAL_RTC_SetTime( &RtcHandle, &time, RTC_FORMAT_BIN );

        // Enable Direct Read of the calendar registers (not through Shadow registers)
        HAL_RTCEx_EnableBypassShadow( &RtcHandle );

        HAL_NVIC_SetPriority( RTC_IRQn, 1, 0 );
        HAL_NVIC_EnableIRQ( RTC_IRQn );

        // Init alarm.
        HAL_RTC_DeactivateAlarm( &RtcHandle, RTC_ALARM_A );

        RtcSetTimerContext( );
        RtcInitialized = true;
    }
}

/*!
 * \brief Sets the RTC timer reference, sets also the RTC_DateStruct and RTC_TimeStruct
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcSetTimerContext( void )
{
    RtcTimerContext.Time = ( uint32_t )RtcGetCalendarValue( &RtcTimerContext.CalendarDate, &RtcTimerContext.CalendarTime );
    return ( uint32_t )RtcTimerContext.Time;
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcGetTimerContext( void )
{
    return RtcTimerContext.Time;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( uint32_t milliseconds )
{
    return ( uint32_t )( ( ( ( uint64_t )milliseconds ) * CONV_DENOM ) / CONV_NUMER );
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
uint32_t RtcTick2Ms( uint32_t tick )
{
    uint32_t seconds = tick >> N_PREDIV_S;

    tick = tick & PREDIV_S;
    return ( ( seconds * 1000 ) + ( ( tick * 1000 ) >> N_PREDIV_S ) );
}

/*!
 * \brief a delay of delay ms by polling RTC
 *
 * \param[IN] delay in ms
 */
void RtcDelayMs( uint32_t delay )
{
    uint64_t delayTicks = 0;
    uint64_t refTicks = RtcGetTimerValue( );

    delayTicks = RtcMs2Tick( delay );

    // Wait delay ms
    while( ( ( RtcGetTimerValue( ) - refTicks ) ) < delayTicks )
    {
        __NOP( );
    }
}

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this function) + timeout
 *
 * \param timeout Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
    // We don't go in Low Power mode for timeout below MIN_ALARM_DELAY
    if( ( int64_t )( MIN_ALARM_DELAY + McuWakeUpTimeCal ) < ( int64_t )( timeout - RtcGetTimerElapsedTime( ) ) )
    {
        LpmSetStopMode( LPM_RTC_ID, LPM_ENABLE );
    }
    else
    {
        LpmSetStopMode( LPM_RTC_ID, LPM_DISABLE );
    }

    // In case stop mode is required
    if( LpmGetMode( ) == LPM_STOP_MODE )
    {
        timeout = timeout - McuWakeUpTimeCal;
    }

    RtcStartAlarm( timeout );
}

void RtcStopAlarm( void )
{
    // Disable the Alarm A interrupt
    HAL_RTC_DeactivateAlarm( &RtcHandle, RTC_ALARM_A );

    // Clear RTC Alarm Flag
    __HAL_RTC_ALARM_CLEAR_FLAG( &RtcHandle, RTC_FLAG_ALRAF );

    // Clear the EXTI's line Flag for RTC Alarm
//    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG( );
}

void RtcStartAlarm( uint32_t timeout )
{
    uint16_t rtcAlarmSubSeconds = 0;
    uint16_t rtcAlarmSeconds = 0;
    uint16_t rtcAlarmMinutes = 0;
    uint16_t rtcAlarmHours = 0;
    uint16_t rtcAlarmDays = 0;
    RTC_TimeTypeDef time = RtcTimerContext.CalendarTime;
    RTC_DateTypeDef date = RtcTimerContext.CalendarDate;

    RtcStopAlarm( );

    /*reverse counter */
    rtcAlarmSubSeconds =  PREDIV_S - time.SubSeconds;
    rtcAlarmSubSeconds += ( timeout & PREDIV_S );
    // convert timeout  to seconds
    timeout >>= N_PREDIV_S;

    // Convert microsecs to RTC format and add to 'Now'
    rtcAlarmDays =  date.Date;
    while( timeout >= SECONDS_IN_1DAY )
    {
        timeout -= SECONDS_IN_1DAY;
        rtcAlarmDays++;
    }

    // Calc hours
    rtcAlarmHours = time.Hours;
    while( timeout >= SECONDS_IN_1HOUR )
    {
        timeout -= SECONDS_IN_1HOUR;
        rtcAlarmHours++;
    }

    // Calc minutes
    rtcAlarmMinutes = time.Minutes;
    while( timeout >= SECONDS_IN_1MINUTE )
    {
        timeout -= SECONDS_IN_1MINUTE;
        rtcAlarmMinutes++;
    }

    // Calc seconds
    rtcAlarmSeconds =  time.Seconds + timeout;

    //***** Correct for modulo********
    while( rtcAlarmSubSeconds >= ( PREDIV_S + 1 ) )
    {
        rtcAlarmSubSeconds -= ( PREDIV_S + 1 );
        rtcAlarmSeconds++;
    }

    while( rtcAlarmSeconds >= SECONDS_IN_1MINUTE )
    {
        rtcAlarmSeconds -= SECONDS_IN_1MINUTE;
        rtcAlarmMinutes++;
    }

    while( rtcAlarmMinutes >= MINUTES_IN_1HOUR )
    {
        rtcAlarmMinutes -= MINUTES_IN_1HOUR;
        rtcAlarmHours++;
    }

    while( rtcAlarmHours >= HOURS_IN_1DAY )
    {
        rtcAlarmHours -= HOURS_IN_1DAY;
        rtcAlarmDays++;
    }

    if( date.Year % 4 == 0 )
    {
        if( rtcAlarmDays > DaysInMonthLeapYear[date.Month - 1] )
        {
            rtcAlarmDays = rtcAlarmDays % DaysInMonthLeapYear[date.Month - 1];
        }
    }
    else
    {
        if( rtcAlarmDays > DaysInMonth[date.Month - 1] )
        {
            rtcAlarmDays = rtcAlarmDays % DaysInMonth[date.Month - 1];
        }
    }

    /* Set RTC_AlarmStructure with calculated values*/
    RtcAlarm.AlarmTime.SubSeconds     = PREDIV_S - rtcAlarmSubSeconds;
    RtcAlarm.AlarmSubSecondMask       = ALARM_SUBSECOND_MASK;
    RtcAlarm.AlarmTime.Seconds        = rtcAlarmSeconds;
    RtcAlarm.AlarmTime.Minutes        = rtcAlarmMinutes;
    RtcAlarm.AlarmTime.Hours          = rtcAlarmHours;
    RtcAlarm.AlarmDateWeekDay         = ( uint8_t )rtcAlarmDays;
    RtcAlarm.AlarmTime.TimeFormat     = time.TimeFormat;
    RtcAlarm.AlarmDateWeekDaySel      = RTC_ALARMDATEWEEKDAYSEL_DATE;
    RtcAlarm.AlarmMask                = RTC_ALARMMASK_NONE;
    RtcAlarm.Alarm                    = RTC_ALARM_A;
    RtcAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    RtcAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;

    // Set RTC_Alarm
    HAL_RTC_SetAlarm_IT( &RtcHandle, &RtcAlarm, RTC_FORMAT_BIN );
}

uint32_t RtcGetTimerValue( void )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    uint32_t calendarValue = ( uint32_t )RtcGetCalendarValue( &date, &time );

    return( calendarValue );
}

uint32_t RtcGetTimerElapsedTime( void )
{
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;

  uint32_t calendarValue = ( uint32_t )RtcGetCalendarValue( &date, &time );

  return( ( uint32_t )( calendarValue - RtcTimerContext.Time ) );
}

void RtcSetMcuWakeUpTime( void )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    uint32_t now, hit;
    int16_t mcuWakeUpTime;

    if( ( McuWakeUpTimeInitialized == false ) &&
       ( HAL_NVIC_GetPendingIRQ( RTC_IRQn ) == 1 ) )
    {
        /* WARNING: Works ok if now is below 30 days
         *          it is ok since it's done once at first alarm wake-up
         */
        McuWakeUpTimeInitialized = true;
        now = ( uint32_t )RtcGetCalendarValue( &date, &time );

        HAL_RTC_GetAlarm( &RtcHandle, &RtcAlarm, RTC_ALARM_A, RTC_FORMAT_BIN );
        hit = RtcAlarm.AlarmTime.Seconds +
              60 * ( RtcAlarm.AlarmTime.Minutes +
              60 * ( RtcAlarm.AlarmTime.Hours +
              24 * ( RtcAlarm.AlarmDateWeekDay - 1 ) ) );
        hit = ( hit << N_PREDIV_S ) + ( PREDIV_S - RtcAlarm.AlarmTime.SubSeconds );

        mcuWakeUpTime = ( int16_t )( ( now - hit ) );
        McuWakeUpTimeCal += mcuWakeUpTime;
        //PRINTF( 3, "Cal=%d, %d\n", McuWakeUpTimeCal, mcuWakeUpTime);
    }
}

int16_t RtcGetMcuWakeUpTime( void )
{
    return McuWakeUpTimeCal;
}

static uint64_t RtcGetCalendarValue( RTC_DateTypeDef* date, RTC_TimeTypeDef* time )
{
    uint64_t calendarValue = 0;
    uint32_t firstRead;
    uint32_t correction;
    uint32_t seconds;

    // Make sure it is correct due to asynchronus nature of RTC
    do
    {
        firstRead = RTC->SSR;
        HAL_RTC_GetDate( &RtcHandle, date, RTC_FORMAT_BIN );
        HAL_RTC_GetTime( &RtcHandle, time, RTC_FORMAT_BIN );
    }while( firstRead != RTC->SSR );

    // Calculte amount of elapsed days since 01/01/2000
    seconds = DIVC( ( DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR ) * date->Year , 4 );

    correction = ( ( date->Year % 4 ) == 0 ) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM;

    seconds += ( DIVC( ( date->Month-1 ) * ( 30 + 31 ), 2 ) - ( ( ( correction >> ( ( date->Month - 1 ) * 2 ) ) & 0x03 ) ) );

    seconds += ( date->Date -1 );

    // Convert from days to seconds
    seconds *= SECONDS_IN_1DAY;

    seconds += ( ( uint32_t )time->Seconds +
                 ( ( uint32_t )time->Minutes * SECONDS_IN_1MINUTE ) +
                 ( ( uint32_t )time->Hours * SECONDS_IN_1HOUR ) ) ;

    calendarValue = ( ( ( uint64_t )seconds ) << N_PREDIV_S ) + ( PREDIV_S - time->SubSeconds );

    return( calendarValue );
}

uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    RTC_TimeTypeDef time ;
    RTC_DateTypeDef date;
    uint32_t ticks;

    uint64_t calendarValue = RtcGetCalendarValue( &date, &time );

    uint32_t seconds = ( uint32_t )( calendarValue >> N_PREDIV_S );

    ticks =  ( uint32_t )calendarValue & PREDIV_S;

    *milliseconds = RtcTick2Ms( ticks );

    return seconds;
}

/*!
 * \brief RTC IRQ Handler of the RTC Alarm
 */
void RTC_Alarm_IRQHandler( void )
{
    RTC_HandleTypeDef* hrtc = &RtcHandle;

    // Enable low power at irq
    LpmSetStopMode( LPM_RTC_ID, LPM_ENABLE );

    // Clear the EXTI's line Flag for RTC Alarm
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG( );

    // Gets the AlarmA interrupt source enable status
    if( __HAL_RTC_ALARM_GET_IT_SOURCE( hrtc, RTC_IT_ALRA ) != RESET )
    {
        // Gets the pending status of the AlarmA interrupt
        if( __HAL_RTC_ALARM_GET_FLAG( hrtc, RTC_FLAG_ALRAF ) != RESET )
        {
            // Clear the AlarmA interrupt pending bit
            __HAL_RTC_ALARM_CLEAR_FLAG( hrtc, RTC_FLAG_ALRAF );
            // AlarmA callback
            HAL_RTC_AlarmAEventCallback( hrtc );
        }
    }
}

/*!
 * \brief  Alarm A callback.
 *
 * \param [IN] hrtc RTC handle
 */
void HAL_RTC_AlarmAEventCallback( RTC_HandleTypeDef *hrtc )
{
    TimerIrqHandler( );
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
    HAL_RTCEx_BKUPWrite( &RtcHandle, RTC_BKP_DR0, data0 );
    HAL_RTCEx_BKUPWrite( &RtcHandle, RTC_BKP_DR1, data1 );
}

void RtcBkupRead( uint32_t *data0, uint32_t *data1 )
{
  *data0 = HAL_RTCEx_BKUPRead( &RtcHandle, RTC_BKP_DR0 );
  *data1 = HAL_RTCEx_BKUPRead( &RtcHandle, RTC_BKP_DR1 );
}

void RtcProcess( void )
{
    // Not used on this platform.
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    float k = RTC_TEMP_COEFFICIENT;
    float kDev = RTC_TEMP_DEV_COEFFICIENT;
    float t = RTC_TEMP_TURNOVER;
    float tDev = RTC_TEMP_DEV_TURNOVER;
    float interim = 0.0f;
    float ppm = 0.0f;

    if( k < 0.0f )
    {
        ppm = ( k - kDev );
    }
    else
    {
        ppm = ( k + kDev );
    }
    interim = ( temperature - ( t - tDev ) );
    ppm *=  interim * interim;

    // Calculate the drift in time
    interim = ( ( float ) period * ppm ) / 1000000.0f;
    // Calculate the resulting time period
    interim += period;
    interim = floor( interim );

    if( interim < 0.0f )
    {
        interim = ( float )period;
    }

    // Calculate the resulting period
    return ( TimerTime_t ) interim;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
