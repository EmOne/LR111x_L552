/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "timer.h"

/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */
/*!
 * \brief Temperature coefficient of the clock source
 */
#define RTC_TEMP_COEFFICIENT                            ( -0.035f )

/*!
 * \brief Temperature coefficient deviation of the clock source
 */
#define RTC_TEMP_DEV_COEFFICIENT                        ( 0.0035f )

/*!
 * \brief Turnover temperature of the clock source
 */
#define RTC_TEMP_TURNOVER                               ( 25.0f )

/*!
 * \brief Turnover temperature deviation of the clock source
 */
#define RTC_TEMP_DEV_TURNOVER                           ( 5.0f )


/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */

/*!
 * \brief Initializes the RTC timer
 *
 * \remark The timer is based on the RTC
 */
void RtcInit( void );

/*!
 * \brief Returns the minimum timeout value
 *
 * \retval minTimeout Minimum timeout value in in ticks
 */
uint32_t RtcGetMinimumTimeout( void );

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( TimerTime_t milliseconds );

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
TimerTime_t RtcTick2Ms( uint32_t tick );

/*!
 * \brief Performs a delay of milliseconds by polling RTC
 *
 * \param[IN] milliseconds Delay in ms
 */
void RtcDelayMs( TimerTime_t milliseconds );

/*!
 * \brief Calculates the wake up time between wake up and MCU start
 *
 * \note Resolution in RTC_ALARM_TIME_BASE
 */
void RtcSetMcuWakeUpTime( void );

/*!
 * \brief Returns the wake up time in ticks
 *
 * \retval wakeUpTime The WakeUpTime value in ticks
 */
int16_t RtcGetMcuWakeUpTime( void );

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this funtion) + timeout
 *
 * \param timeout [IN] Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout );

/*!
 * \brief Stops the Alarm
 */
void RtcStopAlarm( void );

/*!
 * \brief Starts wake up alarm
 *
 * \note  Alarm in RtcTimerContext.Time + timeout
 *
 * \param [IN] timeout Timeout value in ticks
 */
void RtcStartAlarm( uint32_t timeout );

/*!
 * \brief Sets the RTC timer reference
 *
 * \retval value Timer reference value in ticks
 */
uint32_t RtcSetTimerContext( void );

/*!
 * \brief Gets the RTC timer reference
 *
 * \retval value Timer value in ticks
 */
uint32_t RtcGetTimerContext( void );

/*!
 * \brief Gets the system time with the number of seconds elapsed since epoch
 *
 * \param [OUT] milliseconds Number of milliseconds elapsed since epoch
 * \retval seconds Number of seconds elapsed since epoch
 */
uint32_t RtcGetCalendarTime( uint16_t *milliseconds );

/*!
 * \brief Get the RTC timer value
 *
 * \retval RTC Timer value
 */
uint32_t RtcGetTimerValue( void );

/*!
 * \brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * \retval RTC Elapsed time since the last alarm in ticks.
 */
uint32_t RtcGetTimerElapsedTime( void );

/*!
 * \brief Writes data0 and data1 to the RTC backup registers
 *
 * \param [IN] data0 1st Data to be written
 * \param [IN] data1 2nd Data to be written
 */
void RtcBkupWrite( uint32_t data0, uint32_t data1 );

/*!
 * \brief Reads data0 and data1 from the RTC backup registers
 *
 * \param [OUT] data0 1st Data to be read
 * \param [OUT] data1 2nd Data to be read
 */
void RtcBkupRead( uint32_t* data0, uint32_t* data1 );

/*!
 * \brief Processes pending timer events
 */
void RtcProcess( void );

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period Time period to compensate in milliseconds
 * \param [IN] temperature Current temperature
 *
 * \retval Compensated time period
 */
TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature );

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
