/**
  ******************************************************************************
  * @file    Secure/Src/secure_nsc.c
  * @author  MCD Application Team
  * @brief   This file contains the non-secure callable APIs (secure world)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE BEGIN Non_Secure_CallLib */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "secure_nsc.h"

/** @addtogroup STM32L5xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Global variables ----------------------------------------------------------*/
void *pSecureFaultCallback = NULL;   /* Pointer to secure fault callback in Non-secure */
void *pSecureErrorCallback = NULL;   /* Pointer to secure error callback in Non-secure */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Secure registration of non-secure callback.
  * @param  CallbackId  callback identifier
  * @param  func        pointer to non-secure function
  * @retval None
  */
CMSE_NS_ENTRY void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func)
{
  if(func != NULL)
  {
    switch(CallbackId)
    {
      case SECURE_FAULT_CB_ID:           /* SecureFault Interrupt occurred */
        pSecureFaultCallback = func;
        break;
      case GTZC_ERROR_CB_ID:             /* GTZC Interrupt occurred */
        pSecureErrorCallback = func;
        break;
      default:
        /* unknown */
        break;
    }
  }
}

/**
  * @}
  */
CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_SystemClock_config( void )
{
	SystemClock_Config();
}

CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_LEDToggle_RED(void)
{
	HAL_GPIO_TogglePin(MB1_RX_GPIO_Port, MB1_RX_Pin);
}

CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_LED_RED(bool onoff)
{
	HAL_GPIO_WritePin(MB1_RX_GPIO_Port, MB1_RX_Pin, !onoff);
}

CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_LEDToggle_YELLOW(void)
{
	HAL_GPIO_TogglePin(MB1_TX_GPIO_Port, MB1_TX_Pin);
}

CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_LED_YELLOW(bool onoff)
{
	HAL_GPIO_WritePin(MB1_TX_GPIO_Port, MB1_TX_Pin, !onoff);
}

CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_GYO_DEN(bool onoff)
{
	HAL_GPIO_WritePin(GYO_DEN_GPIO_Port, GYO_DEN_Pin, onoff);
}

CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_CURRENT_CS(bool onoff)
{
	HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, onoff);
}

CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_CHARGE_CE(bool onoff)
{
	HAL_GPIO_WritePin(LDO_EN_GPIO_Port, LDO_EN_Pin, onoff);
}

CMSE_NS_ENTRY/*secureportNON_SECURE_CALLABLE*/ void SECURE_WP_EN(bool onoff)
{
	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, onoff);
}

/**
  * @}
  */
/* USER CODE END Non_Secure_CallLib */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
