/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"

#include "stm32l5xx_ll_crc.h"
#include "stm32l5xx_ll_bus.h"
#include "stm32l5xx_ll_cortex.h"
#include "stm32l5xx_ll_rcc.h"
#include "stm32l5xx_ll_system.h"
#include "stm32l5xx_ll_utils.h"
#include "stm32l5xx_ll_pwr.h"
#include "stm32l5xx_ll_gpio.h"
#include "stm32l5xx_ll_dma.h"

#include "secure_nsc.h" /* For export Non-secure callable APIs */
#include "stm32l5xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GYO_INT2_Pin GPIO_PIN_3
#define GYO_INT2_GPIO_Port GPIOE
#define WKUP_Pin GPIO_PIN_0
#define WKUP_GPIO_Port GPIOA
#define CURRENT_CS_Pin GPIO_PIN_4
#define CURRENT_CS_GPIO_Port GPIOA
#define CURRENT_SCK_Pin GPIO_PIN_5
#define CURRENT_SCK_GPIO_Port GPIOA
#define CURRENT_MISO_Pin GPIO_PIN_6
#define CURRENT_MISO_GPIO_Port GPIOA
#define CURRENT_AN_Pin GPIO_PIN_7
#define CURRENT_AN_GPIO_Port GPIOA
#define EXT_TX_Pin GPIO_PIN_4
#define EXT_TX_GPIO_Port GPIOC
#define EXT_RX_Pin GPIO_PIN_5
#define EXT_RX_GPIO_Port GPIOC
#define EXT_AN_Pin GPIO_PIN_1
#define EXT_AN_GPIO_Port GPIOB
#define ENCODER_P_Pin GPIO_PIN_9
#define ENCODER_P_GPIO_Port GPIOE
#define ENCODER_N_Pin GPIO_PIN_11
#define ENCODER_N_GPIO_Port GPIOE
#define DBG_RX_Pin GPIO_PIN_10
#define DBG_RX_GPIO_Port GPIOB
#define DBG_TX_Pin GPIO_PIN_11
#define DBG_TX_GPIO_Port GPIOB
#define LR_RFSW4_Pin GPIO_PIN_14
#define LR_RFSW4_GPIO_Port GPIOD
#define OTG_FS_CC1_Pin GPIO_PIN_15
#define OTG_FS_CC1_GPIO_Port GPIOA
#define LR_SCK_Pin GPIO_PIN_10
#define LR_SCK_GPIO_Port GPIOC
#define LR_MISO_Pin GPIO_PIN_11
#define LR_MISO_GPIO_Port GPIOC
#define LR_MOSI_Pin GPIO_PIN_12
#define LR_MOSI_GPIO_Port GPIOC
#define LR_NSS_Pin GPIO_PIN_0
#define LR_NSS_GPIO_Port GPIOD
#define LR_BUSY_Pin GPIO_PIN_1
#define LR_BUSY_GPIO_Port GPIOD
#define LR_NRST_Pin GPIO_PIN_2
#define LR_NRST_GPIO_Port GPIOD
#define EXT_SCL_Pin GPIO_PIN_8
#define EXT_SCL_GPIO_Port GPIOB
#define EXT_SDA_Pin GPIO_PIN_9
#define EXT_SDA_GPIO_Port GPIOB
#define GYO_DEN_Pin GPIO_PIN_1
#define GYO_DEN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
