/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l5xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32L5xx_IT_H
#define __STM32L5xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

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
void MemManage_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void RTC_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void DMA1_Channel6_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void TIM1_BRK_IRQHandler(void);
void TIM1_UP_IRQHandler(void);
void TIM1_TRG_COM_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM6_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void SPI1_IRQHandler(void);
void USART3_IRQHandler(void);
void LPUART1_IRQHandler(void);
void DMA2_Channel2_IRQHandler(void);
void DMA2_Channel3_IRQHandler(void);
void DMA2_Channel6_IRQHandler(void);
void RNG_IRQHandler(void);
void FPU_IRQHandler(void);
void SPI3_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L5xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
