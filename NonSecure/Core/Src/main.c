/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l552e_eval.h"
#include "lr1110-board.h"
#include "radio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//extern lr1110_t LR1110;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SecureFault_Callback(void);
void SecureError_Callback(void);
/*!
 * \brief  Tx Done callback prototype.
 */
void    LR_TxDone ( void );
/*!
 * \brief  Tx Timeout callback prototype.
 */
void    LR_TxTimeout ( void );
/*!
 * \brief Rx Done callback prototype.
 *
 * \param [IN] payload Received buffer pointer
 * \param [IN] size    Received buffer size
 * \param [IN] rssi    RSSI value computed while receiving the frame [dBm]
 * \param [IN] snr     SNR value computed while receiving the frame [dB]
 *                     FSK : N/A ( set to 0 )
 *                     LoRa: SNR value in dB
 */
void    LR_RxDone ( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
/*!
 * \brief  Rx Timeout callback prototype.
 */
void    LR_RxTimeout ( void );
/*!
 * \brief Rx Error callback prototype.
 */
void    LR_RxError ( void );
/*!
 * \brief  FHSS Change Channel callback prototype.
 *
 * \param [IN] currentChannel   Index number of the current channel
 */
void LR_FhssChangeChannel ( uint8_t currentChannel );

/*!
 * \brief CAD Done callback prototype.
 *
 * \param [IN] channelDetected    Channel Activity detected during the CAD
 */
void LR_CadDone ( bool channelActivityDetected );

/*!
 * \brief  Gnss Done Done callback prototype.
*/
void    LR_GnssDone( void );

/*!
 * \brief  Wifi Done Done callback prototype.
*/
void    LR_WifiDone( void );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float pfData[3];
RadioEvents_t lrEvent = { LR_TxDone, LR_TxTimeout,
		LR_RxDone, LR_RxTimeout, LR_RxError,
		LR_FhssChangeChannel, LR_CadDone, LR_GnssDone, LR_WifiDone
};
#define RF_FREQUENCY                                923000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
  /* Register SecureFault callback defined in non-secure and to be called by secure handler */
    SECURE_RegisterCallback(SECURE_FAULT_CB_ID, (void *)SecureFault_Callback);

    /* Register SecureError callback defined in non-secure and to be called by secure handler */
    SECURE_RegisterCallback(GTZC_ERROR_CB_ID, (void *)SecureError_Callback);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_LPUART1_UART_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  SpiInit(&LR1110.spi, SPI_3, LR_MOSI_GPIO_Port, LR_MOSI_Pin,
		  LR_MISO_GPIO_Port, LR_MISO_Pin, LR_SCK_GPIO_Port, LR_SCK_Pin, NULL, NC);

  lr1110_board_init_io( &LR1110 );

  Radio.Init(&lrEvent);

  Radio.SetModem(MODEM_LORA);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetPublicNetwork(LR1110_RADIO_LORA_NETWORK_PUBLIC);
  Radio.SetMaxPayloadLength( MODEM_LORA, 255 );

//  Radio.SetTxConfig();
  Radio.SetRxConfig( MODEM_LORA, LR1110_RADIO_LORA_BW_125,
		  LR1110_RADIO_LORA_SF10, LR1110_RADIO_LORA_CR_4_5,
		  0, LORA_PREAMBLE_LENGTH,
          LORA_SYMBOL_TIMEOUT, true,
          8,
		  LR1110_RADIO_LORA_CRC_ON, false, 0,
		  LR1110_RADIO_LORA_IQ_STANDARD, true );

  BSP_GYRO_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  Radio.Rx(0);

  while (1)
  {
	  if( Radio.IrqProcess != NULL )
	          {
	              Radio.IrqProcess( );
	          }
	  BSP_GYRO_GetXYZ(pfData);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  SECURE_LEDToggle_YELLOW();
	  HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Callback called by secure code following a secure fault interrupt
  * @note   This callback is called by secure code thanks to the registration
  *         done by the non-secure application with non-secure callable API
  *         SECURE_RegisterCallback(SECURE_FAULT_CB_ID, (void *)SecureFault_Callback);
  * @retval None
  */
void SecureFault_Callback(void)
{
  /* Go to error infinite loop when Secure fault generated by IDAU/SAU check */
  /* because of illegal access */
  Error_Handler();
}


/**
  * @brief  Callback called by secure code following a GTZC TZIC secure interrupt (GTZC_IRQn)
  * @note   This callback is called by secure code thanks to the registration
  *         done by the non-secure application with non-secure callable API
  *         SECURE_RegisterCallback(GTZC_ERROR_CB_ID, (void *)SecureError_Callback);
  * @retval None
  */
void SecureError_Callback(void)
{
  /* Go to error infinite loop when Secure error generated by GTZC check */
  /* because of illegal access */
  Error_Handler();
}

/*!
 * \brief  Tx Done callback prototype.
 */
void LR_TxDone ( void )
{

}

/*!
 * \brief  Tx Timeout callback prototype.
 */
void LR_TxTimeout ( void )
{

}

/*!
 * \brief Rx Done callback prototype.
 *
 * \param [IN] payload Received buffer pointer
 * \param [IN] size    Received buffer size
 * \param [IN] rssi    RSSI value computed while receiving the frame [dBm]
 * \param [IN] snr     SNR value computed while receiving the frame [dB]
 *                     FSK : N/A ( set to 0 )
 *                     LoRa: SNR value in dB
 */
void LR_RxDone ( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{

}

/*!
 * \brief  Rx Timeout callback prototype.
 */
void LR_RxTimeout ( void )
{

}

/*!
 * \brief Rx Error callback prototype.
 */
void LR_RxError ( void )
{

}

/*!
 * \brief  FHSS Change Channel callback prototype.
 *
 * \param [IN] currentChannel   Index number of the current channel
 */
void LR_FhssChangeChannel ( uint8_t currentChannel ){

}

/*!
 * \brief CAD Done callback prototype.
 *
 * \param [IN] channelDetected    Channel Activity detected during the CAD
 */
void LR_CadDone ( bool channelActivityDetected )
{

}

/*!
 * \brief  Gnss Done Done callback prototype.
*/
void    LR_GnssDone( void ) {

}

/*!
 * \brief  Gnss Done Done callback prototype.
*/
void    LR_WifiDone( void ) {

}


/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  SECURE_LEDToggle_RED();
	  HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
