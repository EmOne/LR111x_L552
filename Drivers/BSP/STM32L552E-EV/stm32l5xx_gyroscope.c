/**
  ******************************************************************************
  * @file    stm32l5xx_gyroscope.c
  * @author  EmOne Application Team
  * @brief   This file provides a set of functions needed to manage the ICG20330
  *          MEMS accelerometer available on STM32L5Xx board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 EmOne.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by EmOne under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_gyroscope.h"
#include "stm32l552e_eval_bus.h"
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L5XX
  * @{
  */

/** @defgroup STM32L5XX_GYROSCOPE STM32L476G-DISCOVERY GYROSCOPE
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup STM32L5XX_GYROSCOPE_Private_Types Private Types
  * @{
  */
/**
  * @}
  */

/* Private defines ------------------------------------------------------------*/
/** @defgroup STM32L5XX_GYROSCOPE_Private_Constants Private Constants
  * @{
  */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup STM32L5XX_GYROSCOPE_Private_Macros Private Macros
  * @{
  */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32L5XX_GYROSCOPE_Private_Variables Private Variables
  * @{
  */
static GYRO_DrvTypeDef *GyroscopeDrv;

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup STM32L5XX_GYROSCOPE_Private_FunctionPrototypes Private Functions
  * @{
  */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup STM32L5XX_GYROSCOPE_Exported_Functions
  * @{
  */

/**
  * @brief  Initialize Gyroscope.
  * @retval GYRO_OK or GYRO_ERROR
  */
uint8_t BSP_GYRO_Init(void)
{
  uint8_t ret = GYRO_ERROR;
//  uint16_t ctrl = 0x0000;
  GYRO_InitTypeDef ICG20330_InitStructure;
//  GYRO_FilterConfigTypeDef ICG20330_FilterStructure = {0, 0};

  if (icg20330Drv.ReadID() == I_AM_ICG20330)
  {
    /* Initialize the gyroscope driver structure */
    GyroscopeDrv = &icg20330Drv;

    /* Configure Mems : data rate, power mode, full scale and axes */
    ICG20330_InitStructure.Power_Mode = ICG20330_MODE_ACTIVE;
    ICG20330_InitStructure.Output_DataRate = ICG20330_OUTPUT_DATARATE_1;
    ICG20330_InitStructure.Axes_Enable = ICG20330_AXES_ENABLE;
    ICG20330_InitStructure.Band_Width = ICG20330_BANDWIDTH_250;
    ICG20330_InitStructure.BlockData_Update = ICG20330_BlockDataUpdate_Continous;
    ICG20330_InitStructure.Endianness = ICG20330_BLE_LSB;
    ICG20330_InitStructure.Full_Scale = ICG20330_FULLSCALE_500;

    /* Initialize component */
    GyroscopeDrv->Init(&ICG20330_InitStructure);

    /* Configure component filter */
      GyroscopeDrv->FilterConfig(0x00) ;

    /* Enable component filter */
    GyroscopeDrv->FilterCmd(0x1B);

    ret = GYRO_OK;
  }
  else
  {
    ret = GYRO_ERROR;
  }

  return ret;
}


/**
  * @brief  DeInitialize Gyroscope.
  * @retval None
  */
void BSP_GYRO_DeInit(void)
{
  GYRO_IO_DeInit();
}


/**
  * @brief  Put Gyroscope in low power mode.
  * @retval None
  */
void BSP_GYRO_LowPower(void)
{
  uint16_t ctrl = 0x0000;
  GYRO_InitTypeDef ICG20330_InitStructure;

  /* configure only Power_Mode field */
  ICG20330_InitStructure.Power_Mode = ICG20330_MODE_POWERDOWN;

  ctrl = (uint16_t)(ICG20330_InitStructure.Power_Mode);

  /* Set component in low-power mode */
  GyroscopeDrv->LowPower(ctrl);


}

/**
  * @brief  Read ID of Gyroscope component.
  * @retval ID
  */
uint8_t BSP_GYRO_ReadID(void)
{
  uint8_t id = 0x00;

  if (GyroscopeDrv->ReadID != NULL)
  {
    id = GyroscopeDrv->ReadID();
  }
  return id;
}

/**
  * @brief  Reboot memory content of Gyroscope.
  * @retval None
  */
void BSP_GYRO_Reset(void)
{
  if (GyroscopeDrv->Reset != NULL)
  {
    GyroscopeDrv->Reset();
  }
}

/**
  * @brief  Configure Gyroscope interrupts (INT1 or INT2).
  * @param  pIntConfig: pointer to a GYRO_InterruptConfigTypeDef
  *         structure that contains the configuration setting for the ICG20330 Interrupt.
  * @retval None
  */
void BSP_GYRO_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfig)
{
  uint16_t interruptconfig = 0x0000;

  if (GyroscopeDrv->ConfigIT != NULL)
  {
    /* Configure latch Interrupt request and axe interrupts */
    interruptconfig |= ((uint8_t)(pIntConfig->Latch_Request | \
                                  pIntConfig->Interrupt_Axes) << 8);

    interruptconfig |= (uint8_t)(pIntConfig->Interrupt_ActiveEdge);

    GyroscopeDrv->ConfigIT(interruptconfig);
  }
}

/**
  * @brief  Enable Gyroscope interrupts (INT1 or INT2).
  * @param  IntPin: Interrupt pin
  *      This parameter can be:
  *        @arg ICG20330_INT1
  *        @arg ICG20330_INT2
  * @retval None
  */
void BSP_GYRO_EnableIT(uint8_t IntPin)
{
  if (GyroscopeDrv->EnableIT != NULL)
  {
    GyroscopeDrv->EnableIT(IntPin);
  }
}

/**
  * @brief  Disable Gyroscope interrupts (INT1 or INT2).
  * @param  IntPin: Interrupt pin
  *      This parameter can be:
  *        @arg ICG20330_INT1
  *        @arg ICG20330_INT2
  * @retval None
  */
void BSP_GYRO_DisableIT(uint8_t IntPin)
{
  if (GyroscopeDrv->DisableIT != NULL)
  {
    GyroscopeDrv->DisableIT(IntPin);
  }
}

/**
  * @brief  Get XYZ angular acceleration from the Gyroscope.
  * @param  pfData: pointer on floating array
  * @retval None
  */
void BSP_GYRO_GetXYZ(float *pfData)
{
  if (GyroscopeDrv->GetXYZ != NULL)
  {
    GyroscopeDrv->GetXYZ(pfData);
  }
}

/**
  * @}
  */
void    GYRO_IO_Init(void)
{
	uint8_t data = 0x81;

	BSP_I2C1_Init();

	HAL_GPIO_WritePin(GYO_DEN_GPIO_Port, GYO_DEN_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);

	GYRO_IO_Write(&data, ICG20330_PWR_MGMT_1_ADDR, 1);
	HAL_Delay(100);
}
void    GYRO_IO_DeInit(void)
{
	BSP_I2C1_DeInit();
}
void    GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	uint16_t DevAddress = (ICG20330_ADDR << 1) | 0x00;
	BSP_I2C1_WriteReg(DevAddress, WriteAddr, pBuffer, NumByteToWrite);

}
void    GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	uint16_t DevAddress = (ICG20330_ADDR << 1) | 0x01;
	BSP_I2C1_ReadReg(DevAddress, ReadAddr, pBuffer, NumByteToRead);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
