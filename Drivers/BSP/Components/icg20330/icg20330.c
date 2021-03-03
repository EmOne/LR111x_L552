/**
  ******************************************************************************
  * @file    icg20330.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    26-June-2015
  * @brief   This file provides a set of functions needed to manage the ICG20330,
  *          ST MEMS motion sensor, 3-axis digital output gyroscope.  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "icg20330.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 

/** @addtogroup ICG20330
  * @{
  */

/** @defgroup ICG20330_Private_TypesDefinitions
  * @{
  */
  
/**
  * @}
  */

/** @defgroup ICG20330_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup ICG20330_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup ICG20330_Private_Variables
  * @{
  */ 
GYRO_DrvTypeDef icg20330Drv =
{
  ICG20330_Init,
  ICG20330_DeInit,
  ICG20330_ReadID,
  ICG20330_RebootCmd,
  ICG20330_LowPower,
  ICG20330_INT1InterruptConfig,
  ICG20330_EnableIT,
  ICG20330_DisableIT,
  0,
  0,
  ICG20330_FilterConfig,
  ICG20330_FilterCmd,
  ICG20330_ReadXYZAngRate
};

/**
  * @}
  */

/** @defgroup ICG20330_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup ICG20330_Private_Functions
  * @{
  */

/**
  * @brief  Set ICG20330 Initialization.
  * @param  ICG20330_InitStruct: pointer to a ICG20330_InitTypeDef structure
  *         that contains the configuration setting for the ICG20330.
  * @retval None
  */
void ICG20330_Init(void* InitStruct)
{  
  GYRO_InitTypeDef *init = (GYRO_InitTypeDef *) InitStruct;
  uint8_t ctrl = 0x00;
  /* Configure the low level interface */
  GYRO_IO_Init();
  
  switch (init->Power_Mode) {
	case ICG20330_MODE_POWERDOWN:
		ctrl = 0xFF;
		break;
	case ICG20330_MODE_STANDBY:
		ctrl = 0x10;
		break;
	case ICG20330_MODE_SLEEP:
		ctrl = 0x40;
		break;
	case ICG20330_MODE_ACTIVE:
		ctrl = 0x01;
		break;
	default:
		break;
  }
  GYRO_IO_Write(&ctrl, ICG20330_PWR_MGMT_1_ADDR, 1);
  
  ctrl &=~ (ICG20330_STBY_XG_DISABLE | ICG20330_STBY_YG_DISABLE | ICG20330_STBY_ZG_DISABLE);
  GYRO_IO_Write(&ctrl, ICG20330_PWR_MGMT_2_ADDR, 1);
}



/**
  * @brief ICG20330 De-initialization
  * @param  None
  * @retval None
  */
void ICG20330_DeInit(void)
{
}

/**
  * @brief  Read ID address of ICG20330
  * @param  None
  * @retval ID name
  */
uint8_t ICG20330_ReadID(void)
{
  uint8_t tmp;
  
  /* Configure the low level interface */
  GYRO_IO_Init();
  
  /* Read WHO I AM register */
  GYRO_IO_Read(&tmp, ICG20330_WHO_AM_I_ADDR, 1);
  
  /* Return the ID */
  return (uint8_t)tmp;
}

/**
  * @brief  Reboot memory content of ICG20330
  * @param  None
  * @retval None
  */
void ICG20330_RebootCmd(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  GYRO_IO_Read(&tmpreg, ICG20330_PWR_MGMT_1_ADDR, 1);
  
  /* Enable or Disable the reboot memory */
  tmpreg |= ICG20330_DEV_RESET;
  
  /* Write value to MEMS CTRL_REG5 register */
  GYRO_IO_Write(&tmpreg, ICG20330_PWR_MGMT_1_ADDR, 1);
}

/**
  * @brief Set ICG20330 in low-power mode
  * @param 
  * @retval  None
  */
void ICG20330_LowPower(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;

  /* Write value to MEMS CTRL_REG1 register */
  ctrl = (uint8_t) InitStruct;
  GYRO_IO_Write(&ctrl, ICG20330_PWR_MGMT_1_ADDR, 1);
}

/**
  * @brief  Set ICG20330 Interrupt INT1 configuration
  * @param  Int1Config: the configuration setting for the ICG20330 Interrupt.
  * @retval None
  */
void ICG20330_INT1InterruptConfig(uint16_t Int1Config)
{
  uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;
  
  /* Read INT1_CFG register */
  GYRO_IO_Read(&ctrl_cfr, ICG20330_INT_PIN_CFG_ADDR, 1);
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&ctrl3, ICG20330_INT_ENABLE_ADDR, 1);
  
  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((uint8_t) Int1Config >> 8);
  
  ctrl3 &= 0xDF;
  ctrl3 |= ((uint8_t) Int1Config);   
  
  /* Write value to MEMS INT1_CFG register */
  GYRO_IO_Write(&ctrl_cfr, ICG20330_INT_PIN_CFG_ADDR, 1);
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&ctrl3, ICG20330_INT_ENABLE_ADDR, 1);
}

/**
  * @brief  Enable INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2 
  *      This parameter can be: 
  *        @arg ICG20330_INT1
  *        @arg ICG20330_INT2
  * @retval None
  */
void ICG20330_EnableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, ICG20330_INT_ENABLE_ADDR, 1);
  
//  if(IntSel == ICG20330_INT1)
//  {
//    tmpreg &= 0x7F;
//    tmpreg |= ICG20330_INT1INTERRUPT_ENABLE;
//  }
//  else if(IntSel == ICG20330_INT2)
//  {
//    tmpreg &= 0xF7;
//    tmpreg |= ICG20330_INT2INTERRUPT_ENABLE;
//  }
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&IntSel, ICG20330_INT_ENABLE_ADDR, 1);
}

/**
  * @brief  Disable  INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2 
  *      This parameter can be: 
  *        @arg ICG20330_INT1
  *        @arg ICG20330_INT2
  * @retval None
  */
void ICG20330_DisableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, ICG20330_INT_ENABLE_ADDR, 1);
  
//  if(IntSel == ICG20330_INT1)
//  {
//    tmpreg &= 0x7F;
//    tmpreg |= ICG20330_INT1INTERRUPT_DISABLE;
//  }
//  else if(IntSel == ICG20330_INT2)
//  {
//    tmpreg &= 0xF7;
//    tmpreg |= ICG20330_INT2INTERRUPT_DISABLE;
//  }
  tmpreg &=~ IntSel;
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&tmpreg, ICG20330_INT_ENABLE_ADDR, 1);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  FilterStruct: contains the configuration setting for the ICG20330.
  * @retval None
  */
void ICG20330_FilterConfig(uint8_t FilterStruct)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  GYRO_IO_Read(&tmpreg, ICG20330_CONFIG_ADDR, 1);
  
//  tmpreg &= 0xC0;
//
//  /* Configure MEMS: mode and cutoff frequency */
//  tmpreg |= FilterStruct;
  
  /* Write value to MEMS CTRL_REG2 register */
  GYRO_IO_Write(&FilterStruct, ICG20330_CONFIG_ADDR, 1);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: ICG20330_HIGHPASSFILTER_DISABLE
  *         @arg: ICG20330_HIGHPASSFILTER_ENABLE
  * @retval None
  */
void ICG20330_FilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  GYRO_IO_Read(&tmpreg, ICG20330_GYRO_CONFIG_ADDR, 1);
  
//  tmpreg &= 0xEF;
//
//  tmpreg |= HighPassFilterState;
  
  /* Write value to MEMS CTRL_REG5 register */
  GYRO_IO_Write(&HighPassFilterState, ICG20330_GYRO_CONFIG_ADDR, 1);
}

/**
  * @brief  Get status for ICG20330 data
  * @param  None         
  * @retval Data status in a ICG20330 Data
  */
uint8_t ICG20330_GetDataStatus(void)
{
  uint8_t tmpreg;
  
  /* Read STATUS_REG register */
  GYRO_IO_Read(&tmpreg, ICG20330_INT_STATUS_ADDR, 1);
  
  return tmpreg;
}

/**
* @brief  Calculate the ICG20330 angular data.
* @param  pfData: Data out pointer
* @retval None
*/
void ICG20330_ReadXYZAngRate(float *pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;
  
  GYRO_IO_Read(&tmpreg,ICG20330_GYRO_CONFIG_ADDR,1);
  
  GYRO_IO_Read(tmpbuffer,ICG20330_GYRO_XOUT_H_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
//  if(!(tmpreg & ICG20330_BLE_MSB))
//  {
//    for(i=0; i<3; i++)
//    {
//      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
//    }
//  }
//  else
//  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
//  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & ICG20330_FS_SEL)
  {
  case ICG20330_FULLSCALE_63: //+-31.25
    sensitivity=ICG20330_SENSITIVITY_1024;
    break;
    
  case ICG20330_FULLSCALE_125: //+-62.5
    sensitivity=ICG20330_SENSITIVITY_524;
    break;
    
  case ICG20330_FULLSCALE_250: //+-125
    sensitivity=ICG20330_SENSITIVITY_262;
    break;
  case ICG20330_FULLSCALE_500: //+-250
     sensitivity=ICG20330_SENSITIVITY_131;
     break;
  }
  /* Divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)(RawData[i]);
    pfData[i]/=(float)(sensitivity);
  }
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

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
