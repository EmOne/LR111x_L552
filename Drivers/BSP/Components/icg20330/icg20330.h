/**
  ******************************************************************************
  * @file    icg20330.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    26-June-2015
  * @brief   This file contains all the functions prototypes for the icg20330.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 EmOne</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of EmOne nor the names of its contributors
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
  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICG20330_H
#define __ICG20330_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../Common/gyro.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 

/** @addtogroup ICG20330
  * @{
  */
  
/** @defgroup ICG20330_Exported_Constants
  * @{
  */

/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define ICG20330_SELF_TEST_X_GYRO_ADDR          0x00  /* device identification register */
#define ICG20330_SELF_TEST_Y_GYRO_ADDR         	0x01  /* Control register 1 */
#define ICG20330_SELF_TEST_Z_GYRO_ADDR         	0x02  /* Control register 2 */
#define ICG20330_XG_OFFS_TC_H_ADDR         		0x04  /* Control register 3 */
#define ICG20330_XG_OFFS_TC_L_ADDR         		0x05  /* Control register 4 */
#define ICG20330_YG_OFFS_TC_H_ADDR         		0x07  /* Control register 5 */
#define ICG20330_YG_OFFS_TC_L_ADDR     			0x08  /* Reference register */
#define ICG20330_ZG_OFFS_TC_H_ADDR          	0x0A  /* Out temp register */
#define ICG20330_ZG_OFFS_TC_L_ADDR        		0x0B  /* Status register */
#define ICG20330_XG_OFFS_USRH_ADDR           	0x13  /* Output Register X */
#define ICG20330_XG_OFFS_USRL_ADDR           	0x14  /* Output Register X */
#define ICG20330_YG_OFFS_USRH_ADDR           	0x15  /* Output Register Y */
#define ICG20330_YG_OFFS_USRL_ADDR           	0x16  /* Output Register Y */
#define ICG20330_ZG_OFFS_USRH_ADDR           	0x17  /* Output Register Z */
#define ICG20330_ZG_OFFS_USRL_ADDR           	0x18  /* Output Register Z */
#define ICG20330_SMPLRT_DIV_ADDR     			0x19  /* Fifo control Register */
#define ICG20330_CONFIG_ADDR      				0x1A  /* Fifo src Register */
#define ICG20330_GYRO_CONFIG_ADDR          		0x1B  /* GYROSCOPE CONFIGURATION */
#define ICG20330_FIFO_EN_ADDR       	   		0x23  /* FIFO ENABLE */
#define ICG20330_FSYNC_INT_ADDR       			0x36  /* FSYNC INTERRUPT STATUS */
#define ICG20330_INT_PIN_CFG_ADDR      	 		0x37  /* INT PIN / BYPASS ENABLE CONFIGURATION */
#define ICG20330_INT_ENABLE_ADDR       			0x38  /* INTERRUPT ENABLE */
#define ICG20330_INT_STATUS_ADDR       			0x3A  /* INTERRUPT STATUS */
#define ICG20330_TEMP_OUT_H_ADDR       			0x41  /* TEMPERATURE MEASUREMENT */
#define ICG20330_TEMP_OUT_L_ADDR       			0x42  /* TEMPERATURE MEASUREMENT */
#define ICG20330_GYRO_XOUT_H_ADDR     			0x43  /* GYROSCOPE MEASUREMENTS */
#define ICG20330_GYRO_XOUT_L_ADDR     			0x44  /* GYROSCOPE MEASUREMENTS */
#define ICG20330_GYRO_YOUT_H_ADDR     			0x45  /* GYROSCOPE MEASUREMENTS */
#define ICG20330_GYRO_YOUT_L_ADDR     			0x46  /* GYROSCOPE MEASUREMENTS */
#define ICG20330_GYRO_ZOUT_H_ADDR     			0x47  /* GYROSCOPE MEASUREMENTS */
#define ICG20330_GYRO_ZOUT_L_ADDR     			0x48  /* GYROSCOPE MEASUREMENTS */
#define ICG20330_SIGNAL_PATH_RESET_ADDR     	0x68  /* SIGNAL PATH RESET */
#define ICG20330_USER_CTRL_ADDR     			0x6A  /* USER CONTROL */
#define ICG20330_PWR_MGMT_1_ADDR     			0x6B  /* POWER MANAGEMENT 1 */
#define ICG20330_PWR_MGMT_2_ADDR     			0x6C  /* POWER MANAGEMENT 2 */
#define ICG20330_FIFO_COUNTH_ADDR     			0x72  /* FIFO COUNT REGISTERS */
#define ICG20330_FIFO_COUNTL_ADDR     			0x73  /* FIFO COUNT REGISTERS */
#define ICG20330_FIFO_R_W_ADDR     				0x74  /* FIFO READ WRITE */
#define ICG20330_WHO_AM_I_ADDR     				0x75  /* WHO AM I */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/
#define ICG20330_ADDR					((uint8_t)0x69)
#define I_AM_ICG20330                 	((uint8_t)0x92)

/** @defgroup POWER MANAGEMENT 2
   * @{
   */
 #define ICG20330_STBY_XG_DISABLE       ((uint8_t)0x04)
 #define ICG20330_STBY_YG_DISABLE       ((uint8_t)0x02)
 #define ICG20330_STBY_ZG_DISABLE       ((uint8_t)0x01)

 typedef enum {
	 ICG20330_X_AXE_ENABLE,
	 ICG20330_Y_AXE_ENABLE,
	 ICG20330_Z_AXE_ENABLE,
	 ICG20330_AXES_ENABLE
 } ePowerMgt2_t;
 /**
   * @}
   */


/** @defgroup POWER MANAGEMENT 1
   * @{
   */
#define ICG20330_DEV_RESET       		((uint8_t)0x80)
#define ICG20330_SLEEP       			((uint8_t)0x40)
#define ICG20330_GYRO_STANDBY       	((uint8_t)0x10)
#define ICG20330_TEMP_DIS       		((uint8_t)0x08)
#define ICG20330_CLKSEL		      		((uint8_t)0x07)

typedef enum
{
	ICG20330_MODE_POWERDOWN,
	ICG20330_MODE_STANDBY,
	ICG20330_MODE_SLEEP,
	ICG20330_MODE_ACTIVE
} ePowerMode_t;
/**
* @}
*/

/** @defgroup USER CONTROL
* @{
*/
#define ICG20330_FIFO_EN       			((uint8_t)0x40)
#define ICG20330_I2C_IF_DIS		       	((uint8_t)0x10)
#define ICG20330_FIFO_RST       		((uint8_t)0x04)
#define ICG20330_SIG_COND_RST      		((uint8_t)0x01)
/**
* @}
*/

 /** @defgroup SIGNAL PATH RESET
 * @{
 */
 #define ICG20330_TEMP_RST       		((uint8_t)0x01)
 /**
 * @}
 */

/** @defgroup INTERRUPT ENABLE
  * @{
  */
#define ICG20330_WOM_EN      ((uint8_t)0xE0)
#define ICG20330_FIFO_OFLOW_EN       		((uint8_t)0x10)
#define ICG20330_GDRIVE_INT_EN       	((uint8_t)0x04)
#define ICG20330_DATA_RDY_INT_EN       	((uint8_t)0x01)
  /**
  * @}
  */

 /** @defgroup INTERRUPT STATUS
  * @{
  */
#define ICG20330_FIFO_OVERFLOW_INT      ((uint8_t)0x10)
#define ICG20330_GDRIVE_INT       		((uint8_t)0x40)
#define ICG20330_DATA_RDY_INT       	((uint8_t)0x01)
  /**
  * @}
  */

 /** @defgroup INT PIN / BYPASS ENABLE CONFIGURATION
   * @{
   */
#define ICG20330_INT_LVL       			((uint8_t)0x80)
#define ICG20330_INT_OPEN       		((uint8_t)0x40)
#define ICG20330_LATCH_INT_EN       	((uint8_t)0x20)
#define ICG20330_INT_RD_CLR       		((uint8_t)0x10)
#define ICG20330_FSYNC_INT_LVL       	((uint8_t)0x08)
#define ICG20330_FSYNC_INT_MODE_EN      ((uint8_t)0x04)
   /**
   * @}
   */

/** @defgroup FSYNC INTERRUPT STATUS
   * @{
   */
#define ICG20330_FSYNC_INT      	 	((uint8_t)0x80)
   /**
   * @}
   */

 /** @defgroup FIFO ENABLE
   * @{
   */
#define ICG20330_TEMP_FIFO_EN   		((uint8_t)0x80)
#define ICG20330_XG_FIFO_EN       		((uint8_t)0x40)
#define ICG20330_YG_FIFO_EN 	      	((uint8_t)0x20)
#define ICG20330_ZG_FIFO_EN       		((uint8_t)0x10)
   /**
   * @}
   */

/** @defgroup GYROSCOPE CONFIGURATION
* @{
*/
#define ICG20330_XG_ST  		 		((uint8_t)0x80)
#define ICG20330_YG_ST       			((uint8_t)0x40)
#define ICG20330_ZG_ST 	     	 		((uint8_t)0x20)
#define ICG20330_FS_SEL       			((uint8_t)0x18)
#define ICG20330_FCHOICE       			((uint8_t)0x03)

typedef enum {
	ICG20330_HPM_NORMAL_MODE_RES,
	ICG20330_HPM_HIGH_MODE_RES
} eFCHOICE_t;

typedef enum {
	ICG20330_HPFCF_0,

} eFilter_t;

/**
* @}
*/

/** @defgroup CONFIGURATION
 * @{
 */
#define ICG20330_FIFO_MODE  	 		((uint8_t)0x40)
#define ICG20330_EXT_SYNC_SET   		((uint8_t)0x38)
#define ICG20330_DLPF_CFG      			((uint8_t)0x07)

typedef enum {
	ICG20330_BlockDataUpdate_Continous,
	ICG20330_BlockDataUpdate_Single,
} eFIFO_MODE_t;


/**
 * @}
 */

/** @defgroup Sample rate divider
 * @{
 */
typedef enum {
	ICG20330_OUTPUT_DATARATE_1,
	ICG20330_OUTPUT_DATARATE_8,
	ICG20330_OUTPUT_DATARATE_32,
} eDataRate_t;

typedef enum {
	ICG20330_BANDWIDTH_8K,
	ICG20330_BANDWIDTH_3K,
	ICG20330_BANDWIDTH_250,
	ICG20330_BANDWIDTH_176,
	ICG20330_BANDWIDTH_92,

} eBandWidth_t;

typedef enum {
	ICG20330_BLE_LSB,
	ICG20330_BLE_MSB
} eEndianness_t;

typedef enum {
	ICG20330_FULLSCALE_63 = 0x00,
	ICG20330_FULLSCALE_125 = 0x08,
	ICG20330_FULLSCALE_250 = 0x10,
	ICG20330_FULLSCALE_500 = 0x18,
} eScale_t;

#define ICG20330_SENSITIVITY_1024 1024
#define ICG20330_SENSITIVITY_524 524
#define ICG20330_SENSITIVITY_262 262
#define ICG20330_SENSITIVITY_131 131
 /**
 * @}
 */

/**
  * @}
  */
/** @defgroup ICG20330_Exported_Functions
  * @{
  */
/* Sensor Configuration Functions */ 
void    ICG20330_Init(void* InitStruct);
void    ICG20330_DeInit(void);
void    ICG20330_LowPower(uint16_t InitStruct);
uint8_t ICG20330_ReadID(void);
void    ICG20330_RebootCmd(void);

/* Interrupt Configuration Functions */
void    ICG20330_INT1InterruptConfig(uint16_t Int1Config);
void    ICG20330_EnableIT(uint8_t IntSel);
void    ICG20330_DisableIT(uint8_t IntSel);

/* High Pass Filter Configuration Functions */
void    ICG20330_FilterConfig(uint8_t FilterStruct);
void    ICG20330_FilterCmd(uint8_t HighPassFilterState);
void    ICG20330_ReadXYZAngRate(float *pfData);
uint8_t ICG20330_GetDataStatus(void);

/* Gyroscope IO functions */
void    GYRO_IO_Init(void);
void    GYRO_IO_DeInit(void);
void    GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void    GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

/* Gyroscope driver structure */
extern GYRO_DrvTypeDef icg20330Drv;

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

#ifdef __cplusplus
  }
#endif
  
#endif /* __ICG20330_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
