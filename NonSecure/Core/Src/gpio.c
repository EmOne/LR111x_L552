/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PB3 (JTDO/TRACESWO)   ------> DEBUG_JTDO-SWO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LR_NSS_Pin|LR_NRST_Pin, GPIO_PIN_SET);

  /*Configure the EXTI line attribute */
  HAL_EXTI_ConfigLineAttributes(EXTI_LINE_2, EXTI_LINE_PRIV);

  /*Configure the EXTI line attribute */
  HAL_EXTI_ConfigLineAttributes(EXTI_LINE_3, EXTI_LINE_PRIV);

  /*Configure the EXTI line attribute */
  HAL_EXTI_ConfigLineAttributes(EXTI_LINE_2, EXTI_LINE_PRIV);

  /*Configure the EXTI line attribute */
  HAL_EXTI_ConfigLineAttributes(EXTI_LINE_14, EXTI_LINE_PRIV);

  /*Configure the EXTI line attribute */
  HAL_EXTI_ConfigLineAttributes(EXTI_LINE_10, EXTI_LINE_PRIV);

  /*Configure GPIO pins : PEPin PEPin */
  GPIO_InitStruct.Pin = GYO_INT1_Pin|GYO_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = LR_RFSW2_Pin|LR_RFSW3_Pin|LR_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LR_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LR_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LR_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LR_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LR_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LR_NRST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI10_IRQn);

  HAL_NVIC_SetPriority(EXTI14_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI14_IRQn);

}

/* USER CODE BEGIN 2 */
static Gpio_t *GpioIrq[16];

void GpioInit( Gpio_t *obj, void* port, uint16_t pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
//    if( pin < IOE_0 )
//    {
        GPIO_InitTypeDef GPIO_InitStructure;

        obj->pin = pin;

        if( pin == NC  || port == NULL)
        {
            return;
        }

        obj->pinIndex = pin; //( 0x01 << ( obj->pin & 0x0F ) );
        obj->port = port;
        if( ( obj->port ) == GPIOA )
        {
            obj->port = GPIOA;
            __HAL_RCC_GPIOA_CLK_ENABLE( );
        }
        else if( ( obj->port ) == GPIOB )
        {
            obj->port = GPIOB;
            __HAL_RCC_GPIOB_CLK_ENABLE( );
        }
        else if( ( obj->port ) == GPIOC )
        {
            obj->port = GPIOC;
            __HAL_RCC_GPIOC_CLK_ENABLE( );
        }
        else if( ( obj->port ) == GPIOD )
        {
            obj->port = GPIOD;
            __HAL_RCC_GPIOD_CLK_ENABLE( );
        }
        else
        {
            obj->port = GPIOH;
            __HAL_RCC_GPIOH_CLK_ENABLE( );
        }

        GPIO_InitStructure.Pin =  obj->pinIndex ;
        GPIO_InitStructure.Pull = obj->pull = type;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

        if( mode == PIN_INPUT )
        {
            GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
        }
        else if( mode == PIN_ANALOGIC )
        {
            GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
        }
        else if( mode == PIN_ALTERNATE_FCT )
        {
            if( config == PIN_OPEN_DRAIN )
            {
                GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
            }
            else
            {
                GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
            }
            GPIO_InitStructure.Alternate = value;
        }
        else // mode output
        {
            if( config == PIN_OPEN_DRAIN )
            {
                GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
            }
            else
            {
                GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            }
        }

        // Sets initial output value
        if( mode == PIN_OUTPUT )
        {
            GpioWrite( obj, value );
        }

        HAL_GPIO_Init( obj->port, &GPIO_InitStructure );
//    }
//    else
//    {
//#if defined( BOARD_IOE_EXT )
//        // IOExt Pin
//        GpioIoeInit( obj, pin, mode, config, type, value );
//#endif
//    }
}

void GpioSetContext( Gpio_t *obj, void* context )
{
    obj->Context = context;
}

void GpioSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
//    if( obj->pin < IOE_0 )
//    {
        uint32_t priority = 0;

        IRQn_Type IRQnb = EXTI0_IRQn;
        GPIO_InitTypeDef   GPIO_InitStructure;

        if( irqHandler == NULL )
        {
            return;
        }

        obj->IrqHandler = irqHandler;

        GPIO_InitStructure.Pin =  obj->pinIndex;

        if( irqMode == IRQ_RISING_EDGE )
        {
            GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
        }
        else if( irqMode == IRQ_FALLING_EDGE )
        {
            GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
        }
        else
        {
            GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
        }

        GPIO_InitStructure.Pull = obj->pull;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

        HAL_GPIO_Init( obj->port, &GPIO_InitStructure );

        switch( irqPriority )
        {
        case IRQ_VERY_LOW_PRIORITY:
        case IRQ_LOW_PRIORITY:
            priority = 3;
            break;
        case IRQ_MEDIUM_PRIORITY:
            priority = 2;
            break;
        case IRQ_HIGH_PRIORITY:
            priority = 1;
            break;
        case IRQ_VERY_HIGH_PRIORITY:
        default:
            priority = 0;
            break;
        }

        switch( obj->pinIndex )
        {
        case GPIO_PIN_0:
        	GpioIrq[0] = obj;
            IRQnb = EXTI0_IRQn;
            break;
        case GPIO_PIN_1:
        	GpioIrq[1] = obj;
            IRQnb = EXTI1_IRQn;
            break;
        case GPIO_PIN_2:
        	GpioIrq[2] = obj;
            IRQnb = EXTI2_IRQn;
            break;
        case GPIO_PIN_3:
        	GpioIrq[3] = obj;
            IRQnb = EXTI3_IRQn;
            break;
        case GPIO_PIN_4:
        	GpioIrq[4] = obj;
            IRQnb = EXTI4_IRQn;
            break;
        case GPIO_PIN_5:
        	GpioIrq[5] = obj;
            IRQnb = EXTI5_IRQn;
            break;
        case GPIO_PIN_6:
        	GpioIrq[6] = obj;
            IRQnb = EXTI6_IRQn;
            break;
        case GPIO_PIN_7:
        	GpioIrq[7] = obj;
            IRQnb = EXTI7_IRQn;
            break;
        case GPIO_PIN_8:
        	GpioIrq[8] = obj;
            IRQnb = EXTI8_IRQn;
            break;
        case GPIO_PIN_9:
        	GpioIrq[9] = obj;
            IRQnb = EXTI9_IRQn;
            break;
        case GPIO_PIN_10:
        	GpioIrq[10] = obj;
            IRQnb = EXTI10_IRQn;
            break;
        case GPIO_PIN_11:
        	GpioIrq[11] = obj;
            IRQnb = EXTI11_IRQn;
            break;
        case GPIO_PIN_12:
        	GpioIrq[12] = obj;
            IRQnb = EXTI12_IRQn;
            break;
        case GPIO_PIN_13:
        	GpioIrq[13] = obj;
            IRQnb = EXTI13_IRQn;
            break;
        case GPIO_PIN_14:
        	GpioIrq[14] = obj;
            IRQnb = EXTI14_IRQn;
            break;
        case GPIO_PIN_15:
        	GpioIrq[15] = obj;
            IRQnb = EXTI15_IRQn;
            break;
        default:
            break;
        }

//        GpioIrq[( obj->pin ) & 0x0F] = obj;

        HAL_NVIC_SetPriority( IRQnb , priority, 0 );
        HAL_NVIC_EnableIRQ( IRQnb );
//    }
//    else
//    {
//#if defined( BOARD_IOE_EXT )
//        // IOExt Pin
//        GpioIoeSetInterrupt( obj, irqMode, irqPriority, irqHandler );
//#endif
//    }
}

void GpioRemoveInterrupt( Gpio_t *obj )
{
//    if( obj->pin < IOE_0 )
//    {
        // Clear callback before changing pin mode
        GpioIrq[( obj->pin ) & 0x0F] = NULL;

        GPIO_InitTypeDef   GPIO_InitStructure;

        GPIO_InitStructure.Pin =  obj->pinIndex ;
        GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init( obj->port, &GPIO_InitStructure );
//    }
//    else
//    {
//#if defined( BOARD_IOE_EXT )
//        // IOExt Pin
//        GpioIoeRemoveInterrupt( obj );
//#endif
//    }
}

void GpioWrite( Gpio_t *obj, uint32_t value )
{
//    if( obj->pin < IOE_0 )
//    {
        if( obj == NULL )
        {
            assert_param( FAIL );
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        HAL_GPIO_WritePin( obj->port, obj->pinIndex , ( GPIO_PinState )value );
//    }
//    else
//    {
//#if defined( BOARD_IOE_EXT )
//        // IOExt Pin
//        GpioIoeWrite( obj, value );
//#endif
//    }
}

void GpioToggle( Gpio_t *obj )
{
//    if( obj->pin < IOE_0 )
//    {
        if( obj == NULL )
        {
            assert_param( FAIL );
        }

        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        HAL_GPIO_TogglePin( obj->port, obj->pinIndex );
//    }
//    else
//    {
//#if defined( BOARD_IOE_EXT )
//        // IOExt Pin
//        GpioIoeToggle( obj );
//#endif
//    }
}

uint32_t GpioRead( Gpio_t *obj )
{
//    if( obj->pin < IOE_0 )
//    {
        if( obj == NULL )
        {
            assert_param( FAIL );
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return 0;
        }
        return HAL_GPIO_ReadPin( obj->port, obj->pinIndex );
//    }
//    else
//    {
//#if defined( BOARD_IOE_EXT )
//        // IOExt Pin
//        return GpioIoeRead( obj );
//#else
//        return 0;
//#endif
//    }
}

void EXTI0_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_0 );
}

void EXTI1_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_1 );
}

void EXTI2_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_2 );
}

void EXTI3_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_3 );
}

void EXTI4_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_4 );
}

void EXTI5_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_5 );
}

void EXTI6_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
}

void EXTI7_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_7 );
}

void EXTI8_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );
}

void EXTI9_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_9 );
}

void EXTI10_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );
}

void EXTI11_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_11 );
}

void EXTI12_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_12 );
}

void EXTI13_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );
}

void EXTI14_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_14 );
}

void EXTI15_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_15 );
}

void HAL_GPIO_EXTI_Callback( uint16_t gpioPin )
{
    uint8_t callbackIndex = 0;

    if( gpioPin > 0 )
    {
        while( gpioPin != 0x01 )
        {
            gpioPin = gpioPin >> 1;
            callbackIndex++;
        }
    }

    if( ( GpioIrq[callbackIndex] != NULL ) && ( GpioIrq[callbackIndex]->IrqHandler != NULL ) )
    {
        GpioIrq[callbackIndex]->IrqHandler( GpioIrq[callbackIndex]->Context );
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_EXTI_Callback(GPIO_Pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_EXTI_Callback(GPIO_Pin);
}
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
