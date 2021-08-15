#include "usart.h"
//#include "tim.h"
#include "ModBus.h"
#include "ModBusPort.h"

unsigned char ReceiveBuffer[RECEIVE_BUFFER_SIZE];
unsigned char ReceiveCounter=0;
unsigned char TimerInitFlag =0;

unsigned char MasterReceiveBuffer[MASTER_RECEIVE_BUFFER_SIZE];
unsigned char MasterReceiveCounter=0;

UART_HandleTypeDef *huartModbusSlave;
UART_HandleTypeDef *huartModbusMaster;

TIM_HandleTypeDef *htimModbusSlave;
TIM_HandleTypeDef *htimModbusMaster;

void ModBusSlave_UART_Initialise(void)
{

//	if (huart != NULL) {
	//    General_USART_Init();
	huartModbusMaster = (UART_HandleTypeDef*) &huart3;
		huartModbusSlave = (UART_HandleTypeDef *) &huart3;
//	}
}

void ModBusSlave_TIMER_Initialise(void)
{
    if(TimerInitFlag==0)
    {
        TimerInitFlag =1;
//        htimModbusMaster = (TIM_HandleTypeDef *) &htim13;
//        htimModbusSlave = (TIM_HandleTypeDef *) &htim13;
//        General_Timer_Init();
    }
}

void ModBusSlave_UART_Putch(unsigned char c)
{
	//Wait for the uart to finish sending the byte.
	while(HAL_UART_STATE_BUSY_TX_RX == HAL_UART_GetState(huartModbusSlave));
//	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	HAL_UART_Transmit(huartModbusSlave, &c, 1, 500);
//	USART_SendData(USART2,c);
}

unsigned char ModBusSlave_UART_String(unsigned char *s, unsigned int Length)
{
  while(Length)
	{
		ModBusSlave_UART_Putch(*s++);
		Length--;
	}
	
	return TRUE;
}

/*******************************For Master*************************************/
void ModBusMaster_UART_Initialise(void)
{
    //InitUART2();
//	if (huart != NULL) {
		HAL_GPIO_WritePin(MB1_DE_GPIO_Port, MB1_DE_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(MB1_RE_GPIO_Port, MB1_RE_Pin, GPIO_PIN_RESET);

		huartModbusMaster= (UART_HandleTypeDef *) &huart3;

//	}
}

void ModBusMaster_TIMER_Initialise(void)
{
    if(TimerInitFlag==0)
    {
        TimerInitFlag =1;
//        InitTMR1();
    }
}

void ModBusMaster_UART_Putch(unsigned char c)
{
    //U2TXREG=c;
        //while(U2STAbits.UTXBF);   // Gonderim tamamlandi mi
	HAL_UART_Transmit_IT(huartModbusMaster, &c, 1);
}

unsigned char ModBusMaster_UART_String(unsigned char *s, unsigned int Length)
{
	HAL_StatusTypeDef ret = HAL_OK;
	HAL_UART_StateTypeDef uRet;

	HAL_GPIO_WritePin(MB1_RE_GPIO_Port, MB1_RE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MB1_DE_GPIO_Port, MB1_DE_Pin, GPIO_PIN_SET);

//    while(Length)
//    {
//        ModBusMaster_UART_Putch(*s++);
//        Length--;
//    }

	ret = HAL_UART_Transmit_DMA(huartModbusMaster, s, Length);
	if (ret == HAL_OK)
    {
		while(HAL_UART_STATE_BUSY_TX_RX == HAL_UART_GetState(huartModbusMaster));
//		//TODO: Start timeout timer
//		HAL_TIM_Base_Start(htimModbusMaster);
    }
    else
    {

    }

    return TRUE;
}

/***************************Interrupt For Slave********************************/
void MODBUS_SLAVE_RX_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(huartModbusSlave, UART_FLAG_RXNE))
	{
		ReceiveBuffer[ReceiveCounter]   = (unsigned char) huartModbusSlave->Instance->DR;
		ReceiveCounter++;

		if(ReceiveCounter>RECEIVE_BUFFER_SIZE)
			ReceiveCounter=0;

		SlaveTimerValue=0;
	}
}

void MODBUS_SLAVE_TX_IRQHandler(void)
{
	//TODO: Start timeout timer
	HAL_TIM_Base_Start(htimModbusSlave);
	HAL_UART_Receive_DMA(huartModbusSlave, &ReceiveBuffer[ReceiveCounter], 1);
}

/**************************Interrupts For Master********************************/
void MODBUS_MASTER_RX_IRQHandler(void)
//void __attribute__((interrupt, , auto_psv)) _U2RXInterrupt( void )
{
//    IFS1bits.U2RXIF = 0;                                                        // UART alim kesme bayragi temizleniyor
//		MasterReceiveBuffer[MasterReceiveCounter] = huartModbusMaster->Instance->DR;
		MasterReceiveCounter++;

		if(MasterReceiveCounter>MASTER_RECEIVE_BUFFER_SIZE)
			MasterReceiveCounter=0;

		MasterReadTimerValue=0;
		MasterWriteTimerValue=0;
		HAL_UART_Receive_DMA(huartModbusMaster, &MasterReceiveBuffer[MasterReceiveCounter], 1);
}

void MODBUS_MASTER_TX_IRQHandler(void)
{
	HAL_UART_Receive_DMA(huartModbusMaster, &MasterReceiveBuffer[MasterReceiveCounter], 1);
	HAL_GPIO_WritePin(MB1_RE_GPIO_Port, MB1_RE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MB1_DE_GPIO_Port, MB1_DE_Pin, GPIO_PIN_RESET);
}

/******************************************************************************/



