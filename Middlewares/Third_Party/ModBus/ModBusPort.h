	#ifndef __MODBUSPORT__H
#define __MODBUSPORT__H

#include "ModBus.h"

extern unsigned char ReceiveBuffer[RECEIVE_BUFFER_SIZE];
extern unsigned char ReceiveCounter;

extern unsigned char MasterReceiveBuffer[MASTER_RECEIVE_BUFFER_SIZE];
extern unsigned char MasterReceiveCounter;

extern void ModBusSlave_UART_Initialise(void);
extern void ModBusSlave_TIMER_Initialise(void);
extern void ModBusSlave_UART_Putch(unsigned char c);
extern unsigned char ModBusSlave_UART_String(unsigned char *s, unsigned int Length);

extern void ModBusMaster_UART_Initialise(void);
extern void ModBusMaster_TIMER_Initialise(void);
extern void ModBusMaster_UART_Putch(unsigned char c);
extern unsigned char ModBusMaster_UART_String(unsigned char *s, unsigned int Length);

void MODBUS_SLAVE_RX_IRQHandler(void);
void MODBUS_MASTER_RX_IRQHandler(void);

void MODBUS_SLAVE_TX_IRQHandler(void);
void MODBUS_MASTER_TX_IRQHandler(void);

void MODBUS_SLAVE_TIMER_IRQHandler(void);
void MODBUS_MASTER_TIMER_IRQHandler(void);

#endif
