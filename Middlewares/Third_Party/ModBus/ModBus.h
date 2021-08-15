/*  Modbus Version 1.0
 *  Author: Firat DEVECI
 *  
 *  Ipucu   :
 *  RS485   :	Bu iletisim metodu kullanilirken RX pini pull up yapilmalidir.
 *  Maintain by Anol P. <anol.p@emone.co.th> 2020
 */

#ifndef __MODBUS__H
#define __MODBUS__H

#define NUMBER_OF_COILS                         0                               // Modbus RTU Slave Coil Sayisi             :   Kullanilacak Coil sayisi buradan girilmeli
#define NUMBER_OF_INPUTS                        0                               // Modbus RTU Slave Input Sayisi            :   Kullanilacak Input sayisi buradan girilmeli
#define NUMBER_OF_OUTPUT_REGISTERS              8                             // Modbus RTU Slave Output Register Sayisi  :   Kullanilacak Register sayisi buradan girilmeli
#define NUMBER_OF_INPUT_REGISTERS               0                               // Modbus RTU Slave Input Register Sayisi   :   Kullanilacak Input Register sayisi buradan girilmeli
#define NUMBER_MASTER_INPUT_REGISTERS           16                               // Modbus RTU Master Input Register Sayisi  :   Kullanilacak Master Data Register sayisi buradan girilmeli
#define NUMBER_MASTER_LOOKUP_INPUTS           	5                              // Modbus RTU Master Lookup Input Register  :   Define look up table fot specific registers
#define NUMBER_MASTER_LOOKUP_SLAVE           	1

#define RECEIVE_BUFFER_SIZE                     250                             // Modbus RTU Slave icin kullanilacak buffer boyutu
#define TRANSMIT_BUFFER_SIZE                    RECEIVE_BUFFER_SIZE
#define RXTX_BUFFER_SIZE                        TRANSMIT_BUFFER_SIZE
#define MASTER_RECEIVE_BUFFER_SIZE              RXTX_BUFFER_SIZE                // Modbus RTU Master icin de slave sayisi kullaniliyor

#define TIMEOUTTIMER                            1000                            // Modbus RTU Slave icin timeout suresi [milisaniye]
extern unsigned char SLAVE_ADDRESS;                                             // Modbus RTU Slave icin adres numarasi [0 to 255]

#define MBFN_READ_COILS_ENABLED                 ( 0 )                           // Kullanilacaksa 1, kullanilmayacaksa 0
#define MBFN_READ_DISCRETE_INPUTS_ENABLED       ( 0 )                           // Kullanilacaksa 1, kullanilmayacaksa 0
#define MBFN_READ_HOLDING_REGISTERS_ENABLED     ( 1 )                           // Kullanilacaksa 1, kullanilmayacaksa 0
#define MBFN_READ_INPUT_REGISTERS_ENABLED       ( 0 )                           // Kullanilacaksa 1, kullanilmayacaksa 0
#define MBFN_WRITE_SINGLE_COIL_ENABLED          ( 0 )                           // Kullanilacaksa 1, kullanilmayacaksa 0
#define MBFN_WRITE_SINGLE_REGISTER_ENABLED      ( 1 )                           // Kullanilacaksa 1, kullanilmayacaksa 0
#define MBFN_WRITE_MULTIPLE_COILS_ENABLED       ( 0 )                           // Kullanilacaksa 1, kullanilmayacaksa 0
#define MBFN_WRITE_MULTIPLE_REGISTERS_ENABLED   ( 0 )                           // Kullanilacaksa 1, kullanilmayacaksa 0
#define MBFN_MASTER_REGISTERS_ENABLED           ( 1 )                           // Modbus RTU Master kullanilacaksa 1, kullanilmayacaksa 0

typedef struct{
            int                     ActValue;
            const unsigned int      MaxValue;
            const unsigned int      Default;
            const unsigned int      MinValue;
            unsigned                RW  :1;
        }RegStructure;

#define NUMBER_INPUT_BYTES                      ((NUMBER_OF_INPUTS/8)+1)
#define NUMBER_COIL_BYTES                       ((NUMBER_OF_COILS/8)+1)
#define NUMBER_INPUT_BYTES                      ((NUMBER_OF_INPUTS/8)+1)
#define NUMBER_COIL_BYTES                       ((NUMBER_OF_COILS/8)+1)

#if MBFN_READ_COILS_ENABLED > 0
extern unsigned char ReadCoilOutputs            [NUMBER_COIL_BYTES];
#endif
#if ((MBFN_WRITE_SINGLE_COIL_ENABLED > 0)||(MBFN_WRITE_MULTIPLE_COILS_ENABLED > 0))
extern unsigned char WriteCoilOutputs           [NUMBER_COIL_BYTES];
#endif
#if MBFN_READ_DISCRETE_INPUTS_ENABLED > 0
extern unsigned char DiscreteInputs             [NUMBER_OF_INPUTS];
#endif
#if ((MBFN_WRITE_MULTIPLE_REGISTERS_ENABLED > 0)|| (MBFN_WRITE_SINGLE_REGISTER_ENABLED > 0) || (MBFN_READ_HOLDING_REGISTERS_ENABLED > 0))
extern RegStructure  RegisterOutputs            [NUMBER_OF_OUTPUT_REGISTERS];
#endif
#if MBFN_READ_INPUT_REGISTERS_ENABLED > 0
extern RegStructure  RegisterInputs             [NUMBER_OF_INPUT_REGISTERS];
#endif

#if MBFN_MASTER_REGISTERS_ENABLED > 0
extern RegStructure  MasterRegisterInputs       [NUMBER_MASTER_INPUT_REGISTERS];

typedef struct
{
            unsigned int  LookupAddress;
            unsigned int  Size;
            RegStructure  		RegisterInput[NUMBER_MASTER_INPUT_REGISTERS];
} LookupTable;

extern LookupTable	  MasterLookupTableInputs[NUMBER_MASTER_LOOKUP_INPUTS];
#endif

extern unsigned int SlaveTimerValue;
extern unsigned int MasterReadTimerValue;
extern unsigned int MasterWriteTimerValue;

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

extern void InitModbusSlave(unsigned char SlaveAddress);
extern unsigned char SendMessage(void);
extern unsigned char RxDataAvailable(void);
extern void ProcessModbusSlave(void);
extern void HandleModbusReadCoils(void);                                        // ModBus 01
extern void HandleModbusReadDiscreteInputs(void);                               // ModBus 02
extern void HandleModbusReadHoldingRegisters(void);                             // ModBus 03
extern void HandleModbusReadInputRegisters(void);                               // ModBus 04
extern void HandleModbusWriteSingleCoil(void);                                  // ModBus 05
extern void HandleModbusWriteSingleRegister(void);                              // ModBus 06
extern void HandleModbusWriteMultipleCoils(void);                               // ModBus 15
extern void HandleModbusWriteMultipleRegisters(void);                           // ModBus 16
extern unsigned char CheckSlaveBufferComplete(void);

extern void InitModbusMaster(void);
extern unsigned char CheckMasterBufferComplete(unsigned char SlaveNumber);
extern void HandleModbusMasterReadHoldingRegisters(void);
extern unsigned char ModBusMasterRead(unsigned char SlaveNumber, unsigned char Function, unsigned int StartAddress, unsigned int NumberOfRegisters, unsigned int TimeOut);
extern unsigned char ModBusMasterWrite(unsigned char SlaveNumber, unsigned char Function, unsigned int StartAddress, unsigned char NumberOfData, unsigned int *Data,  unsigned int TimeOut);

#endif
