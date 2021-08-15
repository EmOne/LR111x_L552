#include "ModBus.h"
#include "ModBusPort.h"

/*******************************ModBus Functions*******************************/
#define MBFN_READ_COILS                 1
#define MBFN_READ_DISCRETE_INPUTS       2
#define MBFN_READ_HOLDING_REGISTERS     3
#define MBFN_READ_INPUT_REGISTERS       4
#define MBFN_WRITE_SINGLE_COIL          5
#define MBFN_WRITE_SINGLE_REGISTER      6
#define MBFN_WRITE_MULTIPLE_COILS       15
#define MBFN_WRITE_MULTIPLE_REGISTERS   16
/****************************End of ModBus Functions***************************/

#define FALSE_FUNCTION                  0
#define FALSE_SLAVE_ADDRESS             1
#define DATA_NOT_READY                  2
#define DATA_READY                      3

#define ERROR_CODE_01                   0x01                                    // Function code is not supported
#define ERROR_CODE_02                   0x02                                    // Register address is not allowed or write-protected
#define ERROR_CODE_03                   0x03                                    // Some data values are out of range, invalid number of register
#define ERROR_CODE_06                   0x06                                    // Device can not handle the request at the moment. Repeat the request.
#define ERROR_CODE_0B                   0x0B                                    // Error message of the interconnected gateway: No response of the accessed device.

unsigned char SLAVE_ADDRESS             =1;

typedef enum
{
    RXTX_IDLE,
    RXTX_START,
    RXTX_DATABUF,
    RXTX_WAIT_ANSWER,
    RXTX_TIMEOUT
}RXTX_STATE;

typedef struct
{
  unsigned char     Address;
  unsigned char     Function;
  unsigned char     DataBuf[RXTX_BUFFER_SIZE];
  unsigned int      DataLen;
}RXTX_DATA;

/**********************Slave Transmit and Receive Variables********************/
RXTX_DATA           Tx_Data;
unsigned int        Tx_Current              = 0;
unsigned int        Tx_CRC16                = 0xFFFF;
RXTX_STATE          Tx_State                = RXTX_IDLE;
unsigned char       Tx_Buf[TRANSMIT_BUFFER_SIZE];
unsigned int        Tx_Buf_Size             = 0;

RXTX_DATA           Rx_Data;
unsigned int        Rx_CRC16                = 0xFFFF;
RXTX_STATE          Rx_State                = RXTX_IDLE;
unsigned char       Rx_Data_Available       = FALSE;

unsigned int        SlaveTimerValue         = 0;
/****************End of Slave Transmit and Receive Variables*******************/

/**********************Master Transmit and Receive Variables********************/
RXTX_DATA           MasterRead_Data;
RXTX_DATA           MasterWrite_Data;
RXTX_STATE          MasterRead_State        = RXTX_IDLE;
RXTX_STATE          MasterWrite_State       = RXTX_IDLE;
unsigned char       MasterTx_Buf[TRANSMIT_BUFFER_SIZE];
unsigned int        MasterTx_Buf_Size       = 0;

unsigned int        MasterRead_CRC16        = 0xFFFF;
unsigned int        MasterWrite_CRC16       = 0xFFFF;

unsigned int        MasterReadTimerValue    = 0;
unsigned int        MasterWriteTimerValue   = 0;
/****************End of Slave Transmit and Receive Variables*******************/

/*
 * Function Name        : CRC16
 * @param[in]           : Data  - CRC'si hesaplanacak data
 * @param[in/out]       : CRC   - Anlik CRC degeri
 * @How to use          : CRC'si hesaplanacak dizinin degerleri sirasiyla bu
 *                        fonksiyona girmeli, sonuc olarak her seferde CRC
 *                        geri dondurulecek. Baslangic 0xFFFF ile baslamali.
 */
void CRC16(const unsigned char Data, unsigned int* CRC)
{
    unsigned int i;

    *CRC = *CRC ^(unsigned int) Data;
    for (i = 8; i > 0; i--)
    {
        if (*CRC & 0x0001)
            *CRC = (*CRC >> 1) ^ 0xA001;
        else
            *CRC >>= 1;
    }
}

/******************************************************************************/

/*
 * Function Name        : DoTx
 * @param[out]          : TRUE
 * @How to use          : Gonderilecek dizi bu fonksiyon ile gonderilir.
 */
unsigned char DoSlaveTX(void)
{  
    ModBusSlave_UART_String(Tx_Buf,Tx_Buf_Size);

    Tx_Buf_Size = 0;
    return TRUE;
}

/******************************************************************************/

/*
 * Function Name        : SendMessage
 * @param[out]          : TRUE/FALSE
 * @How to use          : Hazirlanan mesaji gonder komutu bu fonksiyon ile verilir.
 */
unsigned char SendMessage(void)
{
    if (Tx_State != RXTX_IDLE)
        return FALSE;

    Tx_Current  =0;
    Tx_State    =RXTX_START;

    return TRUE;
}

/******************************************************************************/

/*
 * Function Name        : HandleModbusError
 * @How to use          : ModBus master'a hata mesaji gonderilir.
 */
void HandleModbusError(char ErrorCode)
{
    // Initialise the output buffer. The first byte in the buffer says how many registers we have read
    Tx_Data.Function    = ErrorCode | 0x80;
    Tx_Data.Address     = SLAVE_ADDRESS;
    Tx_Data.DataLen     = 0;
    SendMessage();
}

/******************************************************************************/

/*
 * Function Name        : HandleModbusReadCoils
 * @How to use          : Modbus function 01 - Read coil status
 */
#if MBFN_READ_COILS_ENABLED > 0
void HandleModbusReadCoils(void)
{
    // We have two digital outputs - Spindle FWD and REV. These are on port A, outputs 1 and 2. Just set it
    // up to read all of port A
    unsigned int    StartAddress    = 0;
    unsigned int    NumberOfCoils   = 0;
    unsigned int    i               = 0;
    unsigned char   CurrentData     = 0;
    unsigned char   CurrentBit      = 0;

    // The message contains the requested start address and number of inputs
    StartAddress    = ((unsigned int) (Rx_Data.DataBuf[0]) << 8) + (unsigned int) (Rx_Data.DataBuf[1]);
    NumberOfCoils   = ((unsigned int) (Rx_Data.DataBuf[2]) << 8) + (unsigned int) (Rx_Data.DataBuf[3]);

    if((StartAddress+NumberOfCoils)>NUMBER_OF_COILS)
        HandleModbusError(ERROR_CODE_03);
    else
    {
        // Initialise the output buffer. The first byte in the buffer says how many coils we have read
        Tx_Data.Function    = MBFN_READ_COILS;
        Tx_Data.Address     = SLAVE_ADDRESS;
        Tx_Data.DataLen     = 1;
        Tx_Data.DataBuf[0]  = 0;

        for (i = 0; i < NumberOfCoils; i++)
        {
            unsigned char coilBit       = (StartAddress + i) % 8;
            unsigned char coil_index    = (StartAddress + i) / 8;

            if (coil_index < NUMBER_COIL_BYTES)
            {
                // Read the relevant bit out of the port B variable. This may not start at bit 0 if the requested
                // address is not aligned on an 8 value boundary
                if (ReadCoilOutputs[coil_index] & (1 << coilBit))
                    CurrentData |= 1 << CurrentBit;
                CurrentBit++;
            }
            else
            {
                // Populate with 0 for unset
                CurrentBit++;
            }

            if (CurrentBit > 8)
            {
                // Filled a whole byte
                Tx_Data.DataBuf[Tx_Data.DataLen] = CurrentData;
                Tx_Data.DataBuf[0] = Tx_Data.DataLen++;
                // Reset for next byte
                CurrentBit = 0;
                CurrentData = 0;
            }
        }
        // Populate the data we've just read
        if (CurrentBit > 0)
        {
            Tx_Data.DataBuf[Tx_Data.DataLen]    = CurrentData;
            Tx_Data.DataBuf[0]                  = Tx_Data.DataLen++;
        }

        SendMessage();
    }
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusReadDiscreteInputs
 * @How to use          : Modbus function 02 - Read input status
 */
#if MBFN_READ_DISCRETE_INPUTS_ENABLED > 0
void HandleModbusReadDiscreteInputs(void)
{
    // Read digital inputs. In our implementation this reads the inputs in port B and port D
    unsigned int    StartAddress    = 0;
    unsigned int    NumberOfInputs  = 0;
    unsigned int    i               = 0;
    unsigned char   CurrentData     = 0;
    unsigned char   CurrentBit      = 0;

    // The message contains the requested start address and number of inputs
    StartAddress   = ((unsigned int) (Rx_Data.DataBuf[0]) << 8) + (unsigned int) (Rx_Data.DataBuf[1]);
    NumberOfInputs = ((unsigned int) (Rx_Data.DataBuf[2]) << 8) + (unsigned int) (Rx_Data.DataBuf[3]);

    // Initialise the output buffer. The first byte in the buffer says how many inputs we have read
    Tx_Data.Function    = MBFN_READ_DISCRETE_INPUTS;
    Tx_Data.Address     = SLAVE_ADDRESS;
    Tx_Data.DataLen     = 1;
    Tx_Data.DataBuf[0]  = 0;

    if((StartAddress+NumberOfInputs)>NUMBER_OF_INPUTS)
        HandleModbusError(ERROR_CODE_03);
    else
    {
        for (i = 0; i < NumberOfInputs; ++i)
        {
            unsigned char inputBit      = (StartAddress + i) % 8;
            unsigned char port_index    = (StartAddress + i) / 8;

            if (port_index < NUMBER_INPUT_BYTES)
            {
                // Read the relevant bit out of the variable. This may not start at bit 0 if the requested
                // address is not aligned on an 8 value boundary
                if (DiscreteInputs[port_index] & (1 << inputBit))
                    CurrentData |= 1 << CurrentBit;
                ++CurrentBit;
            }
            else
            {
                // Populate with 0 for unset
                ++CurrentBit;
            }

            if (CurrentBit >= 8)
            {
                // Filled a whole byte
                Tx_Data.DataBuf[Tx_Data.DataLen]    = CurrentData;
                Tx_Data.DataBuf[0]                  = Tx_Data.DataLen++;
                // Reset for next byte
                CurrentBit  = 0;
                CurrentData = 0;
            }
        }
        // Populate the data we've just read
        if (CurrentBit > 0)
        {
            Tx_Data.DataBuf[Tx_Data.DataLen]    = CurrentData;
            Tx_Data.DataBuf[0]                  = Tx_Data.DataLen++;
        }
        SendMessage();
    }
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusReadHoldingRegisters
 * @How to use          : Modbus function 03 - Read holding registers
 */
#if MBFN_READ_HOLDING_REGISTERS_ENABLED > 0
void HandleModbusReadHoldingRegisters(void)
{
    // Holding registers are effectively numerical outputs that can be written to by the host.
    // They can be control registers or analogue outputs.
    // We potientially have one - the pwm output value
    unsigned int    StartAddress        = 0;
    unsigned int    NumberOfRegisters   = 0;
    unsigned int    i                   = 0;

    // The message contains the requested start address and number of registers
    StartAddress        = ((unsigned int) (Rx_Data.DataBuf[0]) << 8) + (unsigned int) (Rx_Data.DataBuf[1]);
    NumberOfRegisters   = ((unsigned int) (Rx_Data.DataBuf[2]) << 8) + (unsigned int) (Rx_Data.DataBuf[3]);

    if((StartAddress+NumberOfRegisters)>NUMBER_OF_OUTPUT_REGISTERS)
        HandleModbusError(ERROR_CODE_03);
    else
    {
        // Initialise the output buffer. The first byte in the buffer says how many registers we have read
        Tx_Data.Function    = MBFN_READ_HOLDING_REGISTERS;
        Tx_Data.Address     = SLAVE_ADDRESS;
        Tx_Data.DataLen     = 1;
        Tx_Data.DataBuf[0]  = 0;

        for (i = 0; i < NumberOfRegisters; i++)
        {
            unsigned int currentData = RegisterOutputs[StartAddress+i].ActValue;

            Tx_Data.DataBuf[Tx_Data.DataLen]        = (unsigned char) ((currentData & 0xFF00) >> 8);
            Tx_Data.DataBuf[Tx_Data.DataLen + 1]    = (unsigned char) (currentData & 0xFF);
            Tx_Data.DataLen                        += 2;
            Tx_Data.DataBuf[0]                      = Tx_Data.DataLen - 1;
        }

        SendMessage();
    }
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusReadInputRegisters
 * @How to use          : Modbus function 04 - Read input registers
 */
#if MBFN_READ_INPUT_REGISTERS_ENABLED > 0
void HandleModbusReadInputRegisters(void)
{
    // Input registers are effectively numerical inputs such as analogue inputs
    // We have 1 or 2 depending on the configuration at the top of this file
    unsigned int    StartAddress        = 0;
    unsigned int    NumberOfRegisters   = 0;
    unsigned int    i                   = 0;

    // The message contains the requested start address and number of registers
    StartAddress        = ((unsigned int) (Rx_Data.DataBuf[0]) << 8) + (unsigned int) (Rx_Data.DataBuf[1]);
    NumberOfRegisters   = ((unsigned int) (Rx_Data.DataBuf[2]) << 8) + (unsigned int) (Rx_Data.DataBuf[3]);

    if((StartAddress+NumberOfRegisters)>NUMBER_OF_INPUT_REGISTERS)
        HandleModbusError(ERROR_CODE_03);
    else
    {
        // Initialise the output buffer. The first byte in the buffer says how many registers we have read
        Tx_Data.Function    = MBFN_READ_INPUT_REGISTERS;
        Tx_Data.Address     = SLAVE_ADDRESS;
        Tx_Data.DataLen     = 1;
        Tx_Data.DataBuf[0]  = 0;

        for (i = 0; i < NumberOfRegisters; i++)
        {
            int current_data                        = RegisterInputs[StartAddress+i].ActValue;
            Tx_Data.DataBuf[Tx_Data.DataLen]        = (unsigned char) ((current_data & 0xff00) >> 8);
            Tx_Data.DataBuf[Tx_Data.DataLen + 1]    = (unsigned char) (current_data & 0xff);
            Tx_Data.DataLen                        += 2;
            Tx_Data.DataBuf[0]                      = Tx_Data.DataLen - 1;
        }
        SendMessage();
    }
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusWriteSingleCoil
 * @How to use          : Modbus function 05 - Write single coil
 */
#if MBFN_WRITE_SINGLE_COIL_ENABLED > 0
void HandleModbusWriteSingleCoil(void)
{
    // Write single coil output - these are on port A
    unsigned int address = 0;
    unsigned int value = 0;
    unsigned char i = 0;
    unsigned char port_number = 0;
    unsigned char bit_number = 0;

    // The message contains the requested start address and number of registers
    address     = ((unsigned int) (Rx_Data.DataBuf[0]) << 8) + (unsigned int) (Rx_Data.DataBuf[1]);
    value       = ((unsigned int) (Rx_Data.DataBuf[2]) << 8) + (unsigned int) (Rx_Data.DataBuf[3]);

    // Initialise the output buffer. The first byte in the buffer says how many registers we have read
    Tx_Data.Function    = MBFN_WRITE_SINGLE_COIL;
    Tx_Data.Address     = SLAVE_ADDRESS;
    Tx_Data.DataLen     = 4;
    // Output data buffer is exact copy of input buffer
    for (i = 0; i < 4; ++i)
    {
        Tx_Data.DataBuf[i] = Rx_Data.DataBuf[i];
    }

    // Calculate the port/bit
    port_number     = address / 8;
    bit_number      = address % 8;

    // Read the port data
    if (port_number < NUMBER_COIL_BYTES)
    {
        if (value)
        {
            WriteCoilOutputs[port_number] |= 1 << bit_number;
        }
        else
        {
            WriteCoilOutputs[port_number] &= ~(1 << bit_number);
        }
    }
    SendMessage();
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusReadInputRegisters
 * @How to use          : Modbus function 06 - Write single register
 */
#if MBFN_WRITE_SINGLE_REGISTER_ENABLED > 0
void HandleModbusWriteSingleRegister(void)
{
    // Write single numerical output
    unsigned int Address = 0;
    unsigned int Value = 0;
    unsigned char i = 0;

    // The message contains the requested start address and number of registers
    Address             = ((unsigned int) (Rx_Data.DataBuf[0]) << 8) + (unsigned int) (Rx_Data.DataBuf[1]);
    Value               = ((unsigned int) (Rx_Data.DataBuf[2]) << 8) + (unsigned int) (Rx_Data.DataBuf[3]);

    // Initialise the output buffer. The first byte in the buffer says how many registers we have read
    Tx_Data.Function    = MBFN_WRITE_SINGLE_REGISTER;
    Tx_Data.Address     = SLAVE_ADDRESS;
    Tx_Data.DataLen     = 4;

    if(Address>=NUMBER_OF_OUTPUT_REGISTERS)
        HandleModbusError(ERROR_CODE_02);
    else
    {
        if(RegisterOutputs[Address].RW==1)
        {
            if(RegisterOutputs[Address].MaxValue>=Value && RegisterOutputs[Address].MinValue<=Value)
            {
                RegisterOutputs[Address].ActValue=Value;
                // Output data buffer is exact copy of input buffer
                for (i = 0; i < 4; ++i)
                    Tx_Data.DataBuf[i] = Rx_Data.DataBuf[i];
            }
            else
                HandleModbusError(ERROR_CODE_03);
        }
        else
            HandleModbusError(ERROR_CODE_02);
    }

    SendMessage();
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusWriteMultipleCoils
 * @How to use          : Modbus function 15 - Write multiple coils
 */
#if MBFN_WRITE_MULTIPLE_COILS_ENABLED > 0
void HandleModbusWriteMultipleCoils(void)
{
    // Write single numerical output
    unsigned int    address = 0;
    unsigned int    coils = 0;
    unsigned char   byteCount = 0;
    unsigned char   i = 0;
    unsigned int    count;

    // The message contains the requested start address and number of registers
    address     = ((unsigned int) (Rx_Data.DataBuf[0]) << 8) + (unsigned int) (Rx_Data.DataBuf[1]);
    coils       = ((unsigned int) (Rx_Data.DataBuf[2]) << 8) + (unsigned int) (Rx_Data.DataBuf[3]);
    byteCount   = Rx_Data.DataBuf[4];

    // Initialise the output buffer. The first byte in the buffer says how many outputs we have set
    Tx_Data.Function    = MBFN_WRITE_MULTIPLE_COILS;
    Tx_Data.Address     = SLAVE_ADDRESS;
    Tx_Data.DataLen     = 4;
    Tx_Data.DataBuf[0]  = Rx_Data.DataBuf[0];
    Tx_Data.DataBuf[1]  = Rx_Data.DataBuf[1];
    Tx_Data.DataBuf[2]  = Rx_Data.DataBuf[2];
    Tx_Data.DataBuf[3]  = Rx_Data.DataBuf[3];

    // Sanity check - ensure we have been given enough values
    count = byteCount * 8;
    if (count < coils)
    {
        HandleModbusError(ERROR_CODE_03);
        return;
    }

    for (i = 0; i < coils; ++i)
    {
        unsigned char byte = i / 8;
        unsigned char bt = i % 8;
        unsigned char value = (Rx_Data.DataBuf[5 + byte] & (1 << bt));

        // Write it to the output port
        if (byte < NUMBER_COIL_BYTES)
        {
            unsigned char mask = 1 << (address + bt);
            if (value)
            {
                WriteCoilOutputs[byte] |= mask;
            }
            else
            {
                WriteCoilOutputs[byte] &= ~mask;
            }
        }
    }
    SendMessage();
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusWriteMultipleRegisters
 * @How to use          : Modbus function 16 - Write multiple registers
 */
#if MBFN_WRITE_MULTIPLE_REGISTERS_ENABLED > 0
void HandleModbusWriteMultipleRegisters(void)
{
    // Write single numerical output
    unsigned int    StartAddress				=0;
    unsigned int    NumberOfRegisters   =0;
    unsigned char   i										=0;
    unsigned int	Value									=0;

    // The message contains the requested start address and number of registers
    StartAddress        = ((unsigned int) (Rx_Data.DataBuf[0]) << 8) + (unsigned int) (Rx_Data.DataBuf[1]);
    NumberOfRegisters   = ((unsigned int) (Rx_Data.DataBuf[2]) << 8) + (unsigned int) (Rx_Data.DataBuf[3]);

    if((StartAddress+NumberOfRegisters)>NUMBER_OF_OUTPUT_REGISTERS)
        HandleModbusError(ERROR_CODE_02);
    else
    {
        // Initialise the output buffer. The first byte in the buffer says how many outputs we have set
        Tx_Data.Function    = MBFN_WRITE_MULTIPLE_REGISTERS;
        Tx_Data.Address     = SLAVE_ADDRESS;
        Tx_Data.DataLen     = 4;
        Tx_Data.DataBuf[0]  = Rx_Data.DataBuf[0];
        Tx_Data.DataBuf[1]  = Rx_Data.DataBuf[1];
        Tx_Data.DataBuf[2]  = Rx_Data.DataBuf[2];
        Tx_Data.DataBuf[3]  = Rx_Data.DataBuf[3];

        // Output data buffer is exact copy of input buffer
        for (i = 0; i <NumberOfRegisters; i++)
        {
            Value=(Rx_Data.DataBuf[5+2*i]<<8)+(Rx_Data.DataBuf[6+2*i]);

            if(RegisterOutputs[StartAddress+i].RW==1)
            {
                if((RegisterOutputs[StartAddress+i].MaxValue>=Value) && (RegisterOutputs[StartAddress+i].MinValue<=Value))
                    RegisterOutputs[StartAddress+i].ActValue=Value;
                else
                    HandleModbusError(ERROR_CODE_03);
            }
            else
                HandleModbusError(ERROR_CODE_02);
        }

        SendMessage();
    }
}
#endif

/******************************************************************************/

/*
 * Function Name        : HandleModbusMasterReadHoldingRegisters
 * @How to use          :
 */
#if MBFN_MASTER_REGISTERS_ENABLED > 0
void HandleModbusMasterReadHoldingRegisters(void)
{
    // Write single numerical output
    unsigned int    NumberOfRegisters           =0;
    unsigned char   i				=0;
    unsigned int    Value			=0;

    // The message contains the requested start address and number of registers
    NumberOfRegisters   = ((unsigned int) (MasterRead_Data.DataBuf[0]))>> 1;

    // Output data buffer is exact copy of input buffer
    for (i = 0; i <NumberOfRegisters; i++)
    {
        Value=(MasterRead_Data.DataBuf[2*i+1]<<8)+(MasterRead_Data.DataBuf[2+2*i]);

        if(i<NUMBER_MASTER_INPUT_REGISTERS)
        {
            MasterRegisterInputs[i].ActValue=Value;
        }
    }
}
#endif

/******************************************************************************/

/*
 * Function Name        : RxDataAvailable
 * @return              : Datalar hazirsa TRUE
 *                        Datalar hazir degilse FALSE
 */
unsigned char RxDataAvailable(void)
{
    unsigned char Result    = Rx_Data_Available;
    Rx_Data_Available       = FALSE;

    return Result;
}

/******************************************************************************/

/*
 * Function Name        : CheckRxTimeout
 * @return              : Zaman asimi olmussa TRUE
 *                        Zaman asimi olmamissa FALSE
 */
unsigned char CheckRxTimeout(void)
{
    // A return value of true indicates there is a timeout    
    if (SlaveTimerValue>= TIMEOUTTIMER)
    {
        SlaveTimerValue =0;
        ReceiveCounter  =0;
        return TRUE;
    }

    return FALSE;
}

/******************************************************************************/

/*
 * Function Name        : CheckBufferComplete
 * @return              : Alim kesmesinden bilgiler dogru gelmise   DATA_READY
 *                        Alim kesmesinden slave adresi yanlissa    FALSE_SLAVE_ADDRESS
 *                        Alim kesmesinden bilgiler daha aliniyorsa DATA_NOT_READY
 *                        Alim kesmesinden fonksiyon yanlissa       FALSE_FUNCTION
 */
unsigned char CheckSlaveBufferComplete(void)
{
    int ExpectedReceiveCount=0;

    if(ReceiveCounter>4)
    {
        if(ReceiveBuffer[0]==SLAVE_ADDRESS)
        {
            if(ReceiveBuffer[1]==0x01 || ReceiveBuffer[1]==0x02 || ReceiveBuffer[1]==0x03 || ReceiveBuffer[1]==0x04 || ReceiveBuffer[1]==0x05 || ReceiveBuffer[1]==0x06)  // RHR
            {
                ExpectedReceiveCount    =8;
            }
            else if(ReceiveBuffer[1]==0x0F || ReceiveBuffer[1]==0x10)
            {
                ExpectedReceiveCount=ReceiveBuffer[6]+9;
            }
            else
            {
                ReceiveCounter=0;
                return FALSE_FUNCTION;   // beklenmeyen fonksiyon
            }
        }
        else
        {
            ReceiveCounter=0;
            return FALSE_SLAVE_ADDRESS;   // beklenmeyen adres
        }
    }
    else
        return DATA_NOT_READY;       // Cevap bitmedi

    if(ReceiveCounter==ExpectedReceiveCount)
    {
        return DATA_READY;       // cevap bitti.
    }

    return DATA_NOT_READY;
}

/******************************************************************************/

/*
 * Function Name        : RxRTU
 * @How to use          : Surekli dongu icerisinde kalmali. Gonderilecek data
 *                        hazirsa bu data gonderilir.
 */
void RxRTU(void)
{
    unsigned char   i;
    unsigned char   ReceiveBufferControl=0;

    ReceiveBufferControl    =CheckSlaveBufferComplete();

    if(ReceiveBufferControl==DATA_READY)
    {
        Rx_Data.Address         =ReceiveBuffer[0];
        Rx_CRC16                = 0xffff;
        CRC16(Rx_Data.Address, &Rx_CRC16);
        Rx_Data.Function        =ReceiveBuffer[1];
        CRC16(Rx_Data.Function, &Rx_CRC16);

        Rx_Data.DataLen=0;

        for(i=2;i<ReceiveCounter;i++)
            Rx_Data.DataBuf[Rx_Data.DataLen++]=ReceiveBuffer[i];

        Rx_State =RXTX_DATABUF;

        ReceiveCounter=0;
    }

    CheckRxTimeout();

    if ((Rx_State == RXTX_DATABUF) && (Rx_Data.DataLen >= 2))
    {
        // Finish off our CRC check
        Rx_Data.DataLen -= 2;
        for (i = 0; i < Rx_Data.DataLen; ++i)
        {
            CRC16(Rx_Data.DataBuf[i], &Rx_CRC16);
        }
        
        if (((unsigned int) Rx_Data.DataBuf[Rx_Data.DataLen] + ((unsigned int) Rx_Data.DataBuf[Rx_Data.DataLen + 1] << 8)) == Rx_CRC16)
        {
            // Valid message!
            Rx_Data_Available = TRUE;
        }

        Rx_State =RXTX_IDLE;
    }
}

/******************************************************************************/

/*
 * Function Name        : TxRTU
 * @How to use          : Gonderim bilgisi hazirsa bu fonksiyon ile gonderim yapilir.
 */
void TxRTU(void)
{
    Tx_CRC16                =0xFFFF;
    Tx_Buf_Size             =0;
    Tx_Buf[Tx_Buf_Size++]   =Tx_Data.Address;
    CRC16(Tx_Data.Address, &Tx_CRC16);
    Tx_Buf[Tx_Buf_Size++]   =Tx_Data.Function;
    CRC16(Tx_Data.Function, &Tx_CRC16);

    for(Tx_Current=0;Tx_Current < Tx_Data.DataLen;Tx_Current++)
    {
        Tx_Buf[Tx_Buf_Size++]=Tx_Data.DataBuf[Tx_Current];
        CRC16(Tx_Data.DataBuf[Tx_Current], &Tx_CRC16);
    }
    Tx_Buf[Tx_Buf_Size++] = Tx_CRC16 & 0x00FF;
    Tx_Buf[Tx_Buf_Size++] =(Tx_CRC16 & 0xFF00) >> 8;

    DoSlaveTX();

    Tx_State    =RXTX_IDLE;
}

/******************************************************************************/

/*
 * Function Name        : ProcessModbus
 * @How to use          : ModBus cekirdegi. Bu fonksiyon ana dongu icerisinde
 *                        cagrilmali.
 */
void ProcessModbusSlave(void)
{
    if (Tx_State != RXTX_IDLE)                                                  // Gonderim datalari hazirsa gonderim baslatilir
        TxRTU();

    RxRTU();                                                                    // Alim fonksiyonu surekli isletilir

    if (RxDataAvailable())                                                      // Data alimi tamamlanmissa bu foksiyona girilir
    {
        if (Rx_Data.Address == SLAVE_ADDRESS)                                   // Data bizim icin mi?
        {
            switch (Rx_Data.Function)                                           // Data bizim icinse hangi fonksiyon isletilecek?
            {
                #if MBFN_READ_COILS_ENABLED > 0
                case MBFN_READ_COILS:               {   HandleModbusReadCoils();                break;  }
                #endif
                #if MBFN_READ_DISCRETE_INPUTS_ENABLED > 0
                case MBFN_READ_DISCRETE_INPUTS:     {   HandleModbusReadDiscreteInputs();       break;  }
                #endif
                #if MBFN_READ_HOLDING_REGISTERS_ENABLED > 0
                case MBFN_READ_HOLDING_REGISTERS:   {   HandleModbusReadHoldingRegisters();     break;  }
                #endif
                #if MBFN_READ_INPUT_REGISTERS_ENABLED > 0
                case MBFN_READ_INPUT_REGISTERS:     {   HandleModbusReadInputRegisters();       break;  }
                #endif
                #if MBFN_WRITE_SINGLE_COIL_ENABLED > 0
                case MBFN_WRITE_SINGLE_COIL:        {   HandleModbusWriteSingleCoil();          break;  }
                #endif
                #if MBFN_WRITE_SINGLE_REGISTER_ENABLED > 0
                case MBFN_WRITE_SINGLE_REGISTER:    {   HandleModbusWriteSingleRegister();      break;  }
                #endif
                #if MBFN_WRITE_MULTIPLE_COILS_ENABLED > 0
                case MBFN_WRITE_MULTIPLE_COILS:     {   HandleModbusWriteMultipleCoils();       break;  }
                #endif
                #if MBFN_WRITE_MULTIPLE_REGISTERS_ENABLED > 0
                case MBFN_WRITE_MULTIPLE_REGISTERS: {   HandleModbusWriteMultipleRegisters();   break;  }
                #endif
                default:                            {   HandleModbusError(ERROR_CODE_06);       break;  }
            }
        }
    }
}

/******************************************************************************/

/*
 * Function Name        : InitModbusSlave
 * @How to use          : ModBus slave calismasi icin donanimsal ilklendirmeleri.
 */
void InitModbusSlave(unsigned char SlaveAddress)
{
    if(SlaveAddress<=0 && SlaveAddress>255)
        SlaveAddress=1;

    SLAVE_ADDRESS   =SlaveAddress;
    ModBusSlave_UART_Initialise();
    ModBusSlave_TIMER_Initialise();
}

/******************************************************************************/
/*
 * Function Name        : InitModbusMaster
 * @How to use          : ModBus master calismasi icin donanimsal ilklendirmeleri.
 */
void InitModbusMaster(void)
{
    ModBusMaster_UART_Initialise();
    ModBusMaster_TIMER_Initialise();
}

/******************************************************************************/

/*
 * Function Name        : DoTx
 * @param[out]          : TRUE
 * @How to use          : Gonderilecek dizi bu fonksiyon ile gonderilir.
 */

unsigned char DoMasterTX(unsigned int Length)
{  
    ModBusMaster_UART_String(MasterTx_Buf,Length);

    MasterTx_Buf_Size = 0;
    return TRUE;
}

/*
 * Function Name        : CheckBufferComplete
 * @return              : Alim kesmesinden bilgiler dogru gelmise   DATA_READY
 *                        Alim kesmesinden slave adresi yanlissa    FALSE_SLAVE_ADDRESS
 *                        Alim kesmesinden bilgiler daha aliniyorsa DATA_NOT_READY
 *                        Alim kesmesinden fonksiyon yanlissa       FALSE_FUNCTION
 */
unsigned char CheckMasterBufferComplete(unsigned char SlaveNumber)
{
    int MasterExpectedReceiveCount=0;

    if(MasterReceiveCounter>4)
    {
        if(MasterReceiveBuffer[0]==SlaveNumber)
        {
            if(MasterReceiveBuffer[1]==0x03)  // RHR
            {
                MasterExpectedReceiveCount    =MasterReceiveBuffer[2]+5;
            }
            else if(MasterReceiveBuffer[1]==0x10|| MasterReceiveBuffer[1]==0x06)
            {
                MasterExpectedReceiveCount=8;
            }
            else
            {
                MasterReceiveCounter=0;
                return FALSE_FUNCTION;   // beklenmeyen fonksiyon
            }
        }
        else
        {
            MasterReceiveCounter=0;
            return FALSE_SLAVE_ADDRESS;   // beklenmeyen adres
        }
    }
    else
        return DATA_NOT_READY;       // Cevap bitmedi

    if(MasterReceiveCounter==MasterExpectedReceiveCount)
    {
        return DATA_READY;       // cevap bitti.
    }

    return DATA_NOT_READY;
}

/******************************************************************************/
#if MBFN_MASTER_REGISTERS_ENABLED > 0
unsigned char ModBusMasterRead(unsigned char SlaveNumber, unsigned char Function, unsigned int StartAddress, unsigned int NumberOfRegisters, unsigned int TimeOut)
{
    unsigned char   i,j;
    unsigned char   ReturnValue=FALSE;

    switch(MasterRead_State)
    {
        case RXTX_IDLE  :

            MasterRead_CRC16            =0xFFFF;
            MasterTx_Buf_Size           =8;
            MasterTx_Buf[0]             =SlaveNumber;
            MasterTx_Buf[1]             =Function;
            MasterTx_Buf[2]             =StartAddress>>8;
            MasterTx_Buf[3]             =StartAddress&0xFF;
            MasterTx_Buf[4]             =NumberOfRegisters>>8;
            MasterTx_Buf[5]             =NumberOfRegisters&0xFF;

            CRC16(MasterTx_Buf[0], &MasterRead_CRC16);
            CRC16(MasterTx_Buf[1], &MasterRead_CRC16);
            CRC16(MasterTx_Buf[2], &MasterRead_CRC16);
            CRC16(MasterTx_Buf[3], &MasterRead_CRC16);
            CRC16(MasterTx_Buf[4], &MasterRead_CRC16);
            CRC16(MasterTx_Buf[5], &MasterRead_CRC16);

            MasterTx_Buf[6]             = MasterRead_CRC16 & 0x00FF;
            MasterTx_Buf[7]             =(MasterRead_CRC16 & 0xFF00) >> 8;

            DoMasterTX(MasterTx_Buf_Size);

            MasterReceiveCounter        =0;
            MasterRead_State            =RXTX_WAIT_ANSWER;
            MasterReadTimerValue        =0;
            ReturnValue                 =FALSE;
            break;

        case RXTX_WAIT_ANSWER:

            if(MasterReadTimerValue>TimeOut)
                MasterRead_State        =RXTX_TIMEOUT;

            if(CheckMasterBufferComplete(SlaveNumber)==DATA_READY)
            {
                MasterRead_Data.DataLen       =0;
                MasterRead_Data.Address       =MasterReceiveBuffer[0];
                MasterRead_Data.Function      =MasterReceiveBuffer[1];

                for(i=2;i<MasterReceiveCounter;i++)
                    MasterRead_Data.DataBuf[MasterRead_Data.DataLen++]=MasterReceiveBuffer[i];

                MasterRead_CRC16              = 0xFFFF;

                CRC16(MasterRead_Data.Address, &MasterRead_CRC16);
                CRC16(MasterRead_Data.Function,&MasterRead_CRC16);

                // Finish off our CRC check
                MasterRead_Data.DataLen -= 2;
                for (i = 0; i < MasterRead_Data.DataLen; ++i)
                    CRC16(MasterRead_Data.DataBuf[i], &MasterRead_CRC16);

                if (((unsigned int) MasterRead_Data.DataBuf[MasterRead_Data.DataLen] +
                		((unsigned int) MasterRead_Data.DataBuf[MasterRead_Data.DataLen + 1] << 8))
                		== MasterRead_CRC16)
                {
                    // Valid message!
                    switch(MasterRead_Data.Function)
                    {
                        case MBFN_READ_HOLDING_REGISTERS:
                            HandleModbusMasterReadHoldingRegisters();
                            for (i = 0; i < NUMBER_MASTER_LOOKUP_INPUTS; ++i) {
								if(MasterLookupTableInputs[i].LookupAddress == StartAddress) {
									for (j = 0; j < MasterLookupTableInputs[i].Size; ++j) {
										MasterLookupTableInputs[i].RegisterInput[j].ActValue =
											MasterRegisterInputs[j].ActValue;
									}

									break;
								}
                            }
                            ReturnValue             = TRUE;
                            break;
                        default:
                            ReturnValue             = FALSE;
                            break;
                    }
                    MasterRead_State    =RXTX_IDLE;
                }
            }
            break;

        case RXTX_TIMEOUT:
            MasterReadTimerValue        =0;
            MasterReceiveCounter        =0;
            ReturnValue                 =TRUE;
            MasterRead_State            =RXTX_IDLE;
            break;

        default:
            ReturnValue                 =FALSE;
            MasterRead_State            =RXTX_IDLE;
            break;
    }

    return ReturnValue;
}
#endif

#if MBFN_MASTER_REGISTERS_ENABLED > 0
unsigned char ModBusMasterWrite(unsigned char SlaveNumber, unsigned char Function, unsigned int StartAddress, unsigned char NumberOfData, unsigned int *Data,  unsigned int TimeOut)
{
    // Write single numerical output
    unsigned char   i;
    unsigned char   ReturnValue=FALSE;
    unsigned int    RegStartAddress		=0;
    unsigned int    NumberOfRegisters           =0;
    unsigned int    Value                       =0;

    switch(MasterWrite_State)
    {
        case RXTX_IDLE  :

            if(Function==0x06)                                                  // Function 06
            {
                MasterWrite_CRC16           =0xFFFF;
                MasterTx_Buf_Size           =8;

                MasterTx_Buf[0]             =SlaveNumber;
                MasterTx_Buf[1]             =6;
                MasterTx_Buf[2]             =StartAddress>>8;
                MasterTx_Buf[3]             =StartAddress&0xFF;
                MasterTx_Buf[4]             =Data[0]>>8;
                MasterTx_Buf[5]             =Data[0]&0xFF;

                CRC16(MasterTx_Buf[0], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[1], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[2], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[3], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[4], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[5], &MasterWrite_CRC16);

                MasterTx_Buf[6]             = MasterWrite_CRC16 & 0x00FF;
                MasterTx_Buf[7]             =(MasterWrite_CRC16 & 0xFF00) >> 8;

            }
            else if(Function==0x10)                                             // Function 16
            {
                MasterWrite_CRC16           =0xFFFF;
                MasterTx_Buf_Size           =9+NumberOfData*2;

                MasterTx_Buf[0]             =SlaveNumber;
                MasterTx_Buf[1]             =16;
                MasterTx_Buf[2]             =StartAddress>>8;
                MasterTx_Buf[3]             =StartAddress&0xFF;
                MasterTx_Buf[4]             =NumberOfData>>8;
                MasterTx_Buf[5]             =NumberOfData&0xFF;
                MasterTx_Buf[6]             =NumberOfData*2;

                CRC16(MasterTx_Buf[0], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[1], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[2], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[3], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[4], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[5], &MasterWrite_CRC16);
                CRC16(MasterTx_Buf[6], &MasterWrite_CRC16);

                for(i=0;i<NumberOfData;i++)
                {
                    MasterTx_Buf[7+2*i]=Data[i]>>8;
                    MasterTx_Buf[8+2*i]=Data[i]&0xFF;

                    CRC16(MasterTx_Buf[7+2*i], &MasterWrite_CRC16);
                    CRC16(MasterTx_Buf[8+2*i], &MasterWrite_CRC16);
                }

                MasterTx_Buf[7+2*NumberOfData]=MasterWrite_CRC16 & 0x00FF;
                MasterTx_Buf[8+2*NumberOfData]=(MasterWrite_CRC16 & 0xFF00) >> 8;
            }

            DoMasterTX(MasterTx_Buf_Size);
            MasterWrite_State           =RXTX_WAIT_ANSWER;
            MasterWriteTimerValue       =0;
            MasterReceiveCounter        =0;
            break;

        case RXTX_WAIT_ANSWER:
            if(MasterWriteTimerValue>TimeOut)
                MasterWrite_State        =RXTX_TIMEOUT;

            if(CheckMasterBufferComplete(SlaveNumber)==DATA_READY)
            {
                MasterWrite_Data.DataLen       =0;
                MasterWrite_Data.Address       =MasterReceiveBuffer[0];
                MasterWrite_Data.Function      =MasterReceiveBuffer[1];

                for(i=2;i<MasterReceiveCounter;i++)
                    MasterWrite_Data.DataBuf[MasterWrite_Data.DataLen++]=MasterReceiveBuffer[i];

                MasterWrite_CRC16              = 0xFFFF;

                CRC16(MasterWrite_Data.Address, &MasterWrite_CRC16);
                CRC16(MasterWrite_Data.Function,&MasterWrite_CRC16);

                // Finish off our CRC check
                MasterWrite_Data.DataLen -= 2;
                for (i = 0; i < MasterWrite_Data.DataLen; ++i)
                    CRC16(MasterWrite_Data.DataBuf[i], &MasterWrite_CRC16);

                if (((unsigned int) MasterWrite_Data.DataBuf[MasterWrite_Data.DataLen] + ((unsigned int) MasterWrite_Data.DataBuf[MasterWrite_Data.DataLen + 1] << 8)) == MasterWrite_CRC16)
                {
                    // Valid message!
                    switch(MasterWrite_Data.Function)
                    {
                        case MBFN_WRITE_SINGLE_REGISTER:
                            RegStartAddress     =((unsigned int) MasterWrite_Data.DataBuf[0] << 8)+(unsigned int) MasterWrite_Data.DataBuf[1];
                            Value               =((unsigned int) MasterWrite_Data.DataBuf[2] << 8)+(unsigned int) MasterWrite_Data.DataBuf[3];

                            if((MasterWrite_Data.Function==Function)&&(StartAddress==RegStartAddress)&&(Value==Data[0]))
                                ReturnValue             = TRUE;
                            else
                                ReturnValue             = FALSE;
                            break;

                        case MBFN_WRITE_MULTIPLE_REGISTERS:
                            RegStartAddress     =((unsigned int) MasterWrite_Data.DataBuf[0] << 8)+(unsigned int) MasterWrite_Data.DataBuf[1];
                            NumberOfRegisters   =((unsigned int) MasterWrite_Data.DataBuf[2] << 8)+(unsigned int) MasterWrite_Data.DataBuf[3];

                            if((MasterWrite_Data.Function==Function)&&(StartAddress==RegStartAddress)&&(NumberOfRegisters==NumberOfData))
                                ReturnValue             = TRUE;
                            else
                                ReturnValue             = FALSE;
                            break;

                        default:
                            ReturnValue             = FALSE;
                            break;
                    }
                    MasterWrite_State    =RXTX_IDLE;
                }
            }

            break;

        case RXTX_TIMEOUT:
            MasterWriteTimerValue       =0;
            MasterReceiveCounter        =0;
            ReturnValue                 =FALSE;
            MasterWrite_State           =RXTX_IDLE;
            break;

        default:
            ReturnValue                 =FALSE;
            MasterWrite_State           =RXTX_IDLE;
            break;
    }

    return ReturnValue;
}
#endif
