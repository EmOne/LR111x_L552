/*!
 * \file      radio.c
 *
 * \brief     Radio driver API definition
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Benjamin Boulet ( Semtech )
 */
#include <math.h>
#include <string.h>
#include "radio.h"
#include "lr1110.h"
#include "lr1110_hal.h"
#include "lr1110_radio.h"
#include "lr1110_system.h"
#include "lr1110_regmem.h"
#include "lr1110-board.h"
#include "stm32l552e_eval.h"
#include "timer.h"
#include "delay.h"
/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
void RadioInit( RadioEvents_t* events );

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t RadioGetStatus( void );

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void RadioSetModem( RadioModems_t modem );

/*!
 * \brief Sets the channel frequency
 *
 * \param [IN] freq         Channel RF frequency
 */
void RadioSetChannel( uint32_t freq );

/*!
 * \brief Checks if the channel is free for the given time
 *
 * \remark The FSK modem is always used for this task as we can select the Rx bandwidth at will.
 *
 * \param [IN] freq                Channel RF frequency in Hertz
 * \param [IN] rxBandwidth         Rx bandwidth in Hertz
 * \param [IN] rssiThresh          RSSI threshold in dBm
 * \param [IN] maxCarrierSenseTime Max time in milliseconds while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool RadioIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime );

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either Radio.SetRxConfig or
 *         Radio.SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
uint32_t RadioRandom( void );

/*!
 * \brief Sets the reception parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout in number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
                       uint32_t bandwidthAfc, uint16_t preambleLen, uint16_t symbTimeout, bool fixLen,
                       uint8_t payloadLen, bool crcOn, bool FreqHopOn, uint8_t HopPeriod, bool iqInverted,
                       bool rxContinuous );

/*!
 * \brief Sets the transmission parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev, uint32_t bandwidth, uint32_t datarate,
                       uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool FreqHopOn,
                       uint8_t HopPeriod, bool iqInverted, uint32_t timeout );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool RadioCheckRfFrequency( uint32_t frequency );

/*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
uint32_t RadioTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn );

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void RadioSend( uint8_t* buffer, uint8_t size );

/*!
 * \brief Sets the radio in sleep mode
 */
void RadioSleep( void );

/*!
 * \brief Sets the radio in standby mode
 */
void RadioStandby( void );

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRx( uint32_t timeout );

/*!
 * \brief Start a Channel Activity Detection
 */
void RadioStartCad( void );

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time );

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t RadioRssi( RadioModems_t modem );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void RadioWrite( uint32_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
uint8_t RadioRead( uint32_t addr );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void RadioWriteBuffer( uint32_t addr, uint8_t* buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void RadioReadBuffer( uint32_t addr, uint8_t* buffer, uint8_t size );

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max );

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
void RadioSetPublicNetwork( bool enable );

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
uint32_t RadioGetWakeupTime( void );

/*!
 * \brief Process radio irq
 */
void RadioIrqProcess( void );

/*!
 * \brief Sets the radio in reception mode with Max LNA gain for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRxBoosted( uint32_t timeout );

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime );

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio = {
    RadioInit,
    RadioGetStatus,
    RadioSetModem,
    RadioSetChannel,
    RadioIsChannelFree,
    RadioRandom,
    RadioSetRxConfig,
    RadioSetTxConfig,
    RadioCheckRfFrequency,
    RadioTimeOnAir,
    RadioSend,
    RadioSleep,
    RadioStandby,
    RadioRx,
    RadioStartCad,
    RadioSetTxContinuousWave,
    RadioRssi,
    RadioWrite,
    RadioRead,
    RadioWriteBuffer,
    RadioReadBuffer,
    RadioSetMaxPayloadLength,
    RadioSetPublicNetwork,
    RadioGetWakeupTime,
    RadioIrqProcess,
    // Available on LR1110 only
    RadioRxBoosted,
    RadioSetRxDutyCycle,
};

/*
 * Local types definition
 */

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
} FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] = {
    { 4800, 0x1F },   { 5800, 0x17 },   { 7300, 0x0F },   { 9700, 0x1E },   { 11700, 0x16 },  { 14600, 0x0E },
    { 19500, 0x1D },  { 23400, 0x15 },  { 29300, 0x0D },  { 39000, 0x1C },  { 46900, 0x14 },  { 58600, 0x0C },
    { 78200, 0x1B },  { 93800, 0x13 },  { 117300, 0x0B }, { 156200, 0x1A }, { 187200, 0x12 }, { 234300, 0x0A },
    { 312000, 0x19 }, { 373600, 0x11 }, { 467000, 0x09 }, { 500000, 0x00 },  // Invalid Bandwidth
};

const lr1110_radio_lora_bw_t Bandwidths[] = { LR1110_RADIO_LORA_BW_125, LR1110_RADIO_LORA_BW_250,
                                              LR1110_RADIO_LORA_BW_500 };

uint8_t MaxPayloadLength = 0xFF;

uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;

bool RxContinuous = false;

lr1110_radio_pkt_status_lora_t lora_packet_status;
lr1110_radio_pkt_status_gfsk_t gfsk_packet_status;
uint8_t                           RadioRxPayload[255];

bool IrqFired = false;

/*
 * LR1110 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void RadioOnDioIrq( void* context );

/*!
 * \brief Tx timeout timer callback
 */
void RadioOnTxTimeoutIrq( void* context );

/*!
 * \brief Rx timeout timer callback
 */
void RadioOnRxTimeoutIrq( void* context );

/*
 * Private global variables
 */

/*!
 * Holds the current network type for the radio
 */
typedef struct
{
    bool Previous;
    bool Current;
} RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

/*!
 * Radio callbacks variable
 */
static RadioEvents_t* RadioEvents;

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
lr1110_t LR1110;

/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    if( bandwidth == 0 )
    {
        return ( 0x1F );
    }

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i + 1].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 )
        ;
}

void RadioInit( RadioEvents_t* events )
{
    RadioEvents = events;

    lr1110_board_init( &LR1110, RadioOnDioIrq );

    lr1110_system_set_standby( &LR1110, LR1110_SYSTEM_STANDBY_CFG_RC );
    lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_STDBY_RC );

    lr1110_system_set_reg_mode( &LR1110, LR1110_SYSTEM_REG_MODE_DCDC );

    lr1110_radio_set_tx_params( &LR1110, 0, LR1110_RADIO_RAMP_208_US );
    lr1110_system_set_dio_irq_params( &LR1110, LR1110_SYSTEM_IRQ_ALL_MASK, LR1110_SYSTEM_IRQ_NONE );

    // Initialize driver timeout timers
    TimerInit( &TxTimeoutTimer, RadioOnTxTimeoutIrq );
    TimerInit( &RxTimeoutTimer, RadioOnRxTimeoutIrq );

    IrqFired = false;
}

RadioState_t RadioGetStatus( void )
{
    switch( lr1110_hal_get_operating_mode( &LR1110 ) )
    {
    case LR1110_HAL_OP_MODE_TX:
        return RF_TX_RUNNING;
    case LR1110_HAL_OP_MODE_RX:
    case LR1110_HAL_OP_MODE_RX_C:
    case LR1110_HAL_OP_MODE_RX_DC:
        return RF_RX_RUNNING;
    case LR1110_HAL_OP_MODE_CAD:
        return RF_CAD;
    default:
        return RF_IDLE;
    }
}

void RadioSetModem( RadioModems_t modem )
{
    switch( modem )
    {
    default:
    case MODEM_FSK:
    	lr1110_radio_set_pkt_type( &LR1110, LR1110_RADIO_PKT_TYPE_GFSK );
        // When switching to GFSK mode the LoRa SyncWord register value is reset
        // Thus, we also reset the RadioPublicNetwork variable
        RadioPublicNetwork.Current = false;
        break;
    case MODEM_LORA:
        lr1110_radio_set_pkt_type( &LR1110, LR1110_RADIO_PKT_TYPE_LORA );
        // Public/Private network register is reset when switching modems
        if( RadioPublicNetwork.Current != RadioPublicNetwork.Previous )
        {
            RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
            RadioSetPublicNetwork( RadioPublicNetwork.Current );
        }
        break;
    }
}

void RadioSetChannel( uint32_t freq )
{
    lr1110_radio_set_rf_freq( &LR1110, freq );
}

bool RadioIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool     status           = true;
    int16_t  rssi             = 0;
    uint32_t carrierSenseTime = 0;

    RadioSetModem( MODEM_FSK );

    RadioSetChannel( freq );

    // Set Rx bandwidth. Other parameters are not used.
    RadioSetRxConfig( MODEM_FSK, rxBandwidth, 600, 0, rxBandwidth, 3, 0, false,
                      0, false, 0, 0, false, true );
    RadioRx( 0 );

    DelayMs( 1 );

    carrierSenseTime = TimerGetCurrentTime( );

    // Perform carrier sense for maxCarrierSenseTime
    while( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime )
    {
        rssi = RadioRssi( MODEM_FSK );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }
    RadioSleep( );
    return status;
}

uint32_t RadioRandom( void )
{
    uint32_t rnd  = 0;

    RadioStandby( );

    lr1110_system_get_random_number( &LR1110, &rnd );

    return rnd;
}

void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
                       uint32_t bandwidthAfc, uint16_t preambleLen, uint16_t symbTimeout, bool fixLen,
                       uint8_t payloadLen, bool crcOn, bool freqHopOn, uint8_t hopPeriod, bool iqInverted,
                       bool rxContinuous )
{
    RxContinuous = rxContinuous;
    if( rxContinuous == true )
    {
        symbTimeout = 0;
    }
    if( fixLen == true )
    {
        MaxPayloadLength = payloadLen;
    }
    else
    {
        MaxPayloadLength = 0xFF;
    }

    switch( modem )
    {
    case MODEM_FSK:
        lr1110_radio_stop_timeout_on_preamble( &LR1110, false );
        LR1110.modulation_params.packet_type = LR1110_RADIO_PKT_TYPE_GFSK;

        LR1110.modulation_params.modulation.gfsk.br_in_bps     = datarate;
        LR1110.modulation_params.modulation.gfsk.pulse_shape = LR1110_RADIO_GFSK_PULSE_SHAPE_BT_1;
        LR1110.modulation_params.modulation.gfsk.bw_dsb_param =
            ( lr1110_radio_gfsk_bw_t ) RadioGetFskBandwidthRegValue( bandwidth << 1 ); // LR1110 badwidth is double sided

        LR1110.packet_params.packet_type                     = LR1110_RADIO_PKT_TYPE_GFSK;
        LR1110.packet_params.packet.gfsk.preamble_len_in_bits = ( preambleLen << 3 );  // convert byte into bit
        LR1110.packet_params.packet.gfsk.preamble_detector    = LR1110_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
        LR1110.packet_params.packet.gfsk.sync_word_len_in_bits = 3 << 3;  // convert byte into bit
        LR1110.packet_params.packet.gfsk.address_filtering         = LR1110_RADIO_GFSK_ADDRESS_FILTERING_DISABLE;
        LR1110.packet_params.packet.gfsk.header_type =
            ( fixLen == true ) ? LR1110_RADIO_GFSK_PKT_FIX_LEN : LR1110_RADIO_GFSK_PKT_VAR_LEN;
        LR1110.packet_params.packet.gfsk.pld_len_in_bytes = MaxPayloadLength;
        if( crcOn == true )
        {
            LR1110.packet_params.packet.gfsk.crc_type = LR1110_RADIO_GFSK_CRC_2_BYTES_INV;
        }
        else
        {
            LR1110.packet_params.packet.gfsk.crc_type = LR1110_RADIO_GFSK_CRC_OFF;
        }
        LR1110.packet_params.packet.gfsk.dc_free = LR1110_RADIO_GFSK_DC_FREE_WHITENING;

        RadioStandby( );
        RadioSetModem( ( LR1110.modulation_params.packet_type == LR1110_RADIO_PKT_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
        lr1110_radio_set_gfsk_mod_params( &LR1110, &LR1110.modulation_params.modulation.gfsk );
        lr1110_radio_set_gfsk_pkt_params( &LR1110, &LR1110.packet_params.packet.gfsk );
        lr1110_radio_set_gfsk_sync_word( &LR1110, ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
        lr1110_radio_set_gfsk_crc_params( &LR1110, 0x1D0F, 0x1021 );
        lr1110_radio_set_gfsk_whitening_seed( &LR1110, 0x01FF );

        RxTimeout = ( uint32_t )symbTimeout * 8000UL / datarate;
        break;

    case MODEM_LORA:
        lr1110_radio_stop_timeout_on_preamble( &LR1110, false );
        lr1110_radio_set_lora_sync_timeout( &LR1110, symbTimeout );
        LR1110.modulation_params.packet_type            = LR1110_RADIO_PKT_TYPE_LORA;
        LR1110.modulation_params.modulation.lora.sf = ( lr1110_radio_lora_sf_t ) datarate;
        LR1110.modulation_params.modulation.lora.bw = Bandwidths[bandwidth];
        LR1110.modulation_params.modulation.lora.cr = ( lr1110_radio_lora_cr_t ) coderate;

        if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
        {
            LR1110.modulation_params.modulation.lora.ldro = 0x01;
        }
        else
        {
            LR1110.modulation_params.modulation.lora.ldro = 0x00;
        }

        LR1110.packet_params.packet_type = LR1110_RADIO_PKT_TYPE_LORA;

        if( ( LR1110.modulation_params.modulation.lora.sf == LR1110_RADIO_LORA_SF5 ) ||
            ( LR1110.modulation_params.modulation.lora.sf == LR1110_RADIO_LORA_SF6 ) )
        {
            if( preambleLen < 12 )
            {
                LR1110.packet_params.packet.lora.preamble_len_in_symb = 12;
            }
            else
            {
                LR1110.packet_params.packet.lora.preamble_len_in_symb = preambleLen;
            }
        }
        else
        {
            LR1110.packet_params.packet.lora.preamble_len_in_symb = preambleLen;
        }

        LR1110.packet_params.packet.lora.header_type = ( lr1110_radio_lora_pkt_len_modes_t ) fixLen;

        LR1110.packet_params.packet.lora.pld_len_in_bytes = MaxPayloadLength;
        LR1110.packet_params.packet.lora.crc                    = ( lr1110_radio_lora_crc_t ) crcOn;
        LR1110.packet_params.packet.lora.iq                     = ( lr1110_radio_lora_iq_t ) iqInverted;

        RadioSetModem( ( LR1110.modulation_params.packet_type == LR1110_RADIO_PKT_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
        lr1110_radio_set_lora_mod_params( &LR1110, &LR1110.modulation_params.modulation.lora );
        lr1110_radio_set_lora_pkt_params( &LR1110, &LR1110.packet_params.packet.lora );

        // Timeout Max, Timeout handled directly in SetRx function
        RxTimeout = 0xFFFF;

        break;
    }
}

void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev, uint32_t bandwidth, uint32_t datarate,
                       uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool freqHopOn,
                       uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    switch( modem )
    {
    case MODEM_FSK:
        LR1110.modulation_params.packet_type   = LR1110_RADIO_PKT_TYPE_GFSK;
        LR1110.modulation_params.modulation.gfsk.br_in_bps = datarate;

        LR1110.modulation_params.modulation.gfsk.pulse_shape = LR1110_RADIO_GFSK_PULSE_SHAPE_BT_1;
        LR1110.modulation_params.modulation.gfsk.bw_dsb_param =
            ( lr1110_radio_gfsk_bw_t ) RadioGetFskBandwidthRegValue( bandwidth << 1 ); // LR1110 badwidth is double sided
        LR1110.modulation_params.modulation.gfsk.fdev_in_hz = fdev;

        LR1110.packet_params.packet_type                     = LR1110_RADIO_PKT_TYPE_GFSK;
        LR1110.packet_params.packet.gfsk.preamble_len_in_bits = ( preambleLen << 3 );  // convert byte into bit
        LR1110.packet_params.packet.gfsk.preamble_detector           = LR1110_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
        LR1110.packet_params.packet.gfsk.sync_word_len_in_bits = 3 << 3;  // convert byte into bit
        LR1110.packet_params.packet.gfsk.address_filtering         = LR1110_RADIO_GFSK_ADDRESS_FILTERING_DISABLE;
        LR1110.packet_params.packet.gfsk.header_type =
            ( fixLen == true ) ? LR1110_RADIO_GFSK_PKT_FIX_LEN : LR1110_RADIO_GFSK_PKT_VAR_LEN;

        if( crcOn == true )
        {
            LR1110.packet_params.packet.gfsk.crc_type = LR1110_RADIO_GFSK_CRC_2_BYTES_INV;
        }
        else
        {
            LR1110.packet_params.packet.gfsk.crc_type = LR1110_RADIO_GFSK_CRC_OFF;
        }
        LR1110.packet_params.packet.gfsk.dc_free = LR1110_RADIO_GFSK_DC_FREE_WHITENING;

        RadioStandby( );
        RadioSetModem( ( LR1110.modulation_params.packet_type == LR1110_RADIO_PKT_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA );
        lr1110_radio_set_gfsk_mod_params( &LR1110, &LR1110.modulation_params.modulation.gfsk );
        lr1110_radio_set_gfsk_pkt_params( &LR1110, &LR1110.packet_params.packet.gfsk );
        lr1110_radio_set_gfsk_sync_word( &LR1110, ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
        lr1110_radio_set_gfsk_crc_params( &LR1110, 0x1D0F, 0x1021 );
        lr1110_radio_set_gfsk_whitening_seed( &LR1110, 0x01FF );
        break;

    case MODEM_LORA:
        LR1110.modulation_params.packet_type            = LR1110_RADIO_PKT_TYPE_LORA;
        LR1110.modulation_params.modulation.lora.sf = ( lr1110_radio_lora_sf_t ) datarate;
        LR1110.modulation_params.modulation.lora.bw        = Bandwidths[bandwidth];
        LR1110.modulation_params.modulation.lora.cr      = ( lr1110_radio_lora_cr_t ) coderate;

        if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
        {
            LR1110.modulation_params.modulation.lora.ldro = 0x01;
        }
        else
        {
            LR1110.modulation_params.modulation.lora.ldro = 0x00;
        }

        LR1110.packet_params.packet_type = LR1110_RADIO_PKT_TYPE_LORA;

        if( ( LR1110.modulation_params.modulation.lora.sf == LR1110_RADIO_LORA_SF5 ) ||
            ( LR1110.modulation_params.modulation.lora.sf == LR1110_RADIO_LORA_SF6 ) )
        {
            if( preambleLen < 12 )
            {
                LR1110.packet_params.packet.lora.preamble_len_in_symb = 12;
            }
            else
            {
                LR1110.packet_params.packet.lora.preamble_len_in_symb = preambleLen;
            }
        }
        else
        {
            LR1110.packet_params.packet.lora.preamble_len_in_symb = preambleLen;
        }

        LR1110.packet_params.packet.lora.header_type            = ( lr1110_radio_lora_pkt_len_modes_t ) fixLen;
        LR1110.packet_params.packet.lora.pld_len_in_bytes = MaxPayloadLength;
        LR1110.packet_params.packet.lora.crc                    = ( lr1110_radio_lora_crc_t ) crcOn;
        LR1110.packet_params.packet.lora.iq                     = ( lr1110_radio_lora_iq_t ) iqInverted;

        RadioStandby( );
        RadioSetModem( ( LR1110.modulation_params.packet_type == LR1110_RADIO_PKT_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
        lr1110_radio_set_lora_mod_params( &LR1110, &LR1110.modulation_params.modulation.lora );
        lr1110_radio_set_lora_pkt_params( &LR1110, &LR1110.packet_params.packet.lora );
        break;
    }

    lr1110_board_set_rf_tx_power( &LR1110, power );
    TxTimeout = timeout;
}

bool RadioCheckRfFrequency( uint32_t frequency )
{
    return true;
}

static uint32_t RadioGetLoRaBandwidthInHz( lr1110_radio_lora_bw_t bw )
{
    uint32_t bandwidthInHz = 0;

    switch( bw )
    {
    case LR1110_RADIO_LORA_BW_10:
        bandwidthInHz = 10417UL;
        break;
    case LR1110_RADIO_LORA_BW_15:
        bandwidthInHz = 15625UL;
        break;
    case LR1110_RADIO_LORA_BW_20:
        bandwidthInHz = 20833UL;
        break;
    case LR1110_RADIO_LORA_BW_31:
        bandwidthInHz = 31250UL;
        break;
    case LR1110_RADIO_LORA_BW_41:
        bandwidthInHz = 41667UL;
        break;
    case LR1110_RADIO_LORA_BW_62:
        bandwidthInHz = 62500UL;
        break;
    case LR1110_RADIO_LORA_BW_125:
        bandwidthInHz = 125000UL;
        break;
    case LR1110_RADIO_LORA_BW_250:
        bandwidthInHz = 250000UL;
        break;
    case LR1110_RADIO_LORA_BW_500:
        bandwidthInHz = 500000UL;
        break;
    }

    return bandwidthInHz;
}

static uint32_t RadioGetGfskTimeOnAirNumerator( uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    const uint8_t syncWordLength = 3;

    return ( preambleLen << 3 ) +
           ( ( fixLen == false ) ? 8 : 0 ) +
             ( syncWordLength << 3 ) +
             ( ( payloadLen +
               ( 0 ) +
               ( ( crcOn == true ) ? 2 : 0 ) 
               ) << 3 
             );
}

static uint32_t RadioGetLoRaTimeOnAirNumerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    int32_t crDenom           = coderate + 4;
    bool    lowDatareOptimize = false;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or
    // SF6
    if( ( datarate == 5 ) || ( datarate == 6 ) )
    {
        if( preambleLen < 12 )
        {
            preambleLen = 12;
        }
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
    {
        lowDatareOptimize = true;
    }

    int32_t ceilDenominator;
    int32_t ceilNumerator = ( payloadLen << 3 ) +
                            ( crcOn ? 16 : 0 ) -
                            ( 4 * datarate ) +
                            ( fixLen ? 0 : 20 );

    if( datarate <= 6 )
    {
        ceilDenominator = 4 * datarate;
    }
    else
    {
        ceilNumerator += 8;

        if( lowDatareOptimize == true )
        {
            ceilDenominator = 4 * ( datarate - 2 );
        }
        else
        {
            ceilDenominator = 4 * datarate;
        }
    }

    if( ceilNumerator < 0 )
    {
        ceilNumerator = 0;
    }

    // Perform integral ceil()
    int32_t intermediate =
        ( ( ceilNumerator + ceilDenominator - 1 ) / ceilDenominator ) * crDenom + preambleLen + 12;

    if( datarate <= 6 )
    {
        intermediate += 2;
    }

    return ( uint32_t )( ( 4 * intermediate + 1 ) * ( 1 << ( datarate - 2 ) ) );
}

uint32_t RadioTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    uint32_t numerator = 0;
    uint32_t denominator = 1;

    switch( modem )
    {
    case MODEM_FSK:
        {
            numerator   = 1000U * RadioGetGfskTimeOnAirNumerator( datarate, coderate,
                                                                  preambleLen, fixLen,
                                                                  payloadLen, crcOn );
            denominator = datarate;
        }
        break;
    case MODEM_LORA:
        {
            numerator   = 1000U * RadioGetLoRaTimeOnAirNumerator( bandwidth, datarate,
                                                                  coderate, preambleLen,
                                                                  fixLen, payloadLen, crcOn );
            denominator = RadioGetLoRaBandwidthInHz( Bandwidths[bandwidth] );
        }
        break;
    }
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

void RadioSend( uint8_t* buffer, uint8_t size )
{
    lr1110_radio_pkt_type_t packet_type;

    lr1110_system_set_dio_irq_params( &LR1110, LR1110_SYSTEM_IRQ_TX_DONE | LR1110_SYSTEM_IRQ_TIMEOUT,
                                      LR1110_SYSTEM_IRQ_NONE );

    lr1110_radio_get_pkt_type( &LR1110, &packet_type );
    if( packet_type == LR1110_RADIO_PKT_TYPE_LORA )
    {
        LR1110.packet_params.packet.lora.pld_len_in_bytes = size;
        lr1110_radio_set_lora_pkt_params( &LR1110, &LR1110.packet_params.packet.lora );
    }
    else
    {
        LR1110.packet_params.packet.gfsk.pld_len_in_bytes = size;
        lr1110_radio_set_gfsk_pkt_params( &LR1110, &LR1110.packet_params.packet.gfsk );
    }

    /* Send Payload */
    lr1110_regmem_write_buffer8( &LR1110, buffer, size );
    lr1110_radio_set_tx( &LR1110, 0 );
    lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_TX );

    TimerSetValue( &TxTimeoutTimer, TxTimeout );
    TimerStart( &TxTimeoutTimer );
}

void RadioSleep( void )
{
    lr1110_system_sleep_cfg_t sleep_config;

    sleep_config.is_warm_start  = 1;
    sleep_config.is_rtc_timeout = 0;

    lr1110_system_set_sleep( &LR1110, sleep_config, 0 );
    lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_SLEEP );

    DelayMs( 2 );
}

void RadioStandby( void )
{
    lr1110_system_set_standby( &LR1110, LR1110_SYSTEM_STANDBY_CFG_RC );
    lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_STDBY_RC );
}

void RadioRx( uint32_t timeout )
{
    lr1110_system_set_dio_irq_params(
        &LR1110,
        LR1110_SYSTEM_IRQ_ALL_MASK,  // LR1110_SYSTEM_IRQ_RXDONE_MASK | LR1110_SYSTEM_IRQ_TIMEOUT_MASK,
        LR1110_SYSTEM_IRQ_NONE );

    lr1110_radio_cfg_rx_boosted( &LR1110, false );

    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( RxContinuous == true )
    {
        lr1110_radio_set_rx( &LR1110, 0xFFFFFF );  // Rx Continuous
        lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_RX_C );
    }
    else
    {
        lr1110_radio_set_rx( &LR1110, ( RxTimeout * 32768 ) );
        lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_RX );
    }
}

void RadioRxBoosted( uint32_t timeout )
{
    lr1110_system_set_dio_irq_params(
        &LR1110,
        LR1110_SYSTEM_IRQ_ALL_MASK,  // LR1110_SYSTEM_IRQ_RXDONE_MASK | LR1110_SYSTEM_IRQ_TIMEOUT_MASK,
        LR1110_SYSTEM_IRQ_NONE );

    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    lr1110_radio_cfg_rx_boosted( &LR1110, true );
    if( RxContinuous == true )
    {
        lr1110_radio_set_rx( &LR1110, 0xFFFFFF );  // Rx Continuous
        lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_RX_C );
    }
    else
    {
        lr1110_radio_set_rx( &LR1110, ( RxTimeout * 32768 ) );
        lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_RX );
    }
}

void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
    lr1110_radio_set_rx_duty_cycle( &LR1110, rxTime, sleepTime, 0 );
    lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_RX_DC );
}

void RadioStartCad( void )
{
    lr1110_radio_set_cad( &LR1110 );
    lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_CAD );
}

void RadioTx( uint32_t timeout )
{
    lr1110_radio_set_tx( &LR1110, timeout * 32768 );
    lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_TX );
}

void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    uint32_t timeout = ( uint32_t )time * 1000;

    lr1110_radio_set_rf_freq( &LR1110, freq );
    lr1110_board_set_rf_tx_power( &LR1110, power );
    lr1110_radio_set_tx_cw( &LR1110 );
    lr1110_hal_set_operating_mode( &LR1110, LR1110_HAL_OP_MODE_TX );

    TimerSetValue( &TxTimeoutTimer, timeout );
    TimerStart( &TxTimeoutTimer );
}

int16_t RadioRssi( RadioModems_t modem )
{
    int8_t rssi = 0;

    lr1110_radio_get_rssi_inst( &LR1110, &rssi );

    return rssi;
}

void RadioWrite( uint32_t addr, uint8_t data )
{
    lr1110_regmem_write_mem8( &LR1110, addr, &data, 1 );
}

uint8_t RadioRead( uint32_t addr )
{
    uint8_t data = 0;

    lr1110_regmem_read_mem8( &LR1110, addr, &data, 1 );

    return data;
}

void RadioWriteBuffer( uint32_t addr, uint8_t* buffer, uint8_t size )
{
    lr1110_regmem_write_buffer8( &LR1110, buffer, size );
}

void RadioReadBuffer( uint32_t addr, uint8_t* buffer, uint8_t size )
{
    lr1110_regmem_read_buffer8( &LR1110, buffer, addr, size );
}

void RadioWriteFifo( uint8_t* buffer, uint8_t size )
{
    lr1110_regmem_write_buffer8( &LR1110, buffer, size );
}

void RadioReadFifo( uint8_t* buffer, uint8_t offset, uint8_t size )
{
    lr1110_regmem_read_buffer8( &LR1110, buffer, offset, size );
}

void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    if( modem == MODEM_LORA )
    {
        LR1110.packet_params.packet.lora.pld_len_in_bytes = MaxPayloadLength = max;
        lr1110_radio_set_lora_pkt_params( &LR1110, &LR1110.packet_params.packet.lora );
    }
    else
    {
        if( LR1110.packet_params.packet.gfsk.header_type == LR1110_RADIO_GFSK_PKT_VAR_LEN )
        {
            LR1110.packet_params.packet.gfsk.pld_len_in_bytes = MaxPayloadLength = max;
            lr1110_radio_set_gfsk_pkt_params( &LR1110, &LR1110.packet_params.packet.gfsk );
        }
    }
}

void RadioSetPublicNetwork( bool enable )
{
    RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

    RadioSetModem( MODEM_LORA );
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        lr1110_radio_set_lora_sync_word( &LR1110, LR1110_RADIO_LORA_NETWORK_PUBLIC );
    }
    else
    {
        // Change LoRa modem SyncWord
        lr1110_radio_set_lora_sync_word( &LR1110, LR1110_RADIO_LORA_NETWORK_PRIVATE );
    }
}

uint32_t RadioGetWakeupTime( void )
{
    return lr1110_board_get_tcxo_wakeup_time( &LR1110 ) + 3;
}

void RadioOnTxTimeoutIrq( void* context )
{
    if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
    {
        RadioEvents->TxTimeout( );
    }
}

void RadioOnRxTimeoutIrq( void* context )
{
    if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
    {
        RadioEvents->RxTimeout( );
    }
}

void RadioOnDioIrq( void* context )
{
    IrqFired = true;
}

/*!
 * \brief Callback - handle the interrupt get & clear
 *
 * This function shall be called each time there is an interrupt coming from the
 * radio. It is responsible for the update of the operating mode.
 *
 * \param [in] radio Radio abstraction
 *
 * \param [out] irq  Pointer to the interrupt field returned to the caller
 *
 * \see lr1110_system_get_status, lr1110_system_clear_irq
 */
static void lr1110_system_irq_process( const void* radio, uint32_t* irq )
{
    lr1110_system_stat1_t stat1;
    lr1110_system_stat2_t stat2;

    lr1110_hal_operating_mode_t op_mode = lr1110_hal_get_operating_mode( radio );

    lr1110_system_get_status( radio, &stat1, &stat2, irq );
    lr1110_system_clear_irq_status( radio, *irq );

    if( ( ( *irq & LR1110_SYSTEM_IRQ_TX_DONE ) != 0 ) || ( ( *irq & LR1110_SYSTEM_IRQ_CAD_DONE ) != 0 ) ||
        ( ( *irq & LR1110_SYSTEM_IRQ_TIMEOUT ) != 0 ) )
    {
        lr1110_hal_set_operating_mode( radio, LR1110_HAL_OP_MODE_STDBY_RC );
    }

    if( ( ( *irq & LR1110_SYSTEM_IRQ_HEADER_ERROR ) != 0 ) || ( ( *irq & LR1110_SYSTEM_IRQ_RX_DONE ) != 0 ) ||
        ( ( *irq & LR1110_SYSTEM_IRQ_CRC_ERROR ) != 0 ) )
    {
        if( op_mode != LR1110_HAL_OP_MODE_RX_C )
        {
            lr1110_hal_set_operating_mode( radio, LR1110_HAL_OP_MODE_STDBY_RC );
        }
    }
}

void RadioIrqProcess( void )
{
    if( IrqFired == true )
    {
        CRITICAL_SECTION_BEGIN( );
        // Clear IRQ flag
        IrqFired = false;
        CRITICAL_SECTION_END( );

        uint32_t irqRegs;
        // Get Status
        lr1110_system_irq_process( &LR1110, &irqRegs );

        if( ( irqRegs & LR1110_SYSTEM_IRQ_TX_DONE ) == LR1110_SYSTEM_IRQ_TX_DONE )
        {
            TimerStop( &TxTimeoutTimer );
            if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
            {
                RadioEvents->TxDone( );
            }
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_RX_DONE ) == LR1110_SYSTEM_IRQ_RX_DONE )
        {
            lr1110_radio_pkt_type_t    packet_type;
            lr1110_radio_rx_buffer_status_t rxbuffer_status;

            TimerStop( &RxTimeoutTimer );

            lr1110_radio_get_rx_buffer_status( &LR1110, &rxbuffer_status );
            lr1110_regmem_read_buffer8( &LR1110, RadioRxPayload, rxbuffer_status.buffer_start_pointer,
                                        rxbuffer_status.pld_len_in_bytes );

            lr1110_radio_get_pkt_type( &LR1110, &packet_type );
            if( packet_type == LR1110_RADIO_PKT_TYPE_LORA )
            {
                lr1110_radio_get_lora_pkt_status( &LR1110, &lora_packet_status );
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RadioRxPayload, rxbuffer_status.pld_len_in_bytes,
                                         lora_packet_status.rssi_pkt_in_dbm, lora_packet_status.snr_pkt_in_db);
                }
            }
            else
            {
                lr1110_radio_get_gfsk_pkt_status( &LR1110, &gfsk_packet_status );
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RadioRxPayload, rxbuffer_status.pld_len_in_bytes,
                                         gfsk_packet_status.rssi_avg_in_dbm, 0 );
                }
            }
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_CRC_ERROR ) == LR1110_SYSTEM_IRQ_CRC_ERROR )
        {
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
            {
                RadioEvents->RxError( );
            }
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_CAD_DONE) == LR1110_SYSTEM_IRQ_CAD_DONE )
        {
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone(
                    ( ( irqRegs & LR1110_SYSTEM_IRQ_CAD_DETECTED) == LR1110_SYSTEM_IRQ_CAD_DETECTED ) );
            }
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_TIMEOUT ) == LR1110_SYSTEM_IRQ_TIMEOUT )
        {
            TimerStop( &RxTimeoutTimer );
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
            {
                RadioEvents->RxTimeout( );
            }
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_PREAMBLE_DETECTED ) == LR1110_SYSTEM_IRQ_PREAMBLE_DETECTED )
        {
            //__NOP( );
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID ) == LR1110_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID )
        {
            //__NOP( );
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_HEADER_ERROR) == LR1110_SYSTEM_IRQ_HEADER_ERROR )
        {
            TimerStop( &RxTimeoutTimer );
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
            {
                RadioEvents->RxTimeout( );
            }
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_GNSS_SCAN_DONE) == LR1110_SYSTEM_IRQ_GNSS_SCAN_DONE )
        {
            if( ( RadioEvents != NULL ) && ( RadioEvents->GnssDone != NULL ) )
            {
                RadioEvents->GnssDone( );
            }
        }

        if( ( irqRegs & LR1110_SYSTEM_IRQ_WIFI_SCAN_DONE ) == LR1110_SYSTEM_IRQ_WIFI_SCAN_DONE )
        {
            if( ( RadioEvents != NULL ) && ( RadioEvents->WifiDone != NULL ) )
            {
                RadioEvents->WifiDone( );
            }
        }
    }
}
