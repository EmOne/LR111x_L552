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
#include "gtzc.h"
#include "i2c.h"
#include "icache.h"
#include "iwdg.h"
#include "usart.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "ucpd.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l552e_eval.h"
#include "lr1110-board.h"
#include "lr1110_wifi.h"
#include "lr1110_gnss.h"
#include "lr1110_bootloader.h"
#include "lr1110_driver_version.h"

#include "radio.h"
#include "stdio.h"
#include "stdbool.h"

#include "RegionCommon.h"

#include "cli.h"
#include "Commissioning.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "CayenneLpp.h"
#include "LmHandlerMsgDisplay.h"
#include "firmwareVersion.h"
#include "githubVersion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TX_CW	1
//#define RX_SENSE	1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * LoRaWAN default end-device class
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000 * 6

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_2

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

/*!
 * LoRaWAN application port
 * @remark The allowed port range is from 1 up to 223. Other values are reserved.
 */
#define LORAWAN_APP_PORT                            2

/*!
 *
 */
typedef enum
{
    LORAMAC_HANDLER_TX_ON_TIMER,
    LORAMAC_HANDLER_TX_ON_EVENT,
}LmHandlerTxEvents_t;

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*!
 * User application data structure
 */
static LmHandlerAppData_t AppData =
{
    .Buffer = AppDataBuffer,
    .BufferSize = 0,
    .Port = 0,
};

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;
volatile bool Led1TimerEvent = false;
/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;
volatile bool Led2TimerEvent = false;

/*!
 * Timer to handle the state of LED beacon indicator
 */
static TimerEvent_t LedBeaconTimer;
volatile bool IsBeacon = false;

static TimerEvent_t WifiTimer;

static TimerEvent_t GnssTimer;
extern lr1110_t LR1110;

#if defined( TX_CW ) || defined( RX_SENSE )
#define RF_FREQUENCY                                923000000 // Hz
#define TX_OUTPUT_POWER                             20        // 14 dBm
#define TX_TIMEOUT									100
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;
void OnRadioTxTimeout( void );
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       10        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
void OnRadioRxDone( uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRadioRxTimeout( void );
static int16_t iRssi;
#endif /* TX_CW */

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * Flag used to indicate if board is powered from the USB
 */
static bool UsbIsConnected = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SecureFault_Callback(void);
void SecureError_Callback(void);
static void OnMacProcessNotify( void );
static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size );
static void OnNetworkParametersChange( CommissioningParams_t* params );
static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn );
static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn );
static void OnJoinRequest( LmHandlerJoinParams_t* params );
static void OnTxData( LmHandlerTxParams_t* params );
static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params );
static void OnClassChange( DeviceClass_t deviceClass );
static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params );
#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection );
#else
static void OnSysTimeUpdate( void );
#endif
static void PrepareTxFrame( void );
static void StartTxProcess( LmHandlerTxEvents_t txEvent );
static void UplinkProcess( void );

static void OnTxPeriodicityChanged( uint32_t periodicity );
static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed );
static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity );

static void OnRadioWifiDone( void );
static void OnRadioGnssDone( void );

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context );

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void* context );

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void* context );

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent( void* context );

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * \brief Function executed on Gnss timer Timeout event
 */
static void OnGnssTimerEvent(  void* context  );

/*!
 * \brief Function executed on Wifi timer Timeout event
 */
static void OnWifiTimerEvent(  void* context  );

static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = BoardGetBatteryLevel,
    .GetTemperature = BoardGetTemperature,
    .GetRandomSeed = BoardGetRandomSeed,
    .OnMacProcess = OnMacProcessNotify,
    .OnNvmDataChange = OnNvmDataChange,
    .OnNetworkParametersChange = OnNetworkParametersChange,
    .OnMacMcpsRequest = OnMacMcpsRequest,
    .OnMacMlmeRequest = OnMacMlmeRequest,
    .OnJoinRequest = OnJoinRequest,
    .OnTxData = OnTxData,
    .OnRxData = OnRxData,
    .OnClassChange= OnClassChange,
    .OnBeaconStatusChange = OnBeaconStatusChange,
    .OnSysTimeUpdate = OnSysTimeUpdate,
	.OnWifiScanMAC = OnRadioWifiDone,
	.OnGnssScan = OnRadioGnssDone
};

static LmHandlerParams_t LmHandlerParams =
{
    .Region = ACTIVE_REGION,
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = AppDataBuffer,
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};

static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion.Value = FIRMWARE_VERSION,
    .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};

/*!
 * Indicates if LoRaMacProcess call is pending.
 *
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile uint8_t IsTxFramePending = 0;

static volatile uint32_t TxPeriodicity = 0;

/*!
 * Used to measure and calibrate the system wake-up time from STOP mode
 */
static void CalibrateSystemWakeupTime( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );
/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static volatile bool SystemWakeupTimeCalibrated = false;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent( void* context )
{
    RtcSetMcuWakeUpTime( );
    SystemWakeupTimeCalibrated = true;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float pfData[3];
static int32_t iddValue[2];
static float meters;
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

  /* GTZC initialisation */
  MX_GTZC_NS_Init();

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
  MX_UCPD1_Init();
  MX_USB_Device_Init();
  MX_USART2_UART_Init();
  MX_ICACHE_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  BoardInitMcu();

  TimerInit( &Led1Timer, OnLed1TimerEvent );
  TimerSetValue( &Led1Timer, 25 );

  TimerInit( &Led2Timer, OnLed2TimerEvent );
  TimerSetValue( &Led2Timer, 25 );

  TimerInit( &LedBeaconTimer, OnLedBeaconTimerEvent );
  TimerSetValue( &LedBeaconTimer, 5000 );
   TimerStart(&LedBeaconTimer);

    TimerInit(&WifiTimer, OnWifiTimerEvent);
	TimerSetValue(&WifiTimer, 90 * 1000);
	TimerStart(&WifiTimer);
	TimerInit(&GnssTimer, OnGnssTimerEvent);
	TimerSetValue(&GnssTimer, 60 * 1000);
	TimerStart(&GnssTimer);

#ifdef TX_CW
  RadioEvents.TxTimeout = OnRadioTxTimeout;
  RadioEvents.TxDone = OnRadioTxTimeout;
  Radio.Init( &RadioEvents );

  Radio.SetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
  TimerSetValue( &Led1Timer, 500 );
  TimerSetValue( &Led2Timer, 200 );
  TimerStart( &Led1Timer );
#elif RX_SENSE
  // Radio initialization
      RadioEvents.RxDone = OnRadioRxDone;
      RadioEvents.RxTimeout = OnRadioRxTimeout;
      RadioEvents.RxError = OnRadioRxTimeout;
      RadioEvents.WifiDone = OnRadioWifiDone;
      RadioEvents.GnssDone = OnRadioGnssDone;

      Radio.Init( &RadioEvents );

      Radio.SetChannel( RF_FREQUENCY );

      Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                         0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

      Radio.Rx( 0 ); // Continuous Rx
      TimerStart( &LedBeaconTimer );
#else
  // Initialize transmission periodicity variable
  TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );

  const Version_t appVersion = { .Value = FIRMWARE_VERSION };
  const Version_t gitHubVersion = { .Value = GITHUB_VERSION };
  DisplayAppInfo( "periodic-uplink-lpp",
				  &appVersion,
				  &gitHubVersion );

    if ( LmHandlerInit( &LmHandlerCallbacks, &LmHandlerParams ) != LORAMAC_HANDLER_SUCCESS )
    {
  	  printf( "LoRaMac wasn't properly initialized\n" );
  	  // Fatal error, endless loop.
  	  Error_Handler();
    }

  // Set system maximum tolerated rx error in milliseconds
  LmHandlerSetSystemMaxRxError( 20 );

  // The LoRa-Alliance Compliance protocol package should always be
  // initialized and activated.
  LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams );

  LmHandlerJoin( );

  StartTxProcess( LORAMAC_HANDLER_TX_ON_TIMER );
#endif /* TX_CW */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if defined( TX_CW ) || defined( RX_SENSE )
	  // Process Radio IRQ
	         if( Radio.IrqProcess != NULL )
	         {
	             Radio.IrqProcess( );
	         }
	         if( Led1TimerEvent == true )
	         {
	             Led1TimerEvent = false;
#ifdef VERSION_020
	             // Switch LED 1 OFF
	             SECURE_LED_RED( 1 );
	             // Switch LED 2 ON
	             SECURE_LED_YELLOW( 0 );
#else

#endif

	             TimerStart( &Led2Timer );
	         }

	         if( Led2TimerEvent == true )
	         {
	             Led2TimerEvent = false;
#ifdef VERSION_020
	             // Switch LED 2 OFF
	             SECURE_LED_YELLOW( 1 );
	             // Switch LED 1 ON
	             SECURE_LED_RED( 0 );
#else
#endif

	             TimerStart( &Led1Timer );
	         }
#else
		// Process characters sent over the command line interface
		CliProcess(&hlpuart1);

		// Processes the LoRaMac events
		LmHandlerProcess();

		// Process application uplinks management
		UplinkProcess();

		CRITICAL_SECTION_BEGIN();
		if (IsMacProcessPending == 1) {
			// Clear flag and prevent MCU to go into low power modes.
			IsMacProcessPending = 0;
		} else {
			// The MCU wakes up through events
			BoardLowPowerHandler();
		}
//		HAL_IWDG_Refresh(&hiwdg);
		CRITICAL_SECTION_END();
#endif /* TX_CW */
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


static void OnMacProcessNotify( void )
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size )
{
    DisplayNvmDataChange( state, size );
}

static void OnNetworkParametersChange( CommissioningParams_t* params )
{
    DisplayNetworkParametersUpdate( params );
}

static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn )
{
    DisplayMacMcpsRequestUpdate( status, mcpsReq, nextTxIn );
}

static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn )
{
    DisplayMacMlmeRequestUpdate( status, mlmeReq, nextTxIn );
}

static void OnJoinRequest( LmHandlerJoinParams_t* params )
{
    DisplayJoinRequestUpdate( params );
    if( params->Status == LORAMAC_HANDLER_ERROR )
    {
        LmHandlerJoin( );
    }
    else
    {
        LmHandlerRequestClass( LORAWAN_DEFAULT_CLASS );
    }
}

static void OnTxData( LmHandlerTxParams_t* params )
{
    DisplayTxUpdate( params );
}

static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params )
{
    DisplayRxUpdate( appData, params );

    switch( appData->Port )
    {
    case 1: // The application LED can be controlled on port 1 or 2
    case LORAWAN_APP_PORT:
        {
            AppLedStateOn = appData->Buffer[0] & 0x01;
        }
        break;
    case 0xAA:
    	LR1110.gnss.pos.latitude = ((int16_t) appData->Buffer[1] & 0x0F) << 8 | (int16_t) appData->Buffer[0];
    	LR1110.gnss.pos.longitude = ((int16_t) appData->Buffer[2] << 4) | ((int16_t)appData->Buffer[1] & 0xF0) >> 4;
    	lr1110_gnss_set_assistance_position(&LR1110, &LR1110.gnss.pos);
    	break;
    case 0X63:
    	BoardResetMcu();
    	break;
    default:
        break;
    }

    // Switch LED 2 ON for each received downlink
//    GpioWrite( &Led2, 1 );
#ifdef VERSION_020
    SECURE_LED_RED(true);
#else
#endif
    TimerStart( &Led2Timer );
}

static void OnClassChange( DeviceClass_t deviceClass )
{
    DisplayClassUpdate( deviceClass );

    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
    {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0,
    };
    LmHandlerSend( &appData, LORAMAC_HANDLER_UNCONFIRMED_MSG );
}

static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params )
{
    switch( params->State )
    {
        case LORAMAC_HANDLER_BEACON_RX:
        {
            TimerStart( &LedBeaconTimer );
            break;
        }
        case LORAMAC_HANDLER_BEACON_LOST:
        case LORAMAC_HANDLER_BEACON_NRX:
        {
            TimerStop( &LedBeaconTimer );
            break;
        }
        default:
        {
            break;
        }
    }

    DisplayBeaconUpdate( params );
}

#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection )
{

}
#else
static void OnSysTimeUpdate( void )
{

}
#endif

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame( void )
{
    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    BSP_GYRO_GetXYZ(pfData);
    BSP_GYRO_LowPower();
	BSP_IDD_StartMeasurement(0);
	BSP_IDD_GetValue(0, (uint32_t*) &iddValue[0]);
	BSP_IDD_GetValue(1, (uint32_t*) &iddValue[1]);

	lr1110_gnss_read_assistance_position(&LR1110, &LR1110.gnss.pos);

    uint8_t channel = 0;

    AppData.Port = LORAWAN_APP_PORT;

    CayenneLppReset( );
    CayenneLppAddDigitalInput( channel++, AppLedStateOn );
    CayenneLppAddAnalogInput( channel++, BoardGetBatteryLevel( ) * 100 / 254 );
    CayenneLppAddAnalogInput( channel++, iddValue[0]);
    CayenneLppAddGyrometer(channel++, pfData[0], pfData[1], pfData[2]);
    CayenneLppAddGps(channel++, LR1110.gnss.pos.latitude, LR1110.gnss.pos.longitude, meters);
    CayenneLppCopy( AppData.Buffer );
    AppData.BufferSize = CayenneLppGetSize( );

    if( LmHandlerSend( &AppData, LmHandlerParams.IsTxConfirmed ) == LORAMAC_HANDLER_SUCCESS )
    {
#ifdef VERSION_020
        // Switch LED 1 ON
//        GpioWrite( &Led1, 1 );
        SECURE_LED_YELLOW(false);
#else
#endif
        TimerStart( &Led1Timer );
    }
}

static void StartTxProcess( LmHandlerTxEvents_t txEvent )
{
    switch( txEvent )
    {
    default:
        // Intentional fall through
    case LORAMAC_HANDLER_TX_ON_TIMER:
        {
            // Schedule 1st packet transmission
            TimerInit( &TxTimer, OnTxTimerEvent );
            TimerSetValue( &TxTimer, TxPeriodicity );
            OnTxTimerEvent( NULL );
        }
        break;
    case LORAMAC_HANDLER_TX_ON_EVENT:
        {
        }
        break;
    }
}

static void UplinkProcess( void )
{
    uint8_t isPending = 0;
    CRITICAL_SECTION_BEGIN( );
    isPending = IsTxFramePending;
    IsTxFramePending = 0;
    CRITICAL_SECTION_END( );
    if( isPending == 1 )
    {
        PrepareTxFrame( );
    }
}

static void OnTxPeriodicityChanged( uint32_t periodicity )
{
    TxPeriodicity = periodicity;

    if( TxPeriodicity == 0 )
    { // Revert to application default periodicity
        TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
    }
}

static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed )
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity )
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context )
{
    TimerStop( &TxTimer );

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void* context )
{
	Led1TimerEvent = true;
#ifdef TX_CW

#else
    TimerStop( &Led1Timer );
#ifdef VERSION_020
    // Switch LED 1 OFF

    SECURE_LED_YELLOW(GPIO_PIN_RESET);

    if(IsBeacon){
    	RtcDelayMs(150);
    	IsBeacon = false;
    	SECURE_LED_YELLOW(GPIO_PIN_SET);
    	TimerStart( &Led1Timer );
    }

#else
#endif
#endif
}

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void* context )
{
	Led2TimerEvent = true;
#ifdef TX_CW

#else
    TimerStop( &Led2Timer );
#ifdef VERSION_020
    // Switch LED 2 OFF
//    GpioWrite( &Led2, 0 );
    SECURE_LED_RED(GPIO_PIN_RESET);
#else

#endif
#endif
}

#ifdef TX_CW
/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnRadioTxTimeout( void )
{
    // Restarts continuous wave transmission when timeout expires
    Radio.SetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
}
#elif defined( RX_SENSE )
void OnRadioRxTimeout( void )
{
	Led1TimerEvent = true;
#ifdef VERSION_020
	SECURE_LEDToggle_RED();
#else
#endif
	Radio.Rx( 0 );
}

void OnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	Led2TimerEvent = true;
#ifdef VERSION_020
	SECURE_LEDToggle_YELLOW();
#else
    if( size != 0 )
    {
        printf( "RX DATA     : \n" );
        PrintHexBuffer( payload, size );
    }

    printf( "\n" );
    printf( "RX RSSI     : %d\n", rssi );
    printf( "RX SNR      : %d\n", snr );
#endif
	Radio.Rx( 0 );
}


#endif /* TX_CW */

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent( void* context )
{
//    GpioWrite( &Led2, 1 );
#ifdef VERSION_020
	SECURE_LED_YELLOW(GPIO_PIN_SET);
#else
#endif

    IsBeacon = true;

    TimerStart( &Led1Timer );

    TimerStart( &LedBeaconTimer );
}

static void OnGnssTimerEvent(  void* context  )
{
	SECURE_LED_YELLOW(GPIO_PIN_SET);

	TimerStart( &Led1Timer );

	TimerStop( &TxTimer );

	TimerStop( &GnssTimer );

	lr1110_system_stat1_t stat1;
	lr1110_system_stat2_t stat2;
	uint32_t              irq = 0;
	lr1110_system_get_status(&LR1110, &stat1, &stat2, &irq);

	lr1110_system_errors_t errors = { 0 };
	lr1110_system_get_errors(&LR1110, &errors);
	lr1110_system_clear_errors(&LR1110);
	lr1110_system_get_and_clear_irq_status(&LR1110, &irq);
	LR1110.gnss.delay = 50;
	LR1110.gnss.pos.latitude = 13.798088073124758;
	LR1110.gnss.pos.longitude = 100.60763602927065;
	lr1110_gnss_set_assistance_position(&LR1110, &LR1110.gnss.pos);
	lr1110_gnss_set_constellations_to_use(&LR1110, LR1110_GNSS_GPS_MASK | LR1110_GNSS_BEIDOU_MASK);
	lr1110_gnss_set_scan_mode(&LR1110, LR1110_GNSS_DOUBLE_SCAN_MODE, &LR1110.gnss.delay);
	lr1110_gnss_scan_assisted(&LR1110, 0, LR1110_GNSS_OPTION_DEFAULT,
			LR1110_GNSS_BIT_CHANGE_MASK
			| LR1110_GNSS_DOPPLER_MASK
		    | LR1110_GNSS_IRQ_PSEUDO_RANGE_MASK
			, 16);
//	lr1110_gnss_scan_autonomous(&LR1110, 0,
//			LR1110_GNSS_BIT_CHANGE_MASK
//			| LR1110_GNSS_DOPPLER_MASK
//			| LR1110_GNSS_IRQ_PSEUDO_RANGE_MASK
//			, 10);

	lr1110_gnss_scan_continuous(&LR1110);

}

static void OnWifiTimerEvent(  void* context  )
{
	SECURE_LED_YELLOW(GPIO_PIN_SET);

	TimerStart(&Led1Timer);

	TimerStop( &TxTimer );

	TimerStop( &WifiTimer );

	lr1110_system_stat1_t stat1;
	lr1110_system_stat2_t stat2;
	uint32_t irq = 0;
	lr1110_system_get_status(&LR1110, &stat1, &stat2, &irq);

	lr1110_system_errors_t errors = { 0 };
	lr1110_system_get_errors(&LR1110, &errors);
	lr1110_system_clear_errors(&LR1110);
	lr1110_system_get_and_clear_irq_status(&LR1110, &irq);
	lr1110_system_set_standby(&LR1110, LR1110_SYSTEM_STANDBY_CFG_RC);
	lr1110_wifi_scan(&LR1110, LR1110_WIFI_TYPE_SCAN_B_G_N, 0x3FFF, LR1110_WIFI_SCAN_MODE_BEACON,
	    			3, 3, 500, true);

}

//#ifdef TEST_WIFI_SCAN_MAC
/* *********************************************************************
 * LR1110 WiFi positioning protocol
 * *********************************************************************
 * The WiFi positioning protocol messages consist of a single tag-value
 * pair per message in uplink direction. Downlinks are not defined for
 * this protocol.
 * https://www.loracloud.com/documentation/device_management?url=v1.html
 * #lr1110-wifi-positioning-protocol
 ***********************************************************************
 * U-WIFILOC-MACRSSI - 0x01: WiFi AP MAC List with RSSI
 * Byte Length |    1   |	1  |  6  |     |   1  |  6  |
 * Field:	   |TAG=0x01| RSSI | MAC | ... | RSSI |	MAC |
 *
 * MAC Address Encoding:
 * A 46-bit MAC address denoted as M6:M5:M4:M3:M2:M1 is encoded in:
 * Byte Length |  1 |  1 |  1 |  1 |  1 |  1 |
 * Field:	   | M6 | M5 | M4 | M3 | M2 | M1 |
 *
 * Maintain: Anol P. <anol.p@emone.co.th>
 */
void OnRadioWifiDone(void)
{
	lr1110_wifi_get_nb_results(&LR1110, &LR1110.wifi.nb_results);
	lr1110_wifi_read_basic_complete_results(&LR1110, 0, LR1110.wifi.nb_results, LR1110.wifi.all_results);
	printf("WiFi scan num: %d\n\n", LR1110.wifi.nb_results);

	AppData.Port = 4;
	AppData.Buffer[0] = 0x01; //TAG=0x01
	AppData.BufferSize = 1;
	for (int wn = 0; wn < LR1110.wifi.nb_results; ++wn) {
		AppData.Buffer[AppData.BufferSize] = LR1110.wifi.all_results[wn].rssi;
		AppData.BufferSize++;
		for (int var = 0; var < LR1110_WIFI_MAC_ADDRESS_LENGTH; ++var) {
			AppData.Buffer[AppData.BufferSize] = LR1110.wifi.all_results[wn].mac_address[LR1110_WIFI_MAC_ADDRESS_LENGTH - var];
			AppData.BufferSize++;
		}

		printf("WiFi index: %d MAC:", wn);
		PrintHexBuffer(LR1110.wifi.all_results[wn].mac_address, LR1110_WIFI_MAC_ADDRESS_LENGTH);
		printf("Rssi: %d Phi_Offset: %d Timestamp: %lld\n\n",
								LR1110.wifi.all_results[wn].rssi,
								LR1110.wifi.all_results[wn].phi_offset,
								LR1110.wifi.all_results[wn].timestamp_us);
		printf("Beacon: %ul Channel_Info: 0x%X Data_Rate: 0X%X Frame_Type: 0x%X\n",
				LR1110.wifi.all_results[wn].beacon_period_tu,
				LR1110.wifi.all_results[wn].channel_info_byte,
				LR1110.wifi.all_results[wn].data_rate_info_byte,
				LR1110.wifi.all_results[wn].frame_type_info_byte);
	}

//    lr1110_wifi_search_country_code( &LR1110, 0x3FFF, 3, 3, 1000, false);
//	lr1110_wifi_get_nb_country_code_results(&LR1110, &LR1110.wifi.nb_countries);
//	lr1110_wifi_read_country_code_results(&LR1110, 0, LR1110.wifi.nb_countries, LR1110.wifi.countries);
//
//	printf("\nCountries num: %d\n\n", LR1110.wifi.nb_countries);
//
//	for (int cn = 0; cn < LR1110.wifi.nb_countries; ++cn) {
//		printf("Country: %c%c Mac:", LR1110.wifi.countries[cn].country_code[0], LR1110.wifi.countries[cn].country_code[1]);
//		PrintHexBuffer(LR1110.wifi.countries[cn].mac_address, 6);
//		printf("IO_Reg: 0x%X Channel_info: 0x%X\n\n", LR1110.wifi.countries[cn].io_regulation,
//				LR1110.wifi.countries[cn].channel_info_byte);
//	}
	if (LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed)
			== LORAMAC_HANDLER_SUCCESS) {
		SECURE_LED_YELLOW(GPIO_PIN_SET);

		TimerStart(&Led1Timer);
	}

	OnTxTimerEvent( NULL );

	TimerStart(&WifiTimer);
}
//#endif

//#ifdef TEST_GNSS_SCAN
/* *********************************************************************
 * LR1110 GNSS positioning protocol
 * *********************************************************************
 * The LR1110 GNSS positioning messages consist of a single tag-value
 * pair per message in uplink and downlink direction.
 * Their purpose is to deliver GNSS scan results to the geolocation
 * solver and communicate aiding information adjustments back
 * to the device.
 * https://www.loracloud.com/documentation/device_management?url=v1.html
 * #lr1110-gnss-positioning-protocol
 ***********************************************************************
 * U-GNSSLOC-REQAID - 0x00: Aiding position request
 * Byte Length |    1   |
 * Field:	   |TAG=0x00|
 *
 * U-GNSSLOC-NAV - 0x01: GNSS Navigation message (NAV)
 * Byte Length |    1   |	VL  |
 * Field:	   |TAG=0x01| GNSS_SCAN_RES |
 * GNSS_SCAN_RES: GNSS scan result containing GNSS signal measurements.
 *
 * D-GNSSLOC-AIDP - 0x00: Aiding position update
 * Byte Length |    1   |	3  |
 * Field:	   |TAG=0x00|  POS |
 * POS contains an estimate of the device’s position encoded in WGS84
 * latitude/longitude coordinates.
 * Bits: | 23 - 12 | 11 - 0 |
 * Field:|   LAT   |   LON  |
 *
 * Maintain: Anol P. <anol.p@emone.co.th>
 */
void OnRadioGnssDone(void)
{
	AppData.Port = 5;
	AppData.Buffer[0] = 0x01; //TAG=0x01
	AppData.BufferSize = 1;

	lr1110_gnss_get_result_size(&LR1110, &LR1110.gnss.len);
	lr1110_gnss_read_results(&LR1110, (uint8_t *) &LR1110.gnss.results, LR1110.gnss.len);

	printf("GNSS: len %d : result:", LR1110.gnss.len);
	PrintHexBuffer(LR1110.gnss.results,  LR1110.gnss.len);
	printf("\r\n");

	lr1110_gnss_get_nb_detected_satellites(&LR1110, &LR1110.gnss.nb_satellite);
	printf("GNSS: SV %d detected\r\n", LR1110.gnss.nb_satellite);
	lr1110_gnss_get_detected_satellites(&LR1110, LR1110.gnss.nb_satellite, LR1110.gnss.satellite);
	for (int var = 0; var < LR1110.gnss.nb_satellite; ++var) {
		printf("    GNSS: SV index %d CNR: %d\r\n", LR1110.gnss.satellite[var].satellite_id, LR1110.gnss.satellite[var].cnr);
						   
	}
	lr1110_gnss_get_timings(&LR1110, &LR1110.gnss.timings);
	printf("GNSS: timing radio: %ld computation: %ld\r\n", LR1110.gnss.timings.radio_ms, LR1110.gnss.timings.computation_ms);

	memcpy1(&AppData.Buffer[AppData.BufferSize], (uint8_t *) &LR1110.gnss.results, LR1110.gnss.len);
	AppData.BufferSize += LR1110.gnss.len;

	// U-GNSSLOC-NAV - 0x01: GNSS Navigation message (NAV)
	if (LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed)
			== LORAMAC_HANDLER_SUCCESS) {

		SECURE_LED_YELLOW(GPIO_PIN_SET);

		TimerStart(&Led1Timer);

		//U-GNSSLOC-REQAID - 0x00: Aiding position request
		AppData.Port = 6;
		AppData.Buffer[0] = 0x00; //TAG=0x00
		AppData.BufferSize = 1;

		if (LmHandlerSend(&AppData, LORAMAC_HANDLER_CONFIRMED_MSG)
					== LORAMAC_HANDLER_SUCCESS) {
			SECURE_LED_YELLOW(GPIO_PIN_SET);

			TimerStart(&Led1Timer);
		}
	}

	OnTxTimerEvent( NULL );

	TimerStart(&GnssTimer);
}
//#endif

void BoardInitMcu( void )
{
	if (McuInitialized == false)
	{
		RtcInit();
		BSP_GYRO_Init();
#if V010
		BSP_IDD_Init(0);
		BSP_IDD_Init(1);
#endif
		LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );
	} else {
		SystemClockReConfig();
	}

	BoardInitPeriph();

	if (McuInitialized == false){
		McuInitialized = true;
		lr1110_board_init_dbg_io( &LR1110 );
	} else {
		CalibrateSystemWakeupTime( );
	}
}

void BoardInitPeriph( void )
{
	MX_LPUART1_UART_Init();
	SpiInit(&LR1110.spi, SPI_3, LR_MOSI_GPIO_Port, LR_MOSI_Pin,
				  LR_MISO_GPIO_Port, LR_MISO_Pin, LR_SCK_GPIO_Port, LR_SCK_Pin, NULL, NC);
	lr1110_board_init_io( &LR1110 );
}

void BoardDeInitMcu( void )
{
//    SpiDeInit( &LR1110.spi );
//    lr1110_board_deinit_io( &LR1110 );
}

void CalibrateSystemWakeupTime( void )
{
    if( SystemWakeupTimeCalibrated == false )
    {
        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
        TimerStart( &CalibrateSystemWakeupTimeTimer );
        while( SystemWakeupTimeCalibrated == false )
        {

        }
    } else {

    }
}

void SystemClockReConfig( void )
{
      RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
      RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
      uint32_t pFLatency = 0;

    CRITICAL_SECTION_BEGIN( );

    // In case nvic had a pending IT, the arm doesn't enter stop mode
    // Hence the pll is not switched off and will cause HAL_RCC_OscConfig return
    // an error
    if ( __HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS )
    {
        // Enable Power Control clock
        __HAL_RCC_PWR_CLK_ENABLE( );

        // Get the Oscillators configuration according to the internal RCC registers */
        HAL_RCC_GetOscConfig( &RCC_OscInitStruct );

        // Enable PLL
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
        {
            Error_Handler();
        }

        /* Get the Clocks configuration according to the internal RCC registers */
        HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

        /* Select PLL as system clock source and keep HCLK, PCLK1 and PCLK2 clocks dividers as before */
        RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
        RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
        {
        	Error_Handler();
        }
    }
    else
    {
        // MCU did not enter stop mode beacuse NVIC had a pending IT
    }

    CRITICAL_SECTION_END( );
}

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

/* USER CODE END 4 */

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
#ifdef VERSION_020
	  SECURE_LEDToggle_RED();
#else
#endif
	  for (int var = 0; var < 1000000; ++var) {
		__NOP();
	}
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
  /* User can add his own implementation to report the file name and line number,*/
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
