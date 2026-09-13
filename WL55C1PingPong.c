/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed under terms that can be found in the
  * LICENSE file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* LED esterni                                                               */
/* -------------------------------------------------------------------------- */

#define LED_PING_GPIO_Port GPIOA
#define LED_PING_Pin       GPIO_PIN_5

#define LED_PONG_GPIO_Port GPIOA
#define LED_PONG_Pin       GPIO_PIN_6

/* USER CODE END Includes */


/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
  * @brief Stati della macchina a stati
  */
typedef enum
{
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;

/* USER CODE END PTD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* -------------------------------------------------------------------------- */
/* Configurazione                                                             */
/* -------------------------------------------------------------------------- */

#define RX_TIMEOUT_VALUE              3000
#define TX_TIMEOUT_VALUE              3000

#define MAX_APP_BUFFER_SIZE           255

/* -------------------------------------------------------------------------- */
/* Nomi delle due stazioni                                                   */
/* -------------------------------------------------------------------------- */

#define STATION1_NAME                 "Station1"
#define STATION2_NAME                 "Station2"

/* -------------------------------------------------------------------------- */
/* Ritardo prima della risposta                                              */
/* -------------------------------------------------------------------------- */

#define RESPONSE_DELAY_MS             50

/* -------------------------------------------------------------------------- */
/* Periodo LED                                                                */
/* -------------------------------------------------------------------------- */

#define LED_PERIOD_MS                 200

/* -------------------------------------------------------------------------- */
/* Controllo dimensione payload                                              */
/* -------------------------------------------------------------------------- */

#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif

/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH             83333

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/

int PacketNumber = 0;
int ACKNumber = 0;   // ACK al pacchetto Number
int action = 2;   //1 Apri acqua - 2 Chiudi acqua

/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

/* -------------------------------------------------------------------------- */
/* Stato della macchina a stati                                              */
/* -------------------------------------------------------------------------- */

static States_t State = RX;

/* -------------------------------------------------------------------------- */
/* Buffer RX                                                                   */
/* -------------------------------------------------------------------------- */

static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];

/* -------------------------------------------------------------------------- */
/* Buffer TX                                                                   */
/* -------------------------------------------------------------------------- */

static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];

/* -------------------------------------------------------------------------- */
/* Dimensione ultimo pacchetto ricevuto                                      */
/* -------------------------------------------------------------------------- */

static uint16_t RxBufferSize = 0;

/* -------------------------------------------------------------------------- */
/* Parametri radio                                                            */
/* -------------------------------------------------------------------------- */

static int8_t RssiValue = 0;
static int8_t SnrValue = 0;

/* -------------------------------------------------------------------------- */
/* Timer LED                                                                   */
/* -------------------------------------------------------------------------- */

static UTIL_TIMER_Object_t timerLed;

/* -------------------------------------------------------------------------- */
/* Valori ricevuti da Station1                                               */
/* -------------------------------------------------------------------------- */

static int Station1_Value1 = 0;
static int Station1_Value2 = 0;

/* -------------------------------------------------------------------------- */
/* Valori che Station2 restituisce                                           */
/*                                                                            */
/* SOSTITUISCI QUESTI DUE VALORI con le tue variabili reali.                  */
/* -------------------------------------------------------------------------- */

static int Station2_Value1 = 0;
static int Station2_Value2 = 0;

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/

/**
  * @brief Function to be executed on Radio Tx Done event
  */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload,
                     uint16_t size,
                     int16_t rssi,
                     int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);


/* USER CODE BEGIN PFP */

/**
  * @brief Function executed when LED timer elapses
  */
static void OnledEvent(void *context);

/**
  * @brief Main application state machine
  */
static void Station_Process(void);

/**
  * @brief Configure and start radio TX
  */
static void RadioSend(void);

/**
  * @brief Configure and start radio RX
  */
static void RadioRx(void);

/**
  * @brief Process message received from Station1
  */
static void ProcessReceivedMessage(void);

/**
  * @brief Prepare response message from Station2
  */
static void PrepareResponse(void);

/* USER CODE END PFP */


/* Exported functions ---------------------------------------------------------*/

void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  /* ------------------------------------------------------------------------ */
  /* Configurazione LED esterni                                               */
  /* ------------------------------------------------------------------------ */

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = LED_PING_Pin | LED_PONG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(LED_PING_GPIO_Port, &GPIO_InitStruct);

  /* LED inizialmente spenti */
  HAL_GPIO_WritePin(LED_PING_GPIO_Port,
                    LED_PING_Pin,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LED_PONG_GPIO_Port,
                    LED_PONG_Pin,
                    GPIO_PIN_RESET);

  /* ------------------------------------------------------------------------ */
  /* Messaggi di inizializzazione                                             */
  /* ------------------------------------------------------------------------ */

  APP_LOG(TS_OFF, VLEVEL_M, "\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "=================================\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "       STATION2 LoRa RX/TX\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "=================================\n\r");

  /* Application version */
  APP_LOG(TS_OFF,
          VLEVEL_M,
          "APPLICATION_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Middleware version */
  APP_LOG(TS_OFF,
          VLEVEL_M,
          "MW_RADIO_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* ------------------------------------------------------------------------ */
  /* Timer LED                                                                */
  /* ------------------------------------------------------------------------ */

  UTIL_TIMER_Create(&timerLed,
                    LED_PERIOD_MS,
                    UTIL_TIMER_ONESHOT,
                    OnledEvent,
                    NULL);

  /* ------------------------------------------------------------------------ */
  /* Inizializzazione buffer                                                  */
  /* ------------------------------------------------------------------------ */

  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);

  /* USER CODE END SubghzApp_Init_1 */


  /* ------------------------------------------------------------------------ */
  /* Radio initialization                                                     */
  /* ------------------------------------------------------------------------ */

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);


  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* ------------------------------------------------------------------------ */
  /* Frequenza radio                                                          */
  /* ------------------------------------------------------------------------ */

  Radio.SetChannel(RF_FREQUENCY);


  /* ------------------------------------------------------------------------ */
  /* Stampa configurazione LoRa/FSK                                           */
  /* ------------------------------------------------------------------------ */

#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))

  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");

  APP_LOG(TS_OFF,
          VLEVEL_M,
          "LORA_BW=%d kHz\n\r",
          (1 << LORA_BANDWIDTH) * 125);

  APP_LOG(TS_OFF,
          VLEVEL_M,
          "LORA_SF=%d\n\r",
          LORA_SPREADING_FACTOR);

  APP_LOG(TS_OFF,
          VLEVEL_M,
          "RF_FREQUENCY=%d Hz\n\r",
          RF_FREQUENCY);

#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))

  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");

  APP_LOG(TS_OFF,
          VLEVEL_M,
          "FSK_BW=%d Hz\n\r",
          FSK_BANDWIDTH);

  APP_LOG(TS_OFF,
          VLEVEL_M,
          "FSK_DR=%d bits/s\n\r",
          FSK_DATARATE);

#else

#error "Please define a modulation in the subghz_phy_app.h file."

#endif


  /* ------------------------------------------------------------------------ */
  /* Registra il task applicativo                                            */
  /* ------------------------------------------------------------------------ */

  UTIL_SEQ_RegTask(
      (1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
      UTIL_SEQ_RFU,
      Station_Process);


  /* ------------------------------------------------------------------------ */
  /* Station2 parte immediatamente in RX                                     */
  /* ------------------------------------------------------------------------ */

  APP_LOG(TS_ON,
          VLEVEL_L,
          "Station2 pronta. Attendo Station1...\n\r");

  RadioRx();

  /* USER CODE END SubghzApp_Init_2 */
}


/* USER CODE BEGIN EF */

/* USER CODE END EF */


/* Private functions ---------------------------------------------------------*/


/**
  * @brief Radio TX done callback
  */
static void OnTxDone(void)
{
  APP_LOG(TS_ON,
          VLEVEL_L,
          "Station2: TX completato\n\r");

  State = TX;

  UTIL_SEQ_SetTask(
      (1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
      CFG_SEQ_Prio_0);
}


/**
  * @brief Radio RX done callback
  */
static void OnRxDone(uint8_t *payload,
                     uint16_t size,
                     int16_t rssi,
                     int8_t LoraSnr_FskCfo)
{
  APP_LOG(TS_ON,
          VLEVEL_L,
          "Station2: RX completato\n\r");


  /* ------------------------------------------------------------------------ */
  /* RSSI / SNR                                                               */
  /* ------------------------------------------------------------------------ */

#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))

  APP_LOG(TS_ON,
          VLEVEL_L,
          "RSSI=%d dBm, SNR=%d dB\n\r",
          rssi,
          LoraSnr_FskCfo);

  SnrValue = LoraSnr_FskCfo;

#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))

  APP_LOG(TS_ON,
          VLEVEL_L,
          "RSSI=%d dBm, CFO=%d kHz\n\r",
          rssi,
          LoraSnr_FskCfo);

  SnrValue = 0;

#endif


  RssiValue = rssi;


  /* ------------------------------------------------------------------------ */
  /* Controllo dimensione                                                     */
  /* ------------------------------------------------------------------------ */

  if (size >= MAX_APP_BUFFER_SIZE)
  {
    APP_LOG(TS_ON,
            VLEVEL_L,
            "ERRORE: payload troppo grande: %d byte\n\r",
            size);

    RxBufferSize = 0;

    RadioRx();

    return;
  }


  /* ------------------------------------------------------------------------ */
  /* Pulisce buffer                                                            */
  /* ------------------------------------------------------------------------ */

  memset(BufferRx,
         0,
         MAX_APP_BUFFER_SIZE);


  /* ------------------------------------------------------------------------ */
  /* Copia payload                                                             */
  /* ------------------------------------------------------------------------ */

  memcpy(BufferRx,
         payload,
         size);


  /* ------------------------------------------------------------------------ */
  /* Aggiunge terminatore stringa                                             */
  /* ------------------------------------------------------------------------ */

  BufferRx[size] = '\0';

  RxBufferSize = size;


  /* ------------------------------------------------------------------------ */
  /* Stampa messaggio ricevuto                                                */
  /* ------------------------------------------------------------------------ */

  APP_LOG(TS_ON,
          VLEVEL_L,
          "Payload ricevuto (%d byte): %s\n\r",
          RxBufferSize,
          (char *)BufferRx);

  if (sscanf((char *)BufferRx, "Station1,%d,%d",
             &PacketNumber,
             &action) == 2)
  {
      APP_LOG(TS_ON, VLEVEL_L,
              "Station1 ricevuta\r\n");

      APP_LOG(TS_ON, VLEVEL_L,
              "PacketNumber = %d\r\n",
			  PacketNumber);

      APP_LOG(TS_ON, VLEVEL_L,
              "action = %d\r\n",
			  action);
  }




  /* ------------------------------------------------------------------------ */
  /* Avvia processamento nel task applicativo                                 */
  /* ------------------------------------------------------------------------ */

  State = RX;

  UTIL_SEQ_SetTask(
      (1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
      CFG_SEQ_Prio_0);
}


/**
  * @brief Radio TX timeout callback
  */
static void OnTxTimeout(void)
{
  APP_LOG(TS_ON,
          VLEVEL_L,
          "Station2: TX Timeout\n\r");

  State = TX_TIMEOUT;

  UTIL_SEQ_SetTask(
      (1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
      CFG_SEQ_Prio_0);
}


/**
  * @brief Radio RX timeout callback
  */
static void OnRxTimeout(void)
{
  APP_LOG(TS_ON,
          VLEVEL_L,
          "Station2: RX Timeout\n\r");

  State = RX_TIMEOUT;

  UTIL_SEQ_SetTask(
      (1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
      CFG_SEQ_Prio_0);
}


/**
  * @brief Radio RX error callback
  */
static void OnRxError(void)
{
  APP_LOG(TS_ON,
          VLEVEL_L,
          "Station2: RX Error\n\r");

  State = RX_ERROR;

  UTIL_SEQ_SetTask(
      (1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
      CFG_SEQ_Prio_0);
}


/* USER CODE BEGIN PrFD */


/**
  * @brief Configure radio TX and send BufferTx
  */
static void RadioSend(void)
{
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))

  Radio.Sleep();

  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA,
                    TX_OUTPUT_POWER,
                    0,
                    LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH,
                    LORA_FIX_LENGTH_PAYLOAD_ON,
                    true,
                    0,
                    0,
                    LORA_IQ_INVERSION_ON,
                    TX_TIMEOUT_VALUE);

  Radio.SetMaxPayloadLength(MODEM_LORA,
                            MAX_APP_BUFFER_SIZE);

#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))

  Radio.Sleep();

  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_FSK,
                    TX_OUTPUT_POWER,
                    FSK_FDEV,
                    0,
                    FSK_DATARATE,
                    0,
                    FSK_PREAMBLE_LENGTH,
                    FSK_FIX_LENGTH_PAYLOAD_ON,
                    true,
                    0,
                    0,
                    0,
                    TX_TIMEOUT_VALUE);

  Radio.SetMaxPayloadLength(MODEM_FSK,
                            MAX_APP_BUFFER_SIZE);

#else

#error "Please define a modulation in the subghz_phy_app.h file."

#endif


  /* ------------------------------------------------------------------------ */
  /* Trasmette la lunghezza reale della stringa                               */
  /* ------------------------------------------------------------------------ */

  uint16_t txSize = strlen((char *)BufferTx);

  APP_LOG(TS_ON,
          VLEVEL_L,
          "Station2 TX: %s (%d byte)\n\r",
          BufferTx,
          txSize);

  Radio.Send(BufferTx, txSize);
}


/**
  * @brief Configure radio RX
  */
static void RadioRx(void)
{
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))

  Radio.Sleep();

  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetRxConfig(MODEM_LORA,
                    LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE,
                    0,
                    LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT,
                    LORA_FIX_LENGTH_PAYLOAD_ON,
                    0,
                    true,
                    0,
                    0,
                    LORA_IQ_INVERSION_ON,
                    true);

  /*
   * Se il payload è a lunghezza fissa usiamo PAYLOAD_LEN.
   * Altrimenti accettiamo fino a MAX_APP_BUFFER_SIZE.
   */

  if (LORA_FIX_LENGTH_PAYLOAD_ON == true)
  {
    Radio.SetMaxPayloadLength(MODEM_LORA,
                              PAYLOAD_LEN);
  }
  else
  {
    Radio.SetMaxPayloadLength(MODEM_LORA,
                              MAX_APP_BUFFER_SIZE);
  }


#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))

  Radio.Sleep();

  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetRxConfig(MODEM_FSK,
                    FSK_BANDWIDTH,
                    FSK_DATARATE,
                    0,
                    FSK_AFC_BANDWIDTH,
                    FSK_PREAMBLE_LENGTH,
                    0,
                    FSK_FIX_LENGTH_PAYLOAD_ON,
                    0,
                    true,
                    0,
                    0,
                    false,
                    true);

  /*
   * NOTA:
   * Qui bisogna usare FSK_FIX_LENGTH_PAYLOAD_ON,
   * non LORA_FIX_LENGTH_PAYLOAD_ON.
   */

  if (FSK_FIX_LENGTH_PAYLOAD_ON == true)
  {
    Radio.SetMaxPayloadLength(MODEM_FSK,
                              PAYLOAD_LEN);
  }
  else
  {
    Radio.SetMaxPayloadLength(MODEM_FSK,
                              MAX_APP_BUFFER_SIZE);
  }


#else

#error "Please define a modulation in the subghz_phy_app.h file."

#endif


  APP_LOG(TS_ON,
          VLEVEL_L,
          "Station2: RX start\n\r");

  Radio.Rx(RX_TIMEOUT_VALUE);
}


/**
  * @brief Process message received from Station1
  */
static void ProcessReceivedMessage(void)
{
  int value1;
  int value2;


  /* ------------------------------------------------------------------------ */
  /* Controllo che il messaggio non sia vuoto                                 */
  /* ------------------------------------------------------------------------ */

  if (RxBufferSize == 0)
  {
    APP_LOG(TS_ON,
            VLEVEL_L,
            "RX vuoto\n\r");

    RadioRx();

    return;
  }


  /* ------------------------------------------------------------------------ */
  /* Controllo formato                                                        */
  /*                                                                            */
  /* Formato previsto:                                                        */
  /*                                                                            */
  /* Station1,123,456                                                         */
  /* ------------------------------------------------------------------------ */

  if (sscanf((char *)BufferRx,
             "Station1,%d,%d",
             &value1,
             &value2) == 2)
  {
    /* ---------------------------------------------------------------------- */
    /* Salva valori ricevuti                                                  */
    /* ---------------------------------------------------------------------- */

    Station1_Value1 = value1;
    Station1_Value2 = value2;


    APP_LOG(TS_ON,
            VLEVEL_L,
            "Messaggio valido da Station1\n\r");

    APP_LOG(TS_ON,
            VLEVEL_L,
            "PacketNumber = %d\n\r",
            Station1_Value1);

    APP_LOG(TS_ON,
            VLEVEL_L,
            "Value2 = %d\n\r",
            Station1_Value2);


    /* ---------------------------------------------------------------------- */
    /* LED PING = messaggio Station1 ricevuto                                 */
    /* ---------------------------------------------------------------------- */

    HAL_GPIO_WritePin(LED_PING_GPIO_Port,
                      LED_PING_Pin,
                      GPIO_PIN_SET);

    HAL_GPIO_WritePin(LED_PONG_GPIO_Port,
                      LED_PONG_Pin,
                      GPIO_PIN_RESET);


    /* ---------------------------------------------------------------------- */
    /* Prepara risposta                                                        */
    /* ---------------------------------------------------------------------- */

    PrepareResponse();
  }
  else
  {
    /* ---------------------------------------------------------------------- */
    /* Messaggio non valido                                                    */
    /* ---------------------------------------------------------------------- */

    APP_LOG(TS_ON,
            VLEVEL_L,
            "Messaggio non valido: %s\n\r",
            BufferRx);


    /* ---------------------------------------------------------------------- */
    /* Ignora il messaggio e torna in RX                                      */
    /* ---------------------------------------------------------------------- */

    RxBufferSize = 0;

    memset(BufferRx,
           0,
           MAX_APP_BUFFER_SIZE);

    RadioRx();
  }
}


/**
  * @brief Prepare Station2 response
  */
static void PrepareResponse(void)
{
  /* ------------------------------------------------------------------------ */
  /*                                                                       */
  /* Qui puoi aggiornare Station2_Value1 e Station2_Value2 con i valori      */
  /* reali che vuoi trasmettere.                                             */
  /*                                                                       */
  /* Esempio:                                                                */
  /*                                                                       */
   Station2_Value1 = PacketNumber;
   Station2_Value2 = action;
  /*                                                                       */
  /* ------------------------------------------------------------------------ */


  /*
   * Piccolo ritardo per dare tempo a Station1 di passare
   * dalla modalità TX alla modalità RX.
   */

  HAL_Delay(RESPONSE_DELAY_MS);


  /* ------------------------------------------------------------------------ */
  /* Costruisce la stringa di risposta                                       */
  /* ------------------------------------------------------------------------ */

  memset(BufferTx,
         0,
         MAX_APP_BUFFER_SIZE);


  snprintf((char *)BufferTx,
           MAX_APP_BUFFER_SIZE,
           "%s,%d,%d",
           STATION2_NAME,
           Station2_Value1,
           Station2_Value2);


  APP_LOG(TS_ON,
          VLEVEL_L,
          "Risposta preparata: %s\n\r",
          BufferTx);


  /* ------------------------------------------------------------------------ */
  /* LED PONG = Station2 sta rispondendo                                     */
  /* ------------------------------------------------------------------------ */

  HAL_GPIO_WritePin(LED_PING_GPIO_Port,
                    LED_PING_Pin,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LED_PONG_GPIO_Port,
                    LED_PONG_Pin,
                    GPIO_PIN_SET);


  /* ------------------------------------------------------------------------ */
  /* Trasmette                                                                */
  /* ------------------------------------------------------------------------ */

  State = TX;

  RadioSend();
}


/**
  * @brief Main Station2 state machine
  */
static void Station_Process(void)
{
  Radio.Sleep();


  switch (State)
  {

    /* ====================================================================== */
    /* RX                                                                       */
    /* ====================================================================== */

    case RX:

      APP_LOG(TS_ON,
              VLEVEL_L,
              "Processo messaggio ricevuto\n\r");

      ProcessReceivedMessage();

      break;


    /* ====================================================================== */
    /* TX                                                                       */
    /* ====================================================================== */

    case TX:

      /*
       * Questo stato viene raggiunto dopo OnTxDone().
       *
       * La risposta è stata trasmessa.
       * Station2 torna quindi immediatamente in RX.
       */

      APP_LOG(TS_ON,
              VLEVEL_L,
              "Station2: torno in RX\n\r");

      RxBufferSize = 0;

      memset(BufferRx,
             0,
             MAX_APP_BUFFER_SIZE);

      RadioRx();

      break;


    /* ====================================================================== */
    /* RX TIMEOUT                                                              */
    /* ====================================================================== */

    case RX_TIMEOUT:

      APP_LOG(TS_ON,
              VLEVEL_L,
              "Station2: RX timeout, continuo ad ascoltare\n\r");

      RadioRx();

      break;


    /* ====================================================================== */
    /* RX ERROR                                                                */
    /* ====================================================================== */

    case RX_ERROR:

      APP_LOG(TS_ON,
              VLEVEL_L,
              "Station2: RX error, continuo ad ascoltare\n\r");

      RadioRx();

      break;


    /* ====================================================================== */
    /* TX TIMEOUT                                                              */
    /* ====================================================================== */

    case TX_TIMEOUT:

      APP_LOG(TS_ON,
              VLEVEL_L,
              "Station2: TX timeout, torno in RX\n\r");

      RadioRx();

      break;


    /* ====================================================================== */
    /* DEFAULT                                                                  */
    /* ====================================================================== */

    default:

      RadioRx();

      break;
  }
}


/**
  * @brief LED timer callback
  */
static void OnledEvent(void *context)
{
  UNUSED(context);

  HAL_GPIO_TogglePin(LED2_GPIO_Port,
                     LED2_Pin);

  HAL_GPIO_TogglePin(LED3_GPIO_Port,
                     LED3_Pin);

  UTIL_TIMER_Start(&timerLed);
}


/* USER CODE END PrFD */
