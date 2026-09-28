/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    WL55C1PingPong.c
  * @brief   Application SubGHz Phy - Station2 (solo LoRa)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32wlxx_hal.h"
#include "stm32wlxx_nucleo.h"       /* BSP LED */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  RX,
  TX,
} States_t;

/* USER CODE END PTD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_TIMEOUT_VALUE              3000
#define TX_TIMEOUT_VALUE              3000
#define MAX_APP_BUFFER_SIZE           255

#define STATION2_NAME                 "Station2"

#define RESPONSE_DELAY_MS             50
#define LED_RX_FEEDBACK_MS            200

#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif

/* USER CODE END PD */


/* Private variables ---------------------------------------------------------*/

static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

static States_t State = RX;

static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
static uint16_t RxBufferSize = 0;

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/

static void OnTxDone(void);
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr);
static void OnTxTimeout(void);
static void OnRxTimeout(void);
static void OnRxError(void);

/* USER CODE BEGIN PFP */

static void Station_Process(void);
static void RadioSend(void);
static void RadioRx(void);
static void ProcessReceivedMessage(void);
static void PrepareResponse(void);

/* USER CODE END PFP */


/* Exported functions --------------------------------------------------------*/

void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  /* LED board */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);

  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);

  /* Buffer */
  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio init */
  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.RxDone    = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError   = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  Radio.SetChannel(RF_FREQUENCY);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
                   UTIL_SEQ_RFU,
                   Station_Process);

  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);

  RadioRx();

  /* USER CODE END SubghzApp_Init_2 */
}


/* Private functions ---------------------------------------------------------*/

static void OnTxDone(void)
{
  State = TX;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr)
{
  (void)rssi;
  (void)LoraSnr;

  if (size >= MAX_APP_BUFFER_SIZE)
  {
    RxBufferSize = 0;
    RadioRx();
    return;
  }

  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  memcpy(BufferRx, payload, size);
  BufferRx[size] = '\0';
  RxBufferSize = size;

  State = RX;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void OnTxTimeout(void)
{
  State = RX;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void OnRxTimeout(void)
{
  State = RX;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void OnRxError(void)
{
  State = RX;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}


/* USER CODE BEGIN PrFD */

static void RadioSend(void)
{
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

  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

  uint16_t txSize = strlen((char *)BufferTx);
  Radio.Send(BufferTx, txSize);
}


static void RadioRx(void)
{
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

  if (LORA_FIX_LENGTH_PAYLOAD_ON == true)
  {
    Radio.SetMaxPayloadLength(MODEM_LORA, PAYLOAD_LEN);
  }
  else
  {
    Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  }

  /* Attesa RX: entrambi i LED spenti */
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);

  Radio.Rx(RX_TIMEOUT_VALUE);
}


static void ProcessReceivedMessage(void)
{
  if (RxBufferSize == 0)
  {
    RadioRx();
    return;
  }

  int value1;
  int value2;

  if (sscanf((char *)BufferRx, "Station1,%d,%d", &value1, &value2) == 2)
  {
    /* Feedback RX: verde acceso per 200 ms */
    BSP_LED_On(LED_GREEN);
    BSP_LED_Off(LED_BLUE);
    HAL_Delay(LED_RX_FEEDBACK_MS);
    BSP_LED_Off(LED_GREEN);

    PrepareResponse();
  }
  else
  {
    RxBufferSize = 0;
    memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
    RadioRx();
  }
}


static void PrepareResponse(void)
{
  HAL_Delay(RESPONSE_DELAY_MS);

  memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);
  snprintf((char *)BufferTx, MAX_APP_BUFFER_SIZE, "%s", STATION2_NAME);

  /* TX: blu acceso, verde spento */
  BSP_LED_Off(LED_GREEN);
  BSP_LED_On(LED_BLUE);

  State = TX;
  RadioSend();
}


static void Station_Process(void)
{
  Radio.Sleep();

  switch (State)
  {
    case RX:
      ProcessReceivedMessage();
      break;

    case TX:
      RxBufferSize = 0;
      memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
      RadioRx();
      break;

    default:
      RadioRx();
      break;
  }
}

/* USER CODE END PrFD */
