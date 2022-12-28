/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "ble_hci_le.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* DistanceMeasure */
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  uint8_t               SW1_Status;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CUSTOM_STM_HRS_VALUE_MIN        0x44
#define CUSTOM_STM_HRS_VALUE_MAX        0x49
#define CUSTOM_STM_HRS_ENERGY_MIN       0x0100
#define CUSTOM_STM_HRS_ENERGY_RESET     0x00
#define CUSTOM_STM_HRS_ENERGY_NOT_RESET 0x01

#define TOGGLE_ON                       1
#define TOGGLE_OFF                      0
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
uint8_t hr_energy_reset = CUSTOM_STM_HRS_ENERGY_NOT_RESET;

uint16_t connection;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* DistanceMeasure */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* DistanceMeasure */
    case CUSTOM_STM_LED_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LED_C_READ_EVT */
       PRINT_MESG_DBG("ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE My_Led_Char Read\n");

      /* USER CODE END CUSTOM_STM_LED_C_READ_EVT */
      break;

    case CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT */
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT \n");
      APP_DBG_MSG("\r\n\r** Write Data: 0x%02X %02X \n", pNotification->DataTransfered.pPayload[0], pNotification->DataTransfered.pPayload[1]);
      if (pNotification->DataTransfered.pPayload[1] == 0x01)
      {
        HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_SET);
      }

      if (pNotification->DataTransfered.pPayload[1] == 0x00)
      {
        HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_RESET);
      }

      /* USER CODE END CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */
  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
    	connection = pNotification->ConnectionHandle;
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
      connection = NULL;
      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
#define FILTER_LENGTH 5
#define CUT_OFF 3

int sum = 0;
int counter = 0;

double percent_window[FILTER_LENGTH];

double calculate_percent(double distance) {
	const int end = FILTER_LENGTH - 1;

	for (int i = 0; i < end; i++) {
		percent_window[i] = percent_window[i + 1];
	}

	percent_window[end] = distance;

	double sum = 0;

	for (int i = 0; i < 3; i++) {
		sum += percent_window[i];
	}

	double avg = sum / FILTER_LENGTH;

	return fmax(1 - ( avg / CUT_OFF ), 0);
}

void read_rssi_from_connection() {
	const int tx = -57; // This is the constant RSSI the device has when being 1m away

	int8_t rssi;
	uint8_t status_rssi;

	int avg;
	int sigma;
	double ratio;
	double distance;
	double percent;

	if (connection != NULL) {
		status_rssi = hci_read_rssi(connection, (uint8_t *) &rssi);

		if (BLE_STATUS_SUCCESS == status_rssi && rssi != 127) {
			counter++;
			if (counter == 1000) {
				avg = sum / 1000;
				sigma = tx - avg;
				ratio = sigma / 40.0;
				distance = pow(10.0, ratio);
				percent = calculate_percent(distance);

				dim_led(percent);

				APP_DBG_MSG("Connection: %d has RSSI: %d and tx: %d resulting in distance: %f m and dims to %f %% \n", connection, avg, tx, distance, percent * 100);

				counter = 0;
				sum = 0;
			}

			sum += rssi;
		}
	}

	UTIL_SEQ_SetTask(1 << CFG_TASK_READ_RSSI_ID, CFG_SCH_PRIO_0);
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* DistanceMeasure */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void SW1_Button_Action(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
