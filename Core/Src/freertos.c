/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include <stdio.h>
#include "app_types.h"
#include "app_globals.h"
#include "task_sensor.h"
#include "task_filter.h"
#include "task_display.h"
#include "task_log.h"
#include "task_watchdog.h"
#include "task_baro.h"
#include "task_control.h"
//void task_sensor(void *argument);
////void task_print(void *argument);
//void task_display(void *argument);
//void task_log(void *argument);
//void task_watchdog(void *argument);
//void task_filter(void *argument);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMutexId_t uart_mutex;
osMutexId_t i2c_mutex;
osMessageQueueId_t imu_queue;
osMessageQueueId_t state_queue;
osSemaphoreId_t button_sem;
volatile uint8_t logging_enabled;
osMessageQueueId_t baro_queue;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
const osThreadAttr_t sensor_task_attributes = {
    .name = "sensor",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t display_task_attributes = {
    .name = "display",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t print_task_attributes = {
    .name = "print",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t log_task_attributes = {
    .name = "log",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};


const osThreadAttr_t watchdog_task_attributes = {
    .name = "watchdog",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t filter_task_attributes = {
    .name = "filter",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t baro_task_attributes = {
    .name = "baro",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t control_task_attributes = {
    .name = "control",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	uart_mutex = osMutexNew(NULL);
	i2c_mutex = osMutexNew(NULL);
	button_sem = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	imu_queue = osMessageQueueNew(10, sizeof(IMUData_t), NULL);
	state_queue = osMessageQueueNew(10, sizeof(FlightState_t), NULL);
	baro_queue = osMessageQueueNew(5, sizeof(BarData_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadNew(task_sensor, NULL, &sensor_task_attributes);
//  osThreadNew(task_print, NULL, &print_task_attributes);
  osThreadNew(task_display, NULL, &display_task_attributes);
  osThreadNew(task_log, NULL, &log_task_attributes);
  osThreadNew(task_watchdog, NULL, &watchdog_task_attributes);
  osThreadNew(task_filter, NULL, &filter_task_attributes);
  osThreadNew(task_baro, NULL, &baro_task_attributes);
  osThreadNew(task_control, NULL, &control_task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

