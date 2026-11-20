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
#include "usart.h"
#include <string.h>
#include "app_globals.h"
#include "task_sensor.h"
#include "task_filter.h"
#include "task_display.h"
#include "task_log.h"
#include "task_watchdog.h"
#include "task_baro.h"
#include "task_control.h"
#include "task_gps.h"
#include "task_rc.h"
#include "task_telemetry.h"

void task_sensor(void *argument);
//void task_print(void *argument);
void task_display(void *argument);
void task_log(void *argument);
void task_watchdog(void *argument);
void task_filter(void *argument);
void task_gps(void *argument);
void task_control(void *argument);
void task_baro(void *argument);
void task_rc(void *argument);
void task_telemetry(void* argument);
void task_loggr(void *argument);
void task_dshot(void *argument);
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
osMessageQueueId_t control_state_queue;
osSemaphoreId_t button_sem;
volatile uint8_t logging_enabled = 1;
osMessageQueueId_t baro_queue;
osMessageQueueId_t gps_queue;
volatile uint8_t armed = 0;
volatile float throttle = 0.0f;
osMessageQueueId_t rc_queue;
volatile uint32_t rc_last_packet_ms = 0;
osMessageQueueId_t telemetry_queue;
osMessageQueueId_t log_queue;

/* ---- Stack monitor ------------------------------------------------------
 * Records every task handle so task_stackmon can print each task's
 * worst-case free stack. A task that never drops below ~200 bytes free
 * is safe; one that approaches 0 is about to corrupt its neighbour. */
typedef struct { const char *name; osThreadId_t h; } TaskMon_t;
static TaskMon_t mon[16];
static int mon_count = 0;

static void mon_add(const char *name, osThreadId_t h)
{
    if (h == NULL) { printf("[RTOS] FAILED to create task %s (heap?)\r\n", name); return; }
    if (mon_count < 16) { mon[mon_count].name = name; mon[mon_count].h = h; mon_count++; }
}

void task_stackmon(void *argument);
const osThreadAttr_t stackmon_task_attributes = {
    .name = "stackmon",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityLow,
};

volatile char overflow_task[16];   /* survives in Live Expressions if the print fails */

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
    .stack_size = 512 * 4,   /* was 256*4: snprintf("%f") + interrupt frame overflowed it */
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
    .stack_size = 384 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t control_task_attributes = {
    .name = "control",
    .stack_size = 512 * 4,   /* was 256*4: printf with 8 floats + interrupt frame overflowed it */
    .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t gps_task_attributes = {
    .name = "gps",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t rc_task_attributes = {
    .name = "rc",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t telemetry_task_attributes = {
    .name = "telemetry",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityBelowNormal,
};

const osThreadAttr_t logger_task_attributes = {
    .name = "loggr",
    .stack_size = 512 * 4,               /* float printf needs room */
    .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t dshot_task_attributes = {
    .name = "dshot",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityRealtime,
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
	state_queue = osMessageQueueNew(1, sizeof(FlightState_t), NULL);
	control_state_queue = osMessageQueueNew(10, sizeof(FlightState_t), NULL);
	baro_queue = osMessageQueueNew(1, sizeof(BarData_t), NULL);
	gps_queue = osMessageQueueNew(1, sizeof(GPSData_t), NULL);
	rc_queue = osMessageQueueNew(5, sizeof(RCInput_t), NULL);
	telemetry_queue = osMessageQueueNew(1, sizeof(TelemetryPckt_t), NULL);
	log_queue = osMessageQueueNew(1, sizeof(CtrlLog_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  mon_add("sensor",  osThreadNew(task_sensor,   NULL, &sensor_task_attributes));
  mon_add("display", osThreadNew(task_display,  NULL, &display_task_attributes));
//  osThreadNew(task_log, NULL, &log_task_attributes);
  mon_add("wdog",    osThreadNew(task_watchdog, NULL, &watchdog_task_attributes));
  mon_add("filter",  osThreadNew(task_filter,   NULL, &filter_task_attributes));
  mon_add("baro",    osThreadNew(task_baro,     NULL, &baro_task_attributes));
  mon_add("control", osThreadNew(task_control,  NULL, &control_task_attributes));
  mon_add("gps",     osThreadNew(task_gps,      NULL, &gps_task_attributes));
  mon_add("rc",      osThreadNew(task_rc,       NULL, &rc_task_attributes));
  mon_add("default", defaultTaskHandle);
  mon_add("stackmon",osThreadNew(task_stackmon, NULL, &stackmon_task_attributes));
  mon_add("loggr", osThreadNew(task_loggr, NULL, &logger_task_attributes));
  mon_add("telem", osThreadNew(task_telemetry, NULL, &telemetry_task_attributes));
  mon_add("dshot", osThreadNew(task_dshot, NULL, &dshot_task_attributes));
  printf("[RTOS] heap free after init: %u bytes\r\n", (unsigned)xPortGetFreeHeapSize());
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
    osDelay(1000);   /* was osDelay(1): woke 1000x/s for nothing */
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Every fatal path: motors off FIRST, then take the UART back (a task may have
 * been mid-print, leaving huart3 locked), then report, then let the IWDG reset us. */
static void fatal_prepare(void)
{
    taskDISABLE_INTERRUPTS();
    TIM1->BDTR &= ~TIM_BDTR_MOE;          /* cut all four motor outputs */
    huart3.Lock   = HAL_UNLOCKED;
    huart3.gState = HAL_UART_STATE_READY;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    fatal_prepare();
    strncpy((char *)overflow_task, pcTaskName, sizeof(overflow_task) - 1);
    printf("[FATAL] Stack overflow in task: %s\r\n", pcTaskName);
    for (;;);                              /* IWDG resets; message repeats each time */
}

void vApplicationMallocFailedHook(void)
{
    fatal_prepare();
    printf("[FATAL] FreeRTOS heap exhausted\r\n");
    for (;;);
}

/* Prints each task's worst-case free stack every 5 s (integers only: small stack use). */
void task_stackmon(void *argument)
{
    for (;;)
    {
        osDelay(5000);
        osMutexAcquire(uart_mutex, osWaitForever);
        printf("[STACK] min free bytes:");
        for (int i = 0; i < mon_count; i++)
            printf(" %s:%lu", mon[i].name,
                   (unsigned long)uxTaskGetStackHighWaterMark((TaskHandle_t)mon[i].h) * 4UL);
        printf(" | heap min:%u\r\n", (unsigned)xPortGetMinimumEverFreeHeapSize());
        osMutexRelease(uart_mutex);
    }
}
volatile const char *assert_file = 0;
volatile int assert_line = 0;

void vAssertCalled(const char *file, int line)
{
    taskDISABLE_INTERRUPTS();
    assert_file = file;          // visible in Live Expressions even if the print fails
    assert_line = line;
    printf("[ASSERT] %s:%d\r\n", file, line);
    for (;;);                    // IWDG resets us — message repeats every boot
}
/* USER CODE END Application */
