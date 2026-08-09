/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "iwdg.h"
#include <stdio.h>
#include <math.h>
#include "app_types.h"
#include "app_globals.h"
#include "task_sensor.h"
#include "task_filter.h"
#include "task_display.h"
#include "task_log.h"
#include "task_watchdog.h"
#include "task_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESSED   1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

//void  task_sensor(void *argument);
////void task_print(void *argument);
//void task_display(void *argument);
//void task_log(void *argument);
//void task_watchdog(void *argument);
//void task_filter(void *argument);
//void task_blink_fast(void *argument);
//void task_blink_slow(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  printf("Booting...\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if (HAL_GPIO_ReadPin(B1_button_GPIO_Port, B1_button_Pin) == PRESSED){
//		  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
//	  }
//	  else{
//		  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
//	  }
//	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//void task_sensor(void *argument)
//{
//	while (uart_mutex == NULL || imu_queue == NULL || i2c_mutex == NULL) osDelay(1);
//
//	uint8_t raw[24];
//	IMUData_t data;
//
//	uint8_t wake = 0x00;
//	osMutexAcquire(i2c_mutex, osWaitForever);
//	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake, 1, HAL_MAX_DELAY);
//	osMutexRelease(i2c_mutex);
//
//	for (;;)
//	{
//		osMutexAcquire(i2c_mutex, osWaitForever);
//		HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, raw, 14, HAL_MAX_DELAY);
//		osMutexRelease(i2c_mutex);
//
//        data.accel_x = (int16_t)(raw[0] << 8 | raw[1]);
//        data.accel_y = (int16_t)(raw[2] << 8 | raw[3]);
//        data.accel_z = (int16_t)(raw[4] << 8 | raw[5]);
//
//        data.gyro_x = (int16_t)(raw[8]  << 8 | raw[9]);
//        data.gyro_y  = (int16_t)(raw[10] << 8 | raw[11]);
//        data.gyro_z  = (int16_t)(raw[12] << 8 | raw[13]);
//
//        osMessageQueuePut(imu_queue, &data, 0, 0);
//        osDelay(10); // 100Hz
//
//	}
//}
//
////void task_print(void *argument)
////{
////    while (uart_mutex == NULL || imu_queue == NULL) osDelay(1);
////
////    IMUData_t data;
////
////    for (;;)
////    {
////        // Block until a reading is available
////        if (osMessageQueueGet(imu_queue, &data, NULL, osWaitForever) == osOK)
////        {
////        	if (logging_enabled)
////        	{
////        		osMutexAcquire(uart_mutex, osWaitForever);
////        		printf("AX: %6d  AY: %6d  AZ: %6d\r\n",
////                   data.accel_x, data.accel_y, data.accel_z);
////        		osMutexRelease(uart_mutex);
////        	}
////        }
////    }
////}
//
//void task_display(void *argument)
//{
//    while (uart_mutex == NULL || i2c_mutex == NULL || attitude_queue == NULL) osDelay(1);
//
//    // Initialize OLED
//    osMutexAcquire(i2c_mutex, osWaitForever);
//    ssd1306_Init();
//    osMutexRelease(i2c_mutex);
//
//    Attitude_t att;
//    char buf[32];
//
//    for (;;)
//    {
//        if (osMessageQueueGet(attitude_queue, &att, NULL, osWaitForever) == osOK)
//        {
//            osMutexAcquire(i2c_mutex, osWaitForever);
//
//            ssd1306_Fill(Black);
//
//            ssd1306_SetCursor(0, 0);
//            snprintf(buf, sizeof(buf), "Roll: %.1f deg", att.roll);
//            ssd1306_WriteString(buf, Font_7x10, White);
//
//            ssd1306_SetCursor(0, 16);
//            snprintf(buf, sizeof(buf), "Pitch: %.1f deg", att.pitch);
//            ssd1306_WriteString(buf, Font_7x10, White);
//
//
//            ssd1306_UpdateScreen();
//
//            osMutexRelease(i2c_mutex);
//        }
//    }
//}
//
//void task_log(void *argument)
//{
//    while (uart_mutex == NULL || button_sem == NULL) osDelay(1);
//
//    for (;;)
//    {
//        // Block until button is pressed
//        osSemaphoreAcquire(button_sem, osWaitForever);
//
//        // Toggle logging state
//        logging_enabled = !logging_enabled;
//
//        osMutexAcquire(uart_mutex, osWaitForever);
//        if (logging_enabled)
//            printf("[LOG] Logging ON\r\n");
//        else
//            printf("[LOG] Logging OFF\r\n");
//        osMutexRelease(uart_mutex);
//    }
//}
//
//void task_watchdog(void *argument)
//{
//	for (;;)
//	{
//			HAL_IWDG_Refresh(&hiwdg);
//			osDelay(500);
//	}
//}
//
//void task_filter(void *argument)
//{
//	while (imu_queue == NULL || attitude_queue == NULL) osDelay(1);
//
//	IMUData_t imu;
//	Attitude_t att = {0.0f, 0.0f};
//
//	const float ACCEL_SCALE = 16384.0f;
//	const float GYRO_SCALE = 131.0f;
//	const float ALPHA = 0.98f;
//	const float dt = 0.01f;
//
//	for (;;)
//	{
//		if (osMessageQueueGet(imu_queue, &imu, NULL, osWaitForever) == osOK)
//		{
//			float ax = imu.accel_x / ACCEL_SCALE;
//		    float ay = imu.accel_y / ACCEL_SCALE;
//		    float az = imu.accel_z / ACCEL_SCALE;
//		    float gx = imu.gyro_x  / GYRO_SCALE;  // degrees/sec
//		    float gy = imu.gyro_y  / GYRO_SCALE;
//
//		    float roll_accel  = atan2f(ay, az) * 180.0f / 3.14159f;
//		    float pitch_accel = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / 3.14159f;
//
//		    // Complementary filter
//		    att.roll  = ALPHA * (att.roll  + gx * dt) + (1.0f - ALPHA) * roll_accel;
//		    att.pitch = ALPHA * (att.pitch + gy * dt) + (1.0f - ALPHA) * pitch_accel;
//
//		    osMessageQueuePut(attitude_queue, &att, 0, 0);
//	    }
//	}
//}

//void task_blink_fast(void *argument)
//{
////    uint32_t count = 0;
//    for (;;)
//    {
////    	osMutexAcquire(uart_mutex, osWaitForever);
////        HAL_UART_Transmit(&huart2, (uint8_t *)"[FAST]------\r\n", 14, HAL_MAX_DELAY);
////        osMutexRelease(uart_mutex);
////        osDelay(1);  // 1ms — hammering as fast as possible
//    }
//}

//void task_blink_slow(void *argument)
//{
////    uint32_t count = 0;
//    for (;;)
//    {
////    	osMutexAcquire(uart_mutex, osWaitForever);
////        HAL_UART_Transmit(&huart2, (uint8_t *)"[SLOW]......\r\n", 14, HAL_MAX_DELAY);
////        osMutexRelease(uart_mutex);
////        osDelay(1);
//    }
//}
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
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
