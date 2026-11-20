/*
 * task_display.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "app_types.h"
#include "app_globals.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "i2c.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>

#define DISPLAY_PERIOD_MS  200  //5hz

void task_display(void *argument)
{
    while (uart_mutex == NULL || i2c_mutex == NULL || state_queue == NULL || baro_queue == NULL) osDelay(1);

    osMutexAcquire(i2c_mutex, osWaitForever);
    ssd1306_Init();
    osMutexRelease(i2c_mutex);

    FlightState_t state = {0};
    GPSData_t gps = {0};
    BarData_t baro = {0};
    char buf[24];

    uint32_t next_wake = osKernelGetTickCount();
    for (;;)
    {
    	// Fixed-rate schedule: wake every 200 ms regardless of how long drawing took
    	next_wake += DISPLAY_PERIOD_MS;

        // Latest values; a failed peek leaves the previous value in place
        xQueuePeek((QueueHandle_t)state_queue, &state, 0);
        xQueuePeek((QueueHandle_t)baro_queue,  &baro,  0);
        xQueuePeek((QueueHandle_t)gps_queue,   &gps,   0);

        ssd1306_Fill(Black);

        ssd1306_SetCursor(0, 0);
        snprintf(buf, sizeof(buf), "R: %.1f deg, P: %.1f deg", state.roll, state.pitch);
        ssd1306_WriteString(buf, Font_7x10, White);


        ssd1306_SetCursor(0, 16);
        snprintf(buf, sizeof(buf), "Alt: %.1fm", baro.altitude);
        ssd1306_WriteString(buf, Font_7x10, White);


        ssd1306_SetCursor(0, 32);
        snprintf(buf, sizeof(buf), "Tmp: %.1fC", baro.temperature);
        ssd1306_WriteString(buf, Font_7x10, White);


        ssd1306_SetCursor(0, 48);
        if (gps.valid){
        	snprintf(buf, sizeof(buf), "Sats:%d", gps.satellites);
        }
        else{
        	snprintf(buf, sizeof(buf), "GPS: No Fix");
        }
        ssd1306_WriteString(buf, Font_7x10, White);

        ssd1306_SetCursor(90, 48);
        ssd1306_WriteString(armed ? "ARM" : "DIS", Font_6x8, White);

        // ---- Push to OLED: lock per page, not per frame ----
        for (uint8_t page = 0; page < SSD1306_HEIGHT / 8; page++)
        {
            osMutexAcquire(i2c_mutex, osWaitForever);
            ssd1306_UpdatePage(page);
            osMutexRelease(i2c_mutex);
            // sensor/baro tasks can grab the bus here between pages
        }

        osDelayUntil(next_wake);
    }
}
