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
#include <stdio.h>

void task_display(void *argument)
{
    while (uart_mutex == NULL || i2c_mutex == NULL || state_queue == NULL || baro_queue == NULL) osDelay(1);

    osMutexAcquire(i2c_mutex, osWaitForever);
    ssd1306_Init();
    osMutexRelease(i2c_mutex);

    FlightState_t state = {0};
    GPSData_t gps = {0};
    BarData_t baro = {0};
    char buf[32];

    for (;;)
    {
    	osMessageQueueGet(baro_queue, &baro, NULL, 0);

        if (osMessageQueueGet(state_queue, &state, NULL, osWaitForever) == osOK)
        {
        	osMessageQueueGet(gps_queue, &gps, NULL, 0);
            osMutexAcquire(i2c_mutex, osWaitForever);
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
            if (gps.valid)
                snprintf(buf, sizeof(buf), "Sats:%d", gps.satellites);
            else
                snprintf(buf, sizeof(buf), "GPS: No Fix");
            ssd1306_WriteString(buf, Font_7x10, White);

            ssd1306_UpdateScreen();
            osMutexRelease(i2c_mutex);
        }
    }
}
