/*
 * gps.h
 *
 *  Created on: Aug 9, 2026
 *      Author: akanb
 */

#ifndef DRIVERS_GPS_H
#define DRIVERS_GPS_H

#include "app_types.h"
#include <stdint.h>

uint8_t GPS_ParseGGA(char *sentence, GPSData_t *gps);
float GPS_NMEAToDecimal(float nmea, char direction);

#endif /* INC_DRIVERS_GPS_H_ */
