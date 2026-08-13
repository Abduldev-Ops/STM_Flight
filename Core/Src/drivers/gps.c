/*
 * gps.c
 *
 *  Created on: Aug 9, 2026
 *      Author: akanb
 */


#include "drivers/gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

float GPS_NMEAToDecimal(float nmea, char direction)
{
	int degrees = (int) (nmea / 100);
	float minutes = nmea - (degrees * 100);
	float decimal = degrees + (minutes / 60.0f);

	if (direction == 'S' || direction == 'W')
	{
		decimal = -decimal;
	}

	return decimal;
}

uint8_t GPS_ParseGGA(char *sentence, GPSData_t *gps)
{
	if (strncmp(sentence, "$GPGGA", 6) != 0 && strncmp(sentence, "$GNGGA", 6) != 0) return 0;

	char *token;
	char buf[128];
	strncpy(buf, sentence, sizeof(buf) -1);
	buf[sizeof(buf)-1] = '\0';

	char *fields[15];
	uint8_t field_count = 0;

	token = strtok(buf, ",");
	while (token != NULL && field_count < 15)
	{
		fields[field_count++] = token;
		token = strtok(NULL, ",");
	}

	if (field_count < 10) return 0;

	gps->fix = (uint8_t) atoi(fields[6]);
	if (gps->fix == 0)
	{
		gps->valid = 0;
		return 0;
	}

	gps->satellites = (uint8_t) atoi(fields[7]);

	float lat_nmea = atof(fields[2]);
	gps->latitude = GPS_NMEAToDecimal(lat_nmea, fields[3][0]);

	float long_nmea = atof(fields[4]);
	gps->longitude = GPS_NMEAToDecimal(long_nmea, fields[5][0]);

	gps->altitude = atof(fields[9]);
	gps->valid = 1;
	return 1;
}
