/*
 * ROSIFace.h
 *
 *  Created on: Aug 12, 2010
 *      Author: Titus
 */

#ifndef ROSIFACE_H_
#define ROSIFACE_H_

#include "uart0.h"

#define MAX_PACKET_SIZE		255

// Divisor for making floats a hexadecimal representation
#define DIV_NUM		        32768000		
#define DIV_DEN				1000
#define DIV					(DIV_NUM / DIV_DEN)

// Define message commands and sizes
#define CMD_GPS_FIX			100
#define CMD_GPS_STATUS    	101
#define CMD_IMU_DATA		102
#define CMD_VEL				103

#define SIZE_GPS_FIX		19
#define SIZE_GPS_STATUS		7
#define SIZE_IMU_DATA		31
#define SIZE_VEL			5

#define USE_STRINGS

#define SIZE_VEL_ARR        2
#define SIZE_IMU_ARR        7
#define SIZE_GPS_DATA_ARR   4
#define SIZE_GPS_STAT_ARR   3

void ROSProcessPacket(void);
uint8_t ROSChecksum(void);
void ROSProcessData(void);
void ROSGetVelocityCmd(uint16_t *array);
void ROSGetImuData(float *array);
void ROSGetGpsStatus(uint16_t *array);
void ROSGetGpsData(float *array);

#endif /* ROSIFACE_H_ */
