/*
 * ROSIFace.h
 *
 *  Created on: Aug 12, 2010
 *      Author: Titus
 */

#ifndef ROSIFACE_H_
#define ROSIFACE_H_

#include "uart0.h"
#include "uart2.h"
#include "app_types.h"

#define MAX_PACKET_SIZE		255

#define UART0

// Divisor for making floats a hexadecimal representation
#define DIV_NUM		        32768000		
#define DIV_DEN				1000
#define DIV					(DIV_NUM / DIV_DEN)

// Define message commands and sizes
#define CMD_GPS_FIX			100
#define CMD_GPS_STATUS    	101
#define CMD_IMU_DATA		102
#define CMD_VEL				103
#define CMD_ENC_ODOM		104

#define SIZE_GPS_FIX		19
#define SIZE_GPS_STATUS		7
#define SIZE_IMU_DATA		31
#define SIZE_VEL			5
#define SIZE_ENC_ODOM		21

#define USE_STRINGS

#define SIZE_VEL_ARR        2
#define SIZE_IMU_ARR        7
#define SIZE_GPS_DATA_ARR   4
#define SIZE_GPS_STAT_ARR   3

#define SIZE_MAX_RX_STR     20

// Command Vel Array Indexes
#define ROS_LINEAR_VEL		0
#define ROS_ANGULAR_VEL		1

void ROSProcessPacket(void);
int8_t ROSChecksum(void);
void ROSProcessData(void);
void ROSBuildHeader(uint8_t dataSize);
int ROSCalcChkSum(uint8_t dataSize);
void ROSSendEncOdom(int32_t x_mm, int32_t y_mm, int32_t th_mrad, \
		int32_t linVel, int32_t angVel);
int16_t ROSGetVelocityCmd(uint8_t value);
float ROSGetImuData(uint8_t value);
char* ROSGetImuDataString(uint8_t value);
int16_t ROSGetGpsStatus(uint8_t value);
float ROSGetGpsData(uint8_t value);
char* ROSGetGpsDataString(uint8_t value);

#endif /* ROSIFACE_H_ */
