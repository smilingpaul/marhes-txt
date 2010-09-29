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

typedef struct
{
    unsigned short linVel;
    unsigned short angVel;

}velocityCmd_t;

typedef struct
{
    float rotX;
    float rotY;
    float rotZ;
    float rotW;
    float linX;
    float linY;
    float linZ;
}imuData_t;

typedef struct
{
    unsigned short stat;
    unsigned short satsUsed;
    unsigned short satsVis;
}gpsStatus_t;

typedef struct
{
    float latitude;
    float longitude;
    float altitude;
    float heading
}gpsData_t;


void ROSProcessPacket(void)
uint8_t ROSChecksum(void);
void ROSProcessData(void);
velocityCmd_t ROSGetVelocityCmd(void);
imuData_t ROSGetImuData(void);
gpsStatus_t ROSGetGpsStatus(void);
gpsData_t ROSGetGpsData(void);

#endif /* ROSIFACE_H_ */
