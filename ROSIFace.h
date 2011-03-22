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

#define CMD_VEL				103
#define ODOM_ENC			104
#define ODOM_COMB			105
#define CMD_BATTERY			106

#define SIZE_VEL			5
#define SIZE_ODOM_ENC		21
#define SIZE_ODOM_COMB		21
#define SIZE_BATTERY		7

#define SIZE_VEL_ARR        2
#define SIZE_ODOM_ARR		5

// Command Vel Array Indexes
#define ROS_LINEAR_VEL		0
#define ROS_ANGULAR_VEL		1

void ROSProcessPacket(void);
int8_t ROSChecksum(void);
void ROSProcessData(void);
void ROSBuildHeader(uint8_t dataSize);
int ROSCalcChkSum(uint8_t dataSize);
void ROSSendOdomEnc(int32_t x_mm, int32_t y_mm, int32_t th_mrad, \
		int32_t linVel, int32_t angVel);
void ROSSendBattery(uint16_t cell1_mv, uint16_t cell2_mv, uint16_t cell3_mv);
int16_t ROSGetVelocityCmd(uint8_t value);

#endif /* ROSIFACE_H_ */
