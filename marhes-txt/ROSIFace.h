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
#include "controller.h"
#include "app_types.h"

#define MAX_PACKET_SIZE		255
#define MAX_DATA_SIZE		249
#define HEADER_SIZE			4

#define UART0

#define CMD_VEL				103
#define ODOM_ENC			104
#define ODOM_COMB			105
#define CMD_BATTERY			106
#define CMD_PID_RX			107

#define SIZE_VEL			5
#define SIZE_ODOM_ENC		21
#define SIZE_ODOM_COMB		21
#define SIZE_BATTERY		7
#define SIZE_PID_RX			25

#define SIZE_VEL_ARR        2
#define SIZE_ODOM_ARR		5
#define SIZE_PID_ARR		6

// Command Vel Array Indexes
#define ROS_LINEAR_VEL		0
#define ROS_ANGULAR_VEL		1

#pragma pack(1)
typedef struct {
  uint8_t start_byte_1;
  uint8_t start_byte_2;
  uint8_t length;
  uint8_t command;
} header_t;
#pragma pack()

typedef union {
  header_t var;
  uint8_t bytes[HEADER_SIZE];
} header_u;

#pragma pack(1)
typedef struct {
  header_t header;
  uint8_t command;
  uint8_t data[MAX_DATA_SIZE];
  uint16_t chksum;
} msg_t;
#pragma pack()

typedef union {
  msg_t var;
  uint8_t bytes[MAX_PACKET_SIZE];
} msg_u;

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
