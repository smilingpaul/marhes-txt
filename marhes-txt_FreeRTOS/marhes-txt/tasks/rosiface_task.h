#ifndef ROSIFACE_TASK_H_
#define ROSIFACE_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "app_types.h"
#include "tasks/controller_task.h"
#include "tasks/mode_task.h"

#define MAX_PACKET_SIZE		255
#define MAX_DATA_SIZE		  (MAX_PACKET_SIZE - HEADER_SIZE - CHKSUM_SIZE )
#define COMMAND_SIZE      1
#define HEADER_SIZE			  4   // FA, FB, SIZE, CMD
#define CHKSUM_SIZE       2

#define CMD_VEL				    103
#define CMD_ODOM_ENC			104
#define CMD_ODOM_COMB     105
#define CMD_BATTERY			  106
#define CMD_PID_RX			  107

// Size doesn't include header or chksum
#define SIZE_VEL			    4
#define SIZE_ODOM_ENC		  20
#define SIZE_ODOM_COMB		20
#define SIZE_BATTERY		  4
#define SIZE_PID_RX			  24

// Size of arrays
#define SIZE_VEL_ARR      2
#define SIZE_ODOM_ARR		  5
#define SIZE_PID_ARR		  6

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
  header_u header;
  uint8_t data[MAX_DATA_SIZE];
  uint16_t chksum;
} msg_t;
#pragma pack()

typedef union {
  msg_t var;
  uint8_t bytes[MAX_PACKET_SIZE];
} msg_u;

void ROSBuildHeader(msg_u * pmsg, uint8_t dataSize, uint8_t command);
uint16_t ROSCalcChkSum(msg_u * pmsg);
void ROSSendOdomEnc(msg_u * pmsg, \
                    int32_t x_mm, int32_t y_mm, int32_t th_mrad, \
		                int32_t linVel, int32_t angVel);
void ROSSendBattery(msg_u * pmsg, uint16_t batt1, uint16_t batt2);
void ROSBuildHeader(msg_u * pmsg, uint8_t dataSize, uint8_t command);
void vRxTaskStart(void);
int8_t ROSChecksum(void);
void ROSProcessData(void);

#endif
