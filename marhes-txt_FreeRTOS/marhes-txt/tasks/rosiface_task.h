/**
 @file rosiface_task.h
  
 @brief Header for ROS Interface Packet Description
         
 Contains message types and sizes, and defines the packet structure with message
 and header typedefs.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#ifndef ROSIFACE_TASK_H_
#define ROSIFACE_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "app_types.h"
#include "drivers/pwm.h"
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
#define CMD_PWM_TEST      108
#define CMD_PID_TERMS     109
#define CMD_SWITCH_PWR    110
#define CMD_PWM           111

// Size doesn't include header or chksum
#define SIZE_VEL			    4
#define SIZE_ODOM_ENC		  20
#define SIZE_ODOM_COMB		20
#define SIZE_BATTERY		  4
#define SIZE_PID_RX			  24
#define SIZE_PWM_TEST		  20
#define SIZE_PID_TERMS    16
#define SIZE_SWITCH_PWR   2
#define SIZE_PWM          8

// Size of arrays
#define SIZE_VEL_ARR      2
#define SIZE_ODOM_ARR		  5
#define SIZE_PID_ARR		  6

// Command Vel Array Indexes
#define ROS_LINEAR_VEL		0
#define ROS_ANGULAR_VEL		1

#define PWR_SOURCE_ATX    0
#define PWR_SOURCE_ESC    1

/**
 @brief The header of the message.  Provides access to individual bytes of the 
        message header.
*/
#pragma pack(1)
typedef struct {
  uint8_t start_byte_1;           ///< The first start byte of the packet - 0xFA 
  uint8_t start_byte_2;           ///< The first start byte of the packet - 0xFB 
  uint8_t length;                 ///< The length of the data portion 
  uint8_t command;                ///< The command byte 
} header_t;
#pragma pack()

/**
 @brief The union of the header struct with a byte array.  Allows the header
        to be accessed byte name or by byte number.
*/
typedef union {
  header_t var;                   ///< Access to the header struct 
  uint8_t bytes[HEADER_SIZE];     ///< Access to numbered bytes
} header_u;

/**
 @brief The message structure includes the header, data and the checksum.
 @note The chksum value if not used. The checksum is normally part of the data.
*/
#pragma pack(1)
typedef struct {
  header_u header;                ///< The message header
  uint8_t data[MAX_DATA_SIZE];    ///< The data portion of the message
  uint16_t chksum;                ///< The checksum of the message
} msg_t;
#pragma pack()

/**
 @brief The union of the message structure with a byte array.  Allows the 
        message to be accessed by name or by byte number.
*/
typedef union {
  msg_t var;                      ///< Access to the message struct
  uint8_t bytes[MAX_PACKET_SIZE]; ///< Access to numbered bytes
} msg_u;

void ROSBuildHeader(msg_u * pmsg, uint8_t dataSize, uint8_t command);
uint16_t ROSCalcChkSum(msg_u * pmsg);
void ROSSendOdomEnc(msg_u * pmsg, \
                    int32_t x_mm, int32_t y_mm, int32_t th_mrad, \
		                int32_t linVel, int32_t angVel);
void ROSSendBattery(msg_u * pmsg, uint16_t batt1, uint16_t batt2);
void ROSSendPidTerms(msg_u * pmsg, \
                     int32_t pterm, int32_t iterm, int32_t dterm, \
		                 int32_t signal);
void ROSBuildHeader(msg_u * pmsg, uint8_t dataSize, uint8_t command);
void vRxTaskStart(void);
int8_t ROSChecksum(void);
void ROSProcessData(void);

#endif
