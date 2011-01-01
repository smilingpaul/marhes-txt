/*! \file ROSIFace.c
    \brief An interface between the TXT ROS driver and the TXT controller board.

	Created on: Aug 12, 2010
	Author: Titus Appel
	Institution: UNM-ECE Department

    ROSIFace handles the communication with ROS.  It accepts velocity and data
    commands from ROS and extracts the needed data from the data packets.  It
    also sends odometry and battery information to the ROS driver.
*/


#include "ROSIFace.h"
 
// Storage for current data from ros
static int16_t velocityCmd[SIZE_VEL_ARR] = {0};
static float imuData[SIZE_IMU_ARR] = {0};
static char imuDataStrings[SIZE_IMU_ARR][SIZE_MAX_RX_STR] = {{0}};
static int16_t gpsStatus[SIZE_GPS_STAT_ARR] = {0};
static float gpsData[SIZE_GPS_DATA_ARR] = {0}; 
static char gpsDataStrings[SIZE_GPS_DATA_ARR][SIZE_MAX_RX_STR] = {{0}};

// Storage for leftover data
static uint8_t data[MAX_PACKET_SIZE];
static uint8_t dataNum = 0;

// Send data building array
static uint8_t packet[MAX_PACKET_SIZE];

// Processes data in the serial port buffer.
void ROSProcessPacket(void)
{
#ifdef UART0
    // While there is more data available in uart0
    while (Uart0RxDataReady())
    {                
        if(dataNum == 0)                        // Look for 0xFA
        {
            
            data[dataNum] = Uart0RxChar();      // Get next byte
            if(data[dataNum] == 0xFA)           // If 1st byte of header, get
                dataNum++;                      // second
        }
        else if (dataNum == 1)                  // Look for 0xFB if proceeded by
        {                                       // 0xFA
            data[dataNum] = Uart0RxChar();      // Get next byte
            if(data[dataNum] == 0xFB)           // If 0xFB, get size from next
                dataNum++;                      // byte, else start over looking
            else                                // for 0xFA
                dataNum = 0;
        }
        else if (dataNum == 2)                  // Get size of packet
        {
            data[dataNum] = Uart0RxChar();
            dataNum++;
        }
        else                                    // Get the rest of the data
        {
            data[dataNum] = Uart0RxChar();      // Get next byte 
            dataNum++;
                   
            if(dataNum >= (data[2] + 3))        // If last byte, process cksum
            {                                   // then data.
                if(ROSChecksum())
                    ROSProcessData();
                
                dataNum = 0;                    // If cksum doesn't match, 
                                                // restart the process
            }
        }      
    }
#else
    // While there is more data available in uart2
    while (Uart2RxDataReady())
    {
        if(dataNum == 0)                        // Look for 0xFA
        {

            data[dataNum] = Uart2RxChar();      // Get next byte
            if(data[dataNum] == 0xFA)           // If 1st byte of header, get
                dataNum++;                      // second
        }
        else if (dataNum == 1)                  // Look for 0xFB if proceeded by
        {                                       // 0xFA
            data[dataNum] = Uart2RxChar();      // Get next byte
            if(data[dataNum] == 0xFB)           // If 0xFB, get size from next
                dataNum++;                      // byte, else start over looking
            else                                // for 0xFA
                dataNum = 0;
        }
        else if (dataNum == 2)                  // Get size of packet
        {
            data[dataNum] = Uart2RxChar();
            dataNum++;
        }
        else                                    // Get the rest of the data
        {
            data[dataNum] = Uart2RxChar();      // Get next byte
            dataNum++;

            if(dataNum >= (data[2] + 3))        // If last byte, process cksum
            {                                   // then data.
                if(ROSChecksum())
                    ROSProcessData();

                dataNum = 0;                    // If cksum doesn't match,
                                                // restart the process
            }
        }
    }
#endif
}

int8_t ROSChecksum(void)
{
    uint8_t *buffer = &data[3];
  	int checksum = 0;
  	int n;

  	n = data[2] - 2;						    // Get the number of data bytes
	                                            // Minus the checksum
	// For the even number of bytes, successively adding data byte pairs (high 
	// byte first) to a running checksum (initially zero), disregarding sign and 
	// overflow.
  	while (n > 1) 
	{
    	checksum+= (*(buffer)<<8) | *(buffer+1);    // Add byte pair to checksum
    	checksum = checksum & 0xffff;				// Disregard overflow
    	n -= 2;								        // Calc next pointer
    	buffer += 2;
  	}
	
	// If there are an odd number of data bytes, the last byte is XORed to the 
	// low-order byte of the checksum.
  	if (n>0) checksum = checksum ^ (int)*(buffer++);

    if ((uint8_t)(checksum>>8) == *(buffer++))
        if ((uint8_t)(checksum & 0xFF) == *(buffer++))
            return 1;
    
    return 0;
}

void ROSProcessData(void)
{
    char tempStr[15];    
    uint8_t valueLoop, dataLoop, strPtr, size;
    
    switch(data[3])
    {
        case CMD_GPS_FIX:
        
            break;
        case CMD_GPS_STATUS:
        
            break;
        case CMD_IMU_DATA:
            dataLoop = 4;
            size = data[2] - 2;
         
            for(valueLoop = 0; valueLoop < SIZE_IMU_ARR; valueLoop++)
            {
                strPtr = 0;
                while(data[dataLoop] != ',' && dataLoop < size)
                {
                    tempStr[strPtr] = data[dataLoop];
                    dataLoop++;  
                    strPtr++;              
                }
                tempStr[strPtr + 1] = 0;
                //imuData[valueLoop] = atof(tempStr);
                //strcpy(imuDataStrings[valueLoop], tempStr);
                dataLoop++;
            }
            break;
        case CMD_VEL:
            // Check size of packet
            if (data[2] != SIZE_VEL + 2)
                break;
            // Store velocity command values
            velocityCmd[0] = (int16_t)(data[4] << 8 | data[5]);
            velocityCmd[1] = (int16_t)(data[6] << 8 | data[7]);
            FIO0PIN ^= (1<<21);
            break;
        default:
        
            break;   
    }
}

void ROSBuildHeader(uint8_t dataSize)
{
	packet[0] = 0xFA;				// Write 2 header bytes
    packet[1] = 0xFB;

	packet[2] = dataSize + 2;		// Write the length of packet, which is
									// everything from the command byte to the
                                    // checksum
}

int ROSCalcChkSum(uint8_t dataSize)
{
	uint8_t *buffer = &packet[3];
  	int c = 0;
  	int n;

  	n = dataSize;							// Get the number of data bytes

	// For the even number of bytes, successively adding data byte pairs (high
	// byte first) to a running checksum (initially zero), disregarding sign and
	// overflow.
  	while (n > 1)
	{
    	c+= (*(buffer)<<8) | *(buffer+1);	// Add byte pair to checksum
    	c = c & 0xffff;						// Disregard overflow
    	n -= 2;								// Calc next pointer
    	buffer += 2;
  	}

	// If there are an odd number of data bytes, the last byte is XORed to the
	// low-order byte of the checksum.
  	if (n>0) c = c ^ (int)*(buffer++);

  	return(c);
}

void ROSSendEncOdom(int32_t x_mm, int32_t y_mm, int16_t th_mrad, \
		int16_t linVelX, int16_t linVelY, int16_t angVel)
{
	int checksum;

	ROSBuildHeader(SIZE_ENC_ODOM);			// Build header
	packet[3] = CMD_ENC_ODOM;				// Write message type

	packet[4] = (uint8_t)(x_mm >> 24);		// Write x position
	packet[5] = (uint8_t)(x_mm >> 16);
	packet[6] = (uint8_t)(x_mm >> 8);
	packet[7] = (uint8_t)(x_mm >> 0);

	packet[8] = (uint8_t)(y_mm >> 24);		// Write y position
	packet[9] = (uint8_t)(y_mm >> 16);
	packet[10] = (uint8_t)(y_mm >> 8);
	packet[11] = (uint8_t)(y_mm >> 0);

	packet[12] = (uint8_t)(th_mrad >> 8);	// Write orientation
	packet[13] = (uint8_t)th_mrad;

	packet[14] = (uint8_t)(linVelX >> 8);	// Write linear x velocity
	packet[15] = (uint8_t)linVelX;

	packet[16] = (uint8_t)(linVelY >> 8);	// Write linear y velocity
	packet[17] = (uint8_t)linVelY;

	packet[18] = (uint8_t)(angVel >> 8);	// Write angular velocity
	packet[19] = (uint8_t)angVel;

	// Calculate checksum
	checksum = ROSCalcChkSum(SIZE_ENC_ODOM);    	// Calculate the checksum
	packet[3+SIZE_ENC_ODOM] = checksum >> 8;		// Put the checksum bytes in
	packet[3+SIZE_ENC_ODOM+1] = checksum & 0xFF;	// reverse order

	Uart2TxArr(packet, SIZE_ENC_ODOM + 5);
}

int16_t ROSGetVelocityCmd(uint8_t value)
{
    return velocityCmd[value];
}

float ROSGetImuData(uint8_t value)
{
    return imuData[value];
}

char* ROSGetImuDataString(uint8_t value)
{
    return imuDataStrings[value];
}

int16_t ROSGetGpsStatus(uint8_t value)
{
    return gpsStatus[value];
}

float ROSGetGpsData(uint8_t value)
{
    return gpsData[value];
}

char* ROSGetGpsDataString(uint8_t value)
{
    return gpsDataStrings[value];
}
