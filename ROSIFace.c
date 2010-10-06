/*
 * ROSIFace.c
 *
 *  Created on: Aug 12, 2010
 *      Author: Titus
 */

#include "ROSIFace.h"
 
// Storage for current data from ros
static uint16_t velocityCmd[SIZE_VEL_ARR] = {0};
static float imuData[SIZE_IMU_ARR] = {0};
static uint16_t gpsStatus[SIZE_GPS_STAT_ARR] = {0};
static float gpsData[SIZE_GPS_DATA_ARR] = {0}; 

// Storage for leftover data
static uint8_t data[MAX_PACKET_SIZE];
static uint8_t dataNum = 0;

// Processes data in the serial port buffer.
void ROSProcessPacket(void)
{
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
}

uint8_t ROSChecksum(void)
{
    unsigned char *buffer = &data[3];
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

    if ((unsigned char)(checksum>>8) == *(buffer++))
        if ((unsigned char)(checksum & 0xFF) == *(buffer++))
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
                imuData[valueLoop] = atof(tempStr);
                dataLoop++;
            }
            FIO0PIN ^= (1<<21);
            break;
        case CMD_VEL:
            // Check size of packet
            if (data[2] != SIZE_VEL + 2)
                break;
            // Store velocity command values
            velocityCmd[0] = data[4] << 8 | data[5];
            velocityCmd[1] = data[6] << 8 | data[7]; 
            break;
        default:
        
            break;   
    }
}

void ROSGetVelocityCmd(uint16_t *array)
{
    uint8_t i = 0;
    while(i < SIZE_VEL_ARR)
    {
        *array++ = velocityCmd[i];
        i++;
    }
}

void ROSGetImuData(float *array)
{
    uint8_t i = 0;
    while(i < SIZE_IMU_ARR)
    {
        *array++ = imuData[i];
        i++;
    }
}

void ROSGetGpsStatus(uint16_t *array)
{
    uint8_t i = 0;
    while(i < SIZE_GPS_STAT_ARR)
    {
        *array++ = gpsStatus[i];
        i++;
    }
}

void ROSGetGpsData(float *array)
{
    uint8_t i = 0;
    while(i < SIZE_GPS_DATA_ARR)
    {
        *array++ = gpsData[i];
        i++;
    }
}

