/*
 * ROSIFace.c
 *
 *  Created on: Aug 12, 2010
 *      Author: Titus
 */

#include "ROSIFace.h"
 
// Storage for current data from ros
static velocityCmd_t velocityCmd = {0, 0};
static imuData_t imuData = {0, 0, 0, 0, 0, 0, 0};
static gpsStatus_t gpsStatus = {0, 0, 0};
static gpsData_t gpsData = {0, 0, 0, 0}; 

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
    

}

void ROSProcessData(void)
{


}

velocityCmd_t ROSGetVelocityCmd(void)
{
    return velocityCmd;
}

imuData_t ROSGetImuData(void)
{
    return imuData;
}

gpsStatus_t ROSGetGpsStatus(void)
{
    return gpsStatus;
}

gpsData_t ROSGetGpsData(void)
{
    return gpsData;
}

