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
 
// Storage for current data from ROS
static int16_t velocityCmd[SIZE_VEL_ARR] = {0};
static uint32_t odomCombined[SIZE_ODOM_ARR] = {0};

// Storage for leftover data
static uint8_t data[MAX_PACKET_SIZE];
static uint8_t dataNum = 0;

// Send data building array
static uint8_t packet[MAX_PACKET_SIZE];

// Received ODOM_COMB Messges Count in #secs
uint16_t OdomCombRxCount = 0, CmdVelRxCount = 0;

// Processes data in the serial port buffer.
void ROSProcessPacket(void)
{
#ifdef UART0
    // While there is more data available in uart0
    while (Uart0RxDataReady() && dataNum < MAX_PACKET_SIZE)
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
    while (Uart2RxDataReady() && dataNum < MAX_PACKET_SIZE)
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

    if (dataNum >= MAX_PACKET_SIZE)
    	dataNum = 0;
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
	switch(data[3])
    {
    	case ODOM_COMB:
    		if (data[2] != SIZE_ODOM_COMB + 2)
    			break;

    		odomCombined[0] = (data[4] << 24) || (data[5] << 16) || \
    		    				(data[6] << 8) || data[7];
    		odomCombined[1] = (data[8] << 24) || (data[9] << 16) || \
    		    				(data[10] << 8) || data[11];
    		odomCombined[2] = (data[12] << 24) || (data[13] << 16) || \
    		    				(data[14] << 8) || data[15];
    		odomCombined[3] = (data[16] << 24) || (data[17] << 16) || \
    		    				(data[18] << 8) || data[19];
    		odomCombined[4] = (data[20] << 24) || (data[21] << 16) || \
    		    				(data[22] << 8) || data[23];
    		OdomCombRxCount++;
//    		FIO0PIN ^= (1<<21);
    		break;
        case CMD_VEL:
        	if (data[2] != SIZE_VEL + 2)
        		break;

            // Store velocity command values
            velocityCmd[0] = (int16_t)(data[4] << 8 | data[5]);
            velocityCmd[1] = (int16_t)(data[6] << 8 | data[7]);
            CmdVelRxCount++;
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

void ROSSendOdomEnc(int32_t x_mm, int32_t y_mm, int32_t th_mrad, \
		int32_t linVel, int32_t angVel)
{
	int checksum;

	ROSBuildHeader(SIZE_ODOM_ENC);			// Build header
	packet[3] = ODOM_ENC;				// Write message type

	packet[4] = (uint8_t)(x_mm >> 24);		// Write x position
	packet[5] = (uint8_t)(x_mm >> 16);
	packet[6] = (uint8_t)(x_mm >> 8);
	packet[7] = (uint8_t)(x_mm >> 0);

	packet[8] = (uint8_t)(y_mm >> 24);		// Write y position
	packet[9] = (uint8_t)(y_mm >> 16);
	packet[10] = (uint8_t)(y_mm >> 8);
	packet[11] = (uint8_t)(y_mm >> 0);

	packet[12] = (uint8_t)(th_mrad >> 24);	// Write Orientation
	packet[13] = (uint8_t)(th_mrad >> 16);
	packet[14] = (uint8_t)(th_mrad >> 8);
	packet[15] = (uint8_t)(th_mrad >> 0);

	packet[16] = (uint8_t)(linVel >> 24);	// Write linear velocity
	packet[17] = (uint8_t)(linVel >> 16);
	packet[18] = (uint8_t)(linVel >> 8);
	packet[19] = (uint8_t)(linVel >> 0);

	packet[20] = (uint8_t)(angVel >> 24);	// Write angular velocity
	packet[21] = (uint8_t)(angVel >> 16);
	packet[22] = (uint8_t)(angVel >> 8);
	packet[23] = (uint8_t)(angVel >> 0);

	// Calculate checksum
	checksum = ROSCalcChkSum(SIZE_ODOM_ENC);    	// Calculate the checksum
	packet[3+SIZE_ODOM_ENC] = checksum >> 8;		// Put the checksum bytes in
	packet[3+SIZE_ODOM_ENC+1] = checksum & 0xFF;	// reverse order

#ifdef UART0
	Uart0TxArr(packet, SIZE_ODOM_ENC + 5);
#else
	Uart2TxArr(packet, SIZE_ODOM_ENC + 5);
#endif
}

void ROSSendBattery(uint16_t cell1_mv, uint16_t cell2_mv, uint16_t cell3_mv)
{
	int checksum;

	ROSBuildHeader(SIZE_BATTERY);				// Build header
	packet[3] = CMD_BATTERY;					// Write message type

	packet[4] = (uint8_t)(cell1_mv >> 8);
	packet[5] = (uint8_t)(cell1_mv >> 0);

	packet[6] = (uint8_t)(cell2_mv >> 8);
	packet[7] = (uint8_t)(cell2_mv >> 0);

	packet[8] = (uint8_t)(cell3_mv >> 8);
	packet[9] = (uint8_t)(cell3_mv >> 0);

	// Calculate checksum
	checksum = ROSCalcChkSum(SIZE_BATTERY);    	// Calculate the checksum
	packet[3+SIZE_BATTERY] = checksum >> 8;		// Put the checksum bytes in
	packet[3+SIZE_BATTERY+1] = checksum & 0xFF;	// reverse order

#ifdef UART0
	Uart0TxArr(packet, SIZE_BATTERY + 5);
	FIO0PIN ^= (1<<21);
#else
	Uart2TxArr(packet, SIZE_BATTERY + 5);
#endif
}

int16_t ROSGetVelocityCmd(uint8_t value)
{
    return velocityCmd[value];
}
