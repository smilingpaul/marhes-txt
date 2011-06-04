#include "tasks/rosiface_task.h"

extern xComPortHandle rosPortHandle;

static msg_u data;
int32_t pidVals[6];

void ROSBuildHeader(msg_u * pmsg, uint8_t dataSize, uint8_t command)
{
  pmsg->var.header.var.start_byte_1 = 0xFA;   // Write 2 header bytes
  pmsg->var.header.var.start_byte_2 = 0xFB;
  pmsg->var.header.var.length = dataSize;	    // Write the data byte count
  pmsg->var.header.var.command = command;	    // Write the data byte command
  
}

uint16_t ROSCalcChkSum(msg_u * pmsg)
{
  uint8_t *buffer = &pmsg->var.header.var.command;
  uint32_t c = 0, n;

  n = pmsg->var.header.var.length + COMMAND_SIZE; // Get the number of data bytes

  // For the even number of bytes, successively adding data byte pairs (high
  // byte first) to a running checksum (initially zero), disregarding sign and
  // overflow.
  while (n > 1)
  {
    c += (*(buffer)<<8) | *(buffer+1); // Add byte pair to checksum
    c = c & 0xffff;                    // Disregard overflow
    n -= 2;                            // Calc next pointer
    buffer += 2;
  }

  // If there are an odd number of data bytes, the last byte is XORed to the
  // low-order byte of the checksum.
  if (n>0) c = c ^ (uint32_t)*(buffer++);
  
  return((uint16_t)c);
}

void ROSSendOdomEnc(msg_u * pmsg, \
                    int32_t x_mm, int32_t y_mm, int32_t th_mrad, \
		                int32_t linVel, int32_t angVel)
{
	uint16_t checksum;
	ROSBuildHeader(pmsg, SIZE_ODOM_ENC, CMD_ODOM_ENC);    // Build header

	pmsg->var.data[0] = (uint8_t)(x_mm >> 24);      // Write x position
	pmsg->var.data[1] = (uint8_t)(x_mm >> 16);
	pmsg->var.data[2] = (uint8_t)(x_mm >> 8);
	pmsg->var.data[3] = (uint8_t)(x_mm >> 0);

	pmsg->var.data[4] = (uint8_t)(y_mm >> 24);      // Write y position
	pmsg->var.data[5] = (uint8_t)(y_mm >> 16);
	pmsg->var.data[6] = (uint8_t)(y_mm >> 8);
	pmsg->var.data[7] = (uint8_t)(y_mm >> 0);

	pmsg->var.data[8] = (uint8_t)(th_mrad >> 24);   // Write Orientation
	pmsg->var.data[9] = (uint8_t)(th_mrad >> 16);
	pmsg->var.data[10] = (uint8_t)(th_mrad >> 8);
	pmsg->var.data[11] = (uint8_t)(th_mrad >> 0);

	pmsg->var.data[12] = (uint8_t)(linVel >> 24);   // Write linear velocity
	pmsg->var.data[13] = (uint8_t)(linVel >> 16);
	pmsg->var.data[14] = (uint8_t)(linVel >> 8);
	pmsg->var.data[15] = (uint8_t)(linVel >> 0);

	pmsg->var.data[16] = (uint8_t)(angVel >> 24);   // Write angular velocity
	pmsg->var.data[17] = (uint8_t)(angVel >> 16);
	pmsg->var.data[18] = (uint8_t)(angVel >> 8);
	pmsg->var.data[19] = (uint8_t)(angVel >> 0);

	// Calculate checksum, Put the checksum bytes in reverse order
	checksum = ROSCalcChkSum(pmsg);                 
	pmsg->var.data[SIZE_ODOM_ENC] = checksum >> 8;
	pmsg->var.data[SIZE_ODOM_ENC + 1] = checksum & 0xFF;
	//	FIO0PIN ^= (1<<21);

	vSerialPutString( rosPortHandle, pmsg->bytes,  pmsg->var.header.var.length + HEADER_SIZE + CHKSUM_SIZE);
}

void ROSSendBattery(msg_u * pmsg, uint16_t batt1, uint16_t batt2)
{
	uint16_t checksum;
	ROSBuildHeader(pmsg, SIZE_BATTERY, CMD_BATTERY);                // Build header
	
	pmsg->var.data[0] = (uint8_t)(batt1 >> 8);
	pmsg->var.data[1] = (uint8_t)(batt1 >> 0);

	pmsg->var.data[2] = (uint8_t)(batt2 >> 8);
	pmsg->var.data[3] = (uint8_t)(batt2 >> 0);

	// Calculate checksum
	checksum = ROSCalcChkSum(pmsg);
	pmsg->var.data[SIZE_BATTERY] = checksum >> 8;		  // Put the checksum bytes in
	pmsg->var.data[SIZE_BATTERY+1] = checksum & 0xFF;	// reverse order

  vSerialPutString( rosPortHandle, pmsg->bytes,  pmsg->var.header.var.length + HEADER_SIZE + CHKSUM_SIZE);
	//FIO0PIN ^= (1<<21);
}

static void vRxTask( void *pvParameters )
{
  // Storage for leftover data
  static uint8_t dataNum = 0;
      
  for( ;; )
  {
    // While there is more data available in uart
    while ( dataNum < MAX_PACKET_SIZE )
    {                
      // Block until data is on the queue
      while(xSerialGetChar( rosPortHandle, &data.bytes[dataNum], portMAX_DELAY ) == pdFALSE);
  
      if(dataNum == 0)                    // Look for 0xFA
      {
        if(data.bytes[dataNum] == 0xFA)   // If 1st byte of header, get
          dataNum++;                      // second
      }
      else if (dataNum == 1)              // Look for 0xFB if proceeded by
      {                                   // 0xFA
        if(data.bytes[dataNum] == 0xFB)         // If 0xFB, get size from next
          dataNum++;                      // byte, else start over looking
        else                              // for 0xFA
          dataNum = 0;
      }
      else                                // Get the rest of the data
      {
        dataNum++;
        
        // If last byte, process cksum               
        if(dataNum >= (HEADER_SIZE + data.var.header.var.length + CHKSUM_SIZE))
        {                                 // then data.
          if(ROSChecksum())
            ROSProcessData();

          dataNum = 0;                    // If cksum doesn't match, 
                                          // restart the process
        }
      }
    }
    
    if ( dataNum >= MAX_PACKET_SIZE )
      dataNum = 0;
  }
}

void vRxTaskStart(void)
{
  xTaskCreate( vRxTask, "RxTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
}

int8_t ROSChecksum(void)
{
  uint8_t *buffer = &data.var.header.var.command;
  uint32_t checksum = 0;
  uint32_t n;

  // Get the number of data bytes to calculate the checksum
  n = data.var.header.var.length + COMMAND_SIZE;

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
  if (n>0) checksum = checksum ^ (uint32_t)*(buffer++);

  if ((uint8_t)(checksum>>8) == *(buffer++))
    if ((uint8_t)(checksum & 0xFF) == *(buffer++))
      return 1;
    
  return 0;
}

void ROSProcessData(void)
{
  switch(data.var.header.var.command)
  {
    case CMD_ODOM_COMB:
      if (data.var.header.var.length != SIZE_ODOM_COMB)
    	  break;
//    	odomCombined[0] = (data.var.data[0] << 24) | (data.var.data[1] << 16) | \
//    	                  (data.var.data[2] << 8)  | data.var.data[3];
//      odomCombined[1] = (data.var.data[4] << 24) | (data.var.data[5] << 16) | \
//    	                  (data.var.data[6] << 8)  | data.var.data[7];
//      odomCombined[2] = (data.var.data[8] << 24) | (data.var.data[9] << 16) | \
//                        (data.var.data[10] << 8) | data.var.data[11];
      ControllerSetOdomCombined( \
                      (data.var.data[12] << 24) | (data.var.data[13] << 16) | \
    		    				  (data.var.data[14] << 8)  | data.var.data[15], \
                      (data.var.data[16] << 24) | (data.var.data[17] << 16) | \
    		    				  (data.var.data[18] << 8)  | data.var.data[19]);
      vModeOdomCombRx();
//      FIO0PIN ^= (1<<21);
      break;
    case CMD_VEL:
      if (data.var.header.var.length != SIZE_VEL)
        break;
      // Store velocity command values
      ControllerSetLinearVelocity((int16_t)(data.var.data[0] << 8 | data.var.data[1]));
      ControllerSetAngularVelocity((int16_t)(data.var.data[2] << 8 | data.var.data[3]));
      vModeCmdVelRx();
      //FIO0PIN ^= (1<<21);
      break;
    case CMD_PID_RX:
      if (data.var.header.var.length != SIZE_PID_RX)
        break;

      pidVals[0] = (data.var.data[0] << 24)  | (data.var.data[1] << 16) | \
	                 (data.var.data[2] << 8)   | data.var.data[3];
      pidVals[1] = (data.var.data[4] << 24)  | (data.var.data[5] << 16) | \
	                 (data.var.data[6] << 8)   | data.var.data[7];
      pidVals[2] = (data.var.data[8] << 24)  | (data.var.data[9] << 16) | \
	                 (data.var.data[10] << 8)  | data.var.data[11];
      pidVals[3] = (data.var.data[12] << 24) | (data.var.data[13] << 16) | \
	                 (data.var.data[14] << 8)  | data.var.data[15];
      pidVals[4] = (data.var.data[16] << 24) | (data.var.data[17] << 16) | \
	                 (data.var.data[18] << 8)  | data.var.data[19];
      pidVals[5] = (data.var.data[20] << 24) | (data.var.data[21] << 16) | \
	                 (data.var.data[22] << 8)  | data.var.data[23];
      ControllerSetPid(pidVals[0], pidVals[1], pidVals[2], \
		               pidVals[3], pidVals[4], pidVals[5]);
      break;
    default:
      break;   
  }
}
