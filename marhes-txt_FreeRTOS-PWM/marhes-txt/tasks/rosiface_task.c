/**
 @file rosiface_task.c
  
 @brief Parses or assembles messages from the ROS serial port.
 
 @par TX
 This code provides the functions needed to assemble and build different
 message types to send to the ROS computer. The checksums are also calculated
 here.
 
 @par RX
 A FreeRTOS task is made to received and parse messages and then update data.
 The task is blocked until data is on the RX queue and then it is processed
 accordingly.
 
 @par Message Format
 The message format is a simple header, a data section, and then a 2-byte 
 checksum. The following table shows this structure.  The checksum is the
 addition of the command byte to the last data byte in two byte pairs. If there
 is an odd byte at the end, exclusive or it to the lower byte of the checksum.
 <table border="0" rules="all" cellpadding="5">
   <tr>
     <th colspan="4">Header</th>
     <th>Data</th>
     <th>Checksum</th>
   </tr>
   <tr>
     <th>Start Byte 1 - 0xFA</th>
     <th>Start Byte 2 - 0xFB</th>
     <th>Data Length Byte</th>
     <th>Command Byte</th>
     <th>Data Bytes ( <= 249 )</th>
     <th>Checksum Bytes = 2</th> 
   </tr>
 </table> 
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#include "tasks/rosiface_task.h"

extern xComPortHandle rosPortHandle;
static msg_u data;

/**
 @brief The task to parse received data.
 
 This task runs continuously but is blocked infinitely when there is no data on
 the queue. When data arrives, it waits for the two start bytes. Then it 
 receives data until the all the data required by the data length byte and the 
 two byte checksum are received. Then the checksum is verified and if true, 
 the message is processed.
*/
static void vRxTask( void *pvParameters )
{
  // Storage for leftover data
  static uint8_t dataNum = 0;
  int32_t temp;
      
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
      else if (dataNum == 2)
      {
        if(data.bytes[dataNum] == 0x04)
          dataNum++;
        else
          dataNum = 0;
      }
      else if (dataNum == 3)
      {
        if(data.bytes[dataNum] == 0x55)
          dataNum++;
        else
          dataNum = 0;
      }
      else                                // Get the rest of the data
      {
        dataNum++;
                      
        if(dataNum >= 8)
        {
          FIO0PIN ^= (1<<21);
          temp = 72000 + (int16_t)((data.var.data[0] << 8) | data.var.data[1]);
          PWMSetDuty(4, temp);
          temp = (int16_t)((data.var.data[2] << 8) | data.var.data[3]);
          temp = 72000 - temp;
          PWMSetDuty(2, temp);
          temp = (int16_t)((data.var.data[2] << 8) | data.var.data[3]);
          temp = 72000 + temp;
          PWMSetDuty(3, temp);
          dataNum = 0;
        }
      }
    }
    
    if ( dataNum >= MAX_PACKET_SIZE )
      dataNum = 0;
  }
}

/**
 @brief Starts the receive task.
 @todo Should make the Priority and Stack Size reconfigurable.
*/
void vRxTaskStart(void)
{
  xTaskCreate( vRxTask, "RxTask", configMINIMAL_STACK_SIZE * 4, NULL, 4, NULL );
}
