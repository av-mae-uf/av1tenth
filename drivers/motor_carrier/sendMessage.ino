void sendMessage()
{
  byte sendMessageData[TX_PACKET_SIZE];
  byte stateValue = 0;
  String crcString;
  
  // Gather information into writable format
  switch(State)
  {
    case ACTIVE:
      stateValue = 64;
      break;
    case INACTIVE:
      stateValue = 32;
      break;
  }

  sendMessageData[0] = 157;
  sendMessageData[1] = stateValue;
  //sendMessageData[2] = (byte)floor(throttlePercentEffort);
  //sendMessageData[3] = (byte)throttlePercentEffortDriver;

  // Sensor data is in int16 format when read from sensor.
  // Just transmit that and do the conversion on the receiving side.

  // Bit shift example for sending a int16
  // writeData[i] = tempUInt16 & 0xFF;
	// writeData[i+1] = (tempUInt16 >> 8) & 0xFF;

 
  uint16_t checksum =  checksumCalculator(sendMessageData+1, TX_PACKET_SIZE-3);

  // sendMessageData[4] = byte((unsigned int)crcString.toInt() % CRC_DIVIDER);
  sendMessageData[5] = 147;
  
  //Write data to serial
  Serial.write(sendMessageData, TX_PACKET_SIZE);
  
}

uint16_t checksumCalculator(uint8_t * data, uint16_t length)
{
  /*  Fletcher's Checksum algorithm. I have no idea what this is doing. 
   *  https://www.tutorialspoint.com/cyclic-redundancy-check-crc-in-arduino
   */
   uint16_t curr_crc = 0x0000;
   uint8_t sum1 = (uint8_t) curr_crc;
   uint8_t sum2 = (uint8_t) (curr_crc >> 8);
   int index;
   for(index = 0; index < length; index = index+1)
   {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}
