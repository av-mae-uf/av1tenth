void sendMessage()
{
  byte sendMessageData[TX_PACKET_SIZE];
  byte stateValue = 0;
  uint16_t rpm_uint16 = 0;
  uint16_t usigned_heading = 0;
  
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
  
  rpm_uint16 = (int16_t)(RPM1 * 10.0);
  sendMessageData[2] = (rpm_uint16>>8)&0xFF; // High bits
  sendMessageData[3] = rpm_uint16&0xFF; // Low bits

  rpm_uint16 = (int16_t)(RPM2 * 10.0);
  sendMessageData[4] = (rpm_uint16>>8)&0xFF; // High bits
  sendMessageData[5] = rpm_uint16&0xFF; // Low bits

  // Assemble into uint16 then cast as signed int16
  sendMessageData[6] = (enu_heading>>8)&0xFF; // High bits
  sendMessageData[7] = enu_heading&0xFF; // Low bits

  CRC16 crc;
  crc.setPolynome(0x1021);
  crc.add(sendMessageData+1, (uint16_t)(TX_PACKET_SIZE-4));
  uint16_t crc16 =  crc.getCRC();

  sendMessageData[8] = (crc16>>8)&0xFF; // High bits
  sendMessageData[9] = crc16&0xFF; // Low bits
  
  sendMessageData[10] = 147;
  
  //Write data to serial
  Serial.write(sendMessageData, TX_PACKET_SIZE);
  
}