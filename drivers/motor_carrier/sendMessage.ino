void sendMessage()
{
  byte sendMessageData[TX_PACKET_SIZE];
  byte stateValue = 0;
  String crcString;
  
  // Gather information into writable format
  switch(State)
  {
    case ACTIVE_STATE:
      stateValue = 64;
      break;
    case INACTIVE_STATE:
      stateValue = 32;
      break;
  }

  sendMessageData[0] = 157;
  sendMessageData[1] = stateValue;
  //sendMessageData[2] = (byte)floor(throttlePercentEffort);
  //sendMessageData[3] = (byte)throttlePercentEffortDriver;
 
  // Create CRC string
  for (int i = 1; i < TX_PACKET_SIZE-2; i++)
  {
    crcString.concat(sendMessageData[i]);
  }

  sendMessageData[4] = byte((unsigned int)crcString.toInt() % CRC_DIVIDER);
  sendMessageData[5] = 147;
  
  //Write data to serial
  Serial.write(sendMessageData, TX_PACKET_SIZE);
  
}
