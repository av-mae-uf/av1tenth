//======================================================================================
//============================Serial Message Parser Function============================
//======================================================================================
bool parseReceivedMessage(byte* message)
{
  String crcString;
  // Check CRC
  for(int i = 0; i < RX_PACKET_SIZE-3; i++)
  {
    crcString.concat(message[i]);
  }

  messageComplete = false;
  
  if(((unsigned int)crcString.toInt() % CRC_DIVIDER) == message[RX_PACKET_SIZE-3])
  {
    desiredSteeringAngle  = message[0]; // 0 to 180
    desiredSpeed          = message[1]; // 0 to 180

    sendResponse(true);
    return true;
  }
  else
  {
    // Did not pass the CRC test, data corrupted. Request another message.
    sendResponse(false);
    return false;
  }

}


//======================================================================================
//=================================Serial Event Function================================
//======================================================================================
// This whole function code cause asynchronous issues. Replacement message data while it is being read.
// or not storing all the data if it does not all arrive in a timely manner.
void serialEvent() 
{ 
  static int counter = 0;
  
  while (Serial.available()) 
  {
    // get the new byte:
    byte inByte = Serial.read();
    
    if(messageStarted) // start adding to received message
    {
      if(messageComplete){ messageComplete = false; counter = 0; } // If we got another message before last was read. reset flags and overwrite.
      
      if(inByte == 200 && counter == RX_PACKET_SIZE-2) // PN - Check the indexing 
      { 
        messageComplete = true;
        messageStarted = false;
        counter = 0;
      }
      else
      {
        receivedMessage[counter] = (byte)inByte;
        counter++;

      }
    }
    // Check if this is the beginning of a message
    else if(inByte == 199)
    { 
      messageStarted = true;
    }
  }
}


//======================================================================================
//====================================Helper Functions==================================
//======================================================================================
void sendResponse(bool Correct)
{
  Serial.write(137);
  if(Correct){ Serial.write(128); }
  else{ Serial.write(64); }
  Serial.write(127);
}
