//======================================================================================
//===============================Output Helper Functions================================
//======================================================================================
void setRCSpeed(byte rcSpeed)
{
  // Input is a byte value between 0-180
  servo1.setAngle(rcSpeed);
}


void setSteering(byte steeringAngle)
{
  // Input is a byte value between 0-180
  servo2.setAngle(steeringAngle);
}
