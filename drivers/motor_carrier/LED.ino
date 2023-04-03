void updateLEDs()
{
  // M1(0) is Yellow LED, M2(1) is Green LED, M3(2) is Red LED

  static unsigned long lastBlinkTime = 0;
  static uint8_t motorState[3] = {0};
  static bool ledOn = false;

  switch (ledColor)
  {
    case OFF:
      motorState[0] = 0;
      motorState[1] = 0;
      motorState[2] = 0;          
      break;

    case GREEN:
      motorState[0] = 0;
      motorState[1] = 30;
      motorState[2] = 0;  
      break;
    
    case YELLOW:
      motorState[0] = 30;
      motorState[1] = 0;
      motorState[2] = 0;  
      break;

    case RED:
      motorState[0] = 0;
      motorState[1] = 0;
      motorState[2] = 30;  
      break;

    case ALL:
      motorState[0] = 30;
      motorState[1] = 30;
      motorState[2] = 30;
      break;
  }

  if (ledBlinking == true)
  {
    unsigned long currentTime = millis();
    if (currentTime >= (lastBlinkTime + 1000)) // 1 second bink
    {
      if (ledOn == false)
      {
        M1.setDuty(motorState[0]);
        M2.setDuty(motorState[1]);
        M3.setDuty(motorState[2]);
        ledOn = true;
      }
      else
      {
        M1.setDuty(0);
        M2.setDuty(0);
        M3.setDuty(0);
        ledOn = false; 
      }
      lastBlinkTime = currentTime;
    }
  }
  else
  {
      M1.setDuty(motorState[0]);
      M2.setDuty(motorState[1]);
      M3.setDuty(motorState[2]);
      ledOn = false; // Set this so that blinking will work if set later.
  }
}
