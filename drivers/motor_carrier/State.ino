void stateLoop(void)
{
  /* 
   *  This is where all important actions are taken.
   */

  enum ledColor color = YELLOW;
  bool blinking = false;

  updateState();

  // Delete me later
  byte ledState = 0;

  // ---- Determine LED configuration ----
  switch (ledState)
  {
    case ACTIVE:
      // Set the LED to green
      color = GREEN;
      blinking = false;
      break;

    case INACTIVE:
      // Set the LED to yellow
      color = YELLOW;
      blinking = false;
      break;
    
    case LOW_BATTERY:
      // Flash the red LED
      color = RED;
      blinking = true;
      break;

    case CRITICAL_BATTERY:
      // Turn off the LEDs
      //color = OFF;
      blinking = false;
      break;
  }

  // setLED(color, blinking);

  // ============================= PERFORM PROPER ACTIONS BASED ON STATE ===========================
  switch (State)
  {
    case ACTIVE:
      setSteering(desiredSteeringAngle);
      setRCSpeed(desiredSpeed);
      break;

    case INACTIVE:
      // Defaults all controls to "zero" values.
      setSteering(90);  // Set to forward
      setRCSpeed(90);   // 0 throttle
      break;

    case CRITICAL_BATTERY:
      // Disable motion
      setSteering(90);  // Set to forward
      setRCSpeed(90);   // 0 throttle
      servo1.detach();
      servo2.detach();
      disableIO = true;
      break;
      
  } // End of switch case

} // End of function


//======================================================================================
//==============================State Transition Function===============================
//======================================================================================

void updateState(void)
{
  // Update desired state
  if(criticalBattery){ State = CRITICAL_BATTERY; }
  else if(serialTimedOut){ State = INACTIVE; }
  else{ State = ACTIVE; }

  State = desiredState;

} // End of function
