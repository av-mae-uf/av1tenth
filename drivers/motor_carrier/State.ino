void stateLoop(void)
{
  /* 
   *  This is where all important actions are taken.
   */

  updateState();

  // ============================= PERFORM PROPER ACTIONS BASED ON STATE ===========================
  switch (State)
  {
    case ACTIVE_STATE:
      setSteering(desiredSteeringAngle);
      setRCSpeed(desiredSpeed);
      break;

    case INACTIVE_STATE:
      // Defaults all controls to "zero" values.
      setSteering(90);  // Set to forward
      setRCSpeed(90);   // 0 throttle
      break;

    case LOW_BATTERY_STATE:
      // Disable motion
      setSteering(90);  // Set to forward
      setRCSpeed(90);   // 0 throttle
      // Stop sending data when low battery?
      break;
      
  } // End of switch case

} // End of function


//======================================================================================
//==============================State Transition Function===============================
//======================================================================================

void updateState(void)
{
  // Update desired state
  if(serialTimedOut){ desiredState = INACTIVE_STATE; }
  else{ desiredState = ACTIVE_STATE; }

  State = desiredState;

} // End of function
