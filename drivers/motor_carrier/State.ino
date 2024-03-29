void stateLoop(void)
{
  /* 
   *  This is where all important actions are taken.
   */

  updateLEDs();

  updateState();

  //====================== PERFORM PROPER ACTIONS BASED ON STATE ======================
  switch (State)
  {
    case ACTIVE:
      setSteering(desiredSteeringAngle);
      setRCSpeed(desiredSpeed);
      break;

    case INACTIVE:
      // Defaults all controls to "zero" values.
      setSteering(90);  // Turn wheels 'straight'.
      setRCSpeed(90);   // 0 speed.
      break;
      
  } // End of switch case
} // End of function


//======================================================================================
//============================= State Transition Function ==============================
//======================================================================================

void updateState(void)
{
  // Update desired state
  if(serialTimedOut){ desiredState = INACTIVE; }
  else{ desiredState = ACTIVE; }

  State = desiredState;
}