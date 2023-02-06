void setLED(enum ledColor color, bool blinking)
{
  static unsigned long lastBlinkTime = 0;
  static bool LEDState = false;

  if (blinking)
  {
    unsigned long currentTime = millis();
    if (currentTime >= (lastBlinkTime + 1000)) // 1 second bink
    {
      // Toggle LED state, digitalWrite, etc..
      lastBlinkTime = currentTime;
    }
  }
  else
  {
    // Set LED state
    // digitalWrite(ledPin, HIGH)
  }

}

void disableLEDs()
{
  /* Used to turn off all LED pins */
  // digitalWrite(GREEN_LED_PIN, LOW);
  // digitalWrite(YELLOW_LED_PIN, LOW);
  // digitalWrite(RED_LED_PIN, LOW);
}