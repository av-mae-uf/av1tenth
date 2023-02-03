
/*  Written by: Patrick Neal
 *  Email: neap@ufl.edu
 *  Last Updated: 1/26/2023
 *  
 *  To Do:
 *    - Add LED functionality to code
 *    - update sendMessage for more data
 *    - Can test the motor control without any of the data feedback.
 *    - Add functionality to get low battery, create new state for "critically" low battery
 *    
 *  Notes:
 *    * Allow the arduino some time after opening a serial connection. (like 2 seconds) 
 *    * Make sure to clear the TX buffer on the host computer when connecting to this sketch
 *
 */

// Servos and Encoder objects are initialized in this header
#include <ArduinoMotorCarrier.h>
#include <BNO055_support.h>

// ---- Pin Definitions ----
// PN - Adjust the pin mapping during testing
#define YELLOW_LED 0
#define GREEN_LED 0
#define RED_LED 0

// ---- DIO Debouncing ----
#define ON_COUNT 12
#define OFF_COUNT 30

#define ACTIVE_STATE      1
#define INACTIVE_STATE    2

// ---- Serial Communication Parameters ----
#define BAUDRATE        115200
#define TX_PACKET_SIZE  8
#define RX_PACKET_SIZE  5
#define CRC_DIVIDER     256
#define HEARTBEAT_TIMEOUT 3000 // Time in milliseconds that must pass before heart beat timeout is triggered

// ---Loop Timers----
#define STATE_TIMER 50   // Time in millseconds between each call of the state loop (~20Hz)
#define SEND_TIMER 50   // Time in millseconds between each call of sendMessage (~20Hz)

//======================================================================================
//===============================Global Variables=======================================
//======================================================================================
int State = INACTIVE_STATE;         // start in the inactive State, ignition is off
int desiredState = INACTIVE_STATE;  // stores the desired state based on state transition logic

// ---- Received From ROS2 Driver ----
byte desiredSteeringAngle = 0;             // 0-180 angle value from the ROS2 driver
byte desiredSpeed = 0;                     // 0 if no reverse requested from ROS2 driver, 1 if requested from ROS2 driver
byte led = 0;                       // 0 if no cruise control requested from ROS2 driver, 1 if requested from ROS2 driver
bool blinkLED = false;             // holds the "gear" desired by the ROS2 driver, 1, 2, or 3

double throttlePercentEffort  = 0;        // these are the global % effort variables used to smooth transitions between states
byte currentGear = 1;                     // holds the "gear" currently active 1, 2, or 3
byte brakeEngaged = false;

// ---- IMU Variables ----
struct bno055_t BNO;

struct bno055_gyro gyroData;
struct bno055_mag magData;
struct bno055_quaternion quatData;
struct bno055_linear_accel accelData;

// ---- Serial Variables ----
byte messageStarted = false;
byte messageComplete = false;  // whether the string is complete
byte serialTimedOut = false;
byte receivedMessage[RX_PACKET_SIZE-2];


//======================================================================================
//========================================Setup=========================================
//======================================================================================
void setup() 
{
  
  //Initialization of the BNO055
  BNO_Init(&BNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  // Does there need to be a delay after every set function call?
  delay(1);
  bno055_set_axis_remap_value(REMAP_X_Y);

  bno055_set_y_remap_sign(BNO055_REMAP_Y_SIGN__MSK);

  encoder1.resetCounter(0);
  
  // Initialize the serial
  Serial.begin(BAUDRATE);
}


//======================================================================================
//=========================================Loop=========================================
//======================================================================================
void loop() 
{
  static unsigned long lastStateTime = 0, lastSendTime = 0, lastReceivedMsgTime = 0;
  static bool isValidMsg = false;
  unsigned long currentTime = 0;

  if(messageComplete)
  { 
    isValidMsg = parseReceivedMessage(receivedMessage);
    if (isValidMsg){ lastReceivedMsgTime = millis(); }
  }
  
  currentTime = millis();
  if (currentTime >= (lastReceivedMsgTime + HEARTBEAT_TIMEOUT))
  {
    serialTimedOut = true;
  }

  if(currentTime >= (lastStateTime + STATE_TIMER))
  {
    stateLoop();
    lastStateTime = currentTime;
  }

  if(currentTime >= (lastSendTime + SEND_TIMER))
  {
    sensorUpdate();
    sendMessage();
    lastSendTime = currentTime;
  }
}
