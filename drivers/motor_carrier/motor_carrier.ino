
/*  Written by: Patrick Neal
 *  Email: neap@ufl.edu
 *  Last Updated: 2/6/2023
 *  
 *  To Do:
 *    - Add LED functionality to code
 *    - update sendMessage for more data
 *    - Can test the motor control without any of the data feedback.
 *    - Add functionality to get low battery, create new state for "critically" low battery
 *    - System needs to be Calibrated. Need to add software to describe this.
 *    
 *  Notes:
 *    * Allow the arduino some time after opening a serial connection. (like 2 seconds) 
 *    * Make sure to clear the TX buffer on the host computer when connecting to this sketch
 *
 */

// Servos and Encoder objects are initialized in this header
#include <ArduinoMotorCarrier.h>
#include <BNO055_support.h>
#include "CRC16.h"

// ---- Pin Definitions ----
// PN - Adjust the pin mapping during testing
#define YELLOW_LED_PIN  0
#define GREEN_LED_PIN   0
#define RED_LED_PIN     0

// ---- State/LED State ----
#define ACTIVE            1
#define INACTIVE          2
#define LOW_BATTERY       3
#define CRITICAL_BATTERY  4

// ---- Serial Communication Parameters ----
#define BAUDRATE        115200
#define TX_PACKET_SIZE  11
#define RX_PACKET_SIZE  6
#define HEARTBEAT_TIMEOUT 3000 // Time in milliseconds that must pass before heart beat timeout is triggered

// ---Loop Timers----
#define STATE_TIMER 50  // Time in milleseconds between each call of the state loop (~20Hz)
#define SEND_TIMER 50   // Time in milleseconds between each call of sendMessage (~20Hz)
#define PING_TIMER 100  // Time in millisecods between each controller.ping() (~20HZ)

//======================================================================================
//===============================Global Variables=======================================
//======================================================================================
int State = INACTIVE;         // start in the inactive State, ignition is off
int desiredState = INACTIVE;  // stores the desired state based on state transition logic

enum ledColor {
  GREEN = GREEN_LED_PIN,
  YELLOW = YELLOW_LED_PIN,
  RED = RED_LED_PIN
};

// ---- Received From ROS2 Driver ----
byte desiredSteeringAngle = 0;             // 0-180 angle value from the ROS2 driver
byte desiredSpeed = 0;                     // 0 if no reverse requested from ROS2 driver, 1 if requested from ROS2 driver

// ---- Encoder Variables ----
float RPM1;
float RPM2;

// ---- IMU Variables ----
struct bno055_t BNO;
struct bno055_gyro gyroData;
struct bno055_mag magData;
struct bno055_euler eulerData;
struct bno055_quaternion quatData;
struct bno055_linear_accel accelData;
uint16_t enu_heading;

bool criticalBattery = false;
bool disableIO = false;

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
  Wire.begin(); // Required for communication with the BNO055 sensor.

  //Initialization of the BNO055
  BNO_Init(&BNO); //Assigning the structure to hold information about the device
  delay(50);

  //Configuration to NDoF mode - Make any configuration changes before this command!
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(50);
  
  // Initialize the serial
  Serial.begin(BAUDRATE);

  encoder1.resetCounter(0);
  encoder2.resetCounter(0);
}


//======================================================================================
//=========================================Loop=========================================
//======================================================================================
void loop() 
{
  static unsigned long lastStateTime = 0, lastSendTime = 0, lastReceivedMsgTime = 0, lastPingTime = 0;
  static bool isValidMsg = false;
  unsigned long currentTime = 0;

  controller.ping();

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

  // Ping the controller. I have no idea if there is a desired rate.
  if(currentTime >= (lastPingTime + PING_TIMER))
  {
    controller.ping();
    lastPingTime = currentTime;
  }
}
