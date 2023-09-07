
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

// ---- State/LED State ----
#define ACTIVE            1
#define INACTIVE          2
#define LOW_BATTERY       3
#define CRITICAL_BATTERY  4

// ---- Serial Communication Parameters ----
#define BAUDRATE        115200
#define TX_PACKET_SIZE  11
#define RX_PACKET_SIZE  8
#define HEARTBEAT_TIMEOUT 500 // Time in milliseconds that must pass before heart beat timeout is triggered

// ---Loop Timers----
#define STATE_TIMER 50  // Time in milleseconds between each call of the state loop (~20Hz)
#define SEND_TIMER 50   // Time in milleseconds between each call of sendMessage (~20Hz)
#define PING_TIMER 100  // Time in millisecods between each controller.ping() (~10HZ)

//======================================================================================
//===============================Global Variables=======================================
//======================================================================================
int State = INACTIVE;         // start in the inactive State, ignition is off
int desiredState = INACTIVE;  // stores the desired state based on state transition logic

enum ledConfiguration {
  OFF,
  GREEN ,
  YELLOW,
  RED,
  ALL
};

// ---- Received From ROS2 Driver ----
byte desiredSteeringAngle = 0;    // 0-180 angle value from the ROS2 driver
byte desiredSpeed = 0;            // 0-180. corresponds to different speed values for the ESC.
byte ledColor = 1;                // The desired configuration of LEDs. 0=OFF, 1=GREEN, 2=YELLOW, 3=RED, 4=ALL
bool ledBlinking = false;         // True if you want the specified ledColor to blink at a fixed rate.

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
byte serialTimedOut = true;
byte receivedMessage[RX_PACKET_SIZE-2];


//======================================================================================
//========================================Setup=========================================
//======================================================================================
void setup() 
{

  pinMode(LED_BUILTIN, OUTPUT);
  
  Wire.begin(); // Required for communication with the BNO055 sensor.

  //Initialization of the BNO055
  BNO_Init(&BNO); //Assigning the structure to hold information about the device
  delay(50);

  //Configuration to NDoF mode - Make any configuration changes before this command!
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(50);
  
  controller.begin();
  delay(50);
  
  controller.reboot();  
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
  static bool isValidMsg = false, ledState = false;
  unsigned long currentTime = 0;

  controller.ping();

  if(messageComplete)
  { 
    isValidMsg = parseReceivedMessage(receivedMessage);
    if (isValidMsg){ lastReceivedMsgTime = millis(); serialTimedOut = false;}
  }
  
  currentTime = millis();
  if (currentTime >= (lastReceivedMsgTime + HEARTBEAT_TIMEOUT))
  {
    serialTimedOut = true;
  }

  currentTime = millis();
  if(currentTime >= (lastStateTime + STATE_TIMER))
  {
    // Have to manually call the "serialEvent" for Nano 33 IoT because apparently it is not used internally.
    serialEvent();
    stateLoop();
    lastStateTime = currentTime;
  }

  currentTime = millis();
  if(currentTime >= (lastSendTime + SEND_TIMER))
  {
    sensorUpdate();
    sendMessage();
    lastSendTime = currentTime;
  }

  // Ping the controller. I have no idea if there is a desired rate.
  if(currentTime >= (lastPingTime + PING_TIMER))
  {
    if (ledState == true) { digitalWrite(LED_BUILTIN, LOW); ledState = false;}
    else { digitalWrite(LED_BUILTIN, HIGH); ledState = true;}
    lastPingTime = currentTime;
  }
  
  controller.ping();
}
