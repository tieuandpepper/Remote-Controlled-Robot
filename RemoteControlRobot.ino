/* Remote Control Robot
 * Group 15: Kevin Tieu, Kenneth Thomas, Jason Shi
 * Advanced Microprocessor Systems
 */

/*____________________________________________________________________________________________________________________
 *                 +-----------------------+  
 *                 |    Wiring diagram     |
 *                 |  Motor Driver DRV8835 |
 *                 +-----------------------+ 
 *                      
 *                     +------------+
 *                GND -|-GND    GND-|- GND
 *           (2V-11V) -|-VIN    VCC-|- (2V-7V)
 *   LEFT WHEEL FRONT -|-O2  IN2 EN-|- Pin 11 (PWM)
 *    LEFT WHEEL REAR -|-O1  IN1 PH-|- Pin 10
 *   RIGHT WHEEL REAR -|-O2  IN2 EN-|- Pin 9 (PWM)
 *  RIGHT WHEEL FRONT -|-O1  IN1 PH-|- Pin 8
 *                    -|-VM      MD-|- Pin 7
 *                     +------------+
 *____________________________________________________________________________________________________________________                     
 *
 *                 +----------------------+
 *                 |     Wiring Diagram   |
 *                 |       HM-10 BLE      |
 *                 +----------------------+
 *                 
 *                        +--------+
 *                        | STATE -|-  
 *                        |   RXD -|- Pin 13
 *                        |   TXD -|- Pin 12
 *                        |   GND -|- GND
 *                        |   VCC -|- 3.3V
 *                        |    EN -|-
 *                        +--------+
 *____________________________________________________________________________________________________________________             
 *
 *                 +----------------------+
 *                 |     Wiring Diagram   |
 *                 |          LED         |
 *                 +----------------------+
 *                 
 *                 HEADLIGHT
 *                    |__LEFT HALF - PIN 3
 *                    |__RIGHT HALF - PIN 2
 *                 
 *                 TAIL LIGHT
 *                    |__LEFT - PIN 6
 *                    |__RIGHT - PIN 5
 *                                      
 *____________________________________________________________________________________________________________________
 *
 */
// bluetooth modules
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
// FreeRTOS
#include <Arduino_FreeRTOS.h>
/*----------------------------------------------------------------------------------------------------------*/
// Motor Driver pins
#define LEFT_WHEEL_ENABLE    11
#define LEFT_WHEEL_PHASE     10
#define RIGHT_WHEEL_ENABLE   9
#define RIGHT_WHEEL_PHASE    8
#define MOTOR_MODE           7

/*----------------------------------------------------------------------------------------------------------*/
// LED pins
#define HEADLIGHT_LEFT      3
#define HEADLIGHT_RIGHT     2
#define TAIL_LIGHT_LEFT     6
#define TAIL_LIGHT_RIGHT    5

/*----------------------------------------------------------------------------------------------------------*/
// Bluetooth Pins
#define BLUETOOTH_TX         12
#define BLUETOOTH_RX         13
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(bluetooth); // pass reference of bluetooth object to ArduinoBlue constructor.
int getThrottleBluetooth(void);
int getSteeringBluetooth(void);
/*----------------------------------------------------------------------------------------------------------*/
// MOTOR control constants
const int SPEED_MAX = 255;
const int SPEED_MIN =   0;

// Motor status
// Direction: stop, forward, reverse
// Steering: straight, left, right
String motorDirection = "stop";
String motorSteering = "straight";

// Motor control functions
void MotorStop(void);
void MotorForward(void);
void MotorReverse(void);
void MotorLeftSpeed(int);
void MotorRightSpeed(int);
void MotorControl(int, int);

void TaskMotorControl(void*);
/*----------------------------------------------------------------------------------------------------------*/
// Headlight and tail light controls
void LightBlink(uint8_t);
void LightOn(uint8_t);
void LightOff(uint8_t);
void TaskLightControl(void*);
/*----------------------------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(9600);
  // start bluetooth module at 9600 baud rate
  bluetooth.begin(9600);
  // delay just in case bluetooth module needs time to "get ready".
  delay(100);
  // put your setup code here, to run once:
  // Setting up tasks, other initializations are run in tasks
  xTaskCreate(TaskMotorControl,                     // Point to TaskMotorControl function
              "TaskMotorControl",                   // Task name
              128,                                  // number of words (32 bits/ 4 bytes) for the task's stack
              NULL,                                 // task input parameter
              2,                                    // priority number (0 is lowest)
              NULL);                                // task's handle  
  
  xTaskCreate(TaskLightControl,                     // Point to TaskLightControl function
              "TaskLightControl",                   // Task name
              128,                                  // number of words (32 bits/ 4 bytes) for the task's stack
              NULL,                                 // task input parameter
              1,                                    // priority number (0 is lowest)
              NULL);                                // task's handle  
  
  // Start the scheduler
  vTaskStartScheduler();
}
/*----------------------------------------------------------------------------------------------------------*/
void loop()
{
// nothing here: scheduler takes care of running tasks
}

/* 
 * ==================================================================================================================
 *                                         BLUETOOTH CONTROL Functions
 * ==================================================================================================================
 */
/*
 * Retrieve throttle data of the joystick from ArduinoBlue 
 * Original throttle range: 0-99
 * Return: throttle Percentage (-98-100)
 * 
 *  THROTTLE |  THROTTLE PERCENTAGE  |    TRANSLATION    
 *  ---------+-----------------------+---------------------
 *     0     |         -98           |  SPEED_MAX REVERSE
 *     49    |          0            |        STOP
 *     99    |         100           |  SPEED_MAX FORWARD
 */ 
int getThrottleBluetooth(void) {
   int throttle = phone.getThrottle();
   throttle = (throttle-49)*2;
   return throttle;
}

/*
 * Retrieve steering data of the joystick from ArduinoBlue
 * Original steering range: 0-99
 * Return: steering percentage (-98-100)
 * 
 *  STEERING |  STEERING PERCENTAGE  |    TRANSLATION    
 *  ---------+-----------------------+---------------------
 *     0     |         -98           |  TURN LEFT (MAX ANGLE)
 *     49    |          0            |      STRAIGHT
 *     99    |         100           |  TURN RIGHT (MAX ANGLE)
 */
int getSteeringBluetooth(void) {
  int steering = phone.getSteering();
  steering = (steering-49)*2;
  return steering;
}
/* 
 * ==================================================================================================================
 *                                            MOTOR CONTROL TASK
 * ==================================================================================================================
*/
/*
 * Read joystick input from bluetooth on ArduinoBlue App --> getSteeringBluetooth and getThrottleBluetooth functions
 * Write to the motor driver to change direction/speed of robot --> MotorControl function
 * Task delay: 20ms
 * Priority: 2
 */
void TaskMotorControl(void* voidParameter) {
  int steering = 0;
  int throttle = 0;
  // Initialization OUTPUT pins
  pinMode(RIGHT_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_WHEEL_PHASE, OUTPUT);
  pinMode(LEFT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_WHEEL_PHASE, OUTPUT);
  pinMode(MOTOR_MODE, OUTPUT);   

  /*
   * Set up motor driver
   * MODE: ENABLE/PHASE
   * Direction: Forward
   * Speed: 0
   */
  digitalWrite(MOTOR_MODE, HIGH); // choose ENABLE/PHASE MODE
  MotorForward();
  MotorStop();
  
  while (1) {
    steering = getSteeringBluetooth();
    throttle = getThrottleBluetooth();
    MotorControl(throttle, steering);
    //Delay task 20ms
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

/* 
 * ==================================================================================================================
 *                                        MOTOR CONTROL FUNCTIONS
 * ==================================================================================================================
*/

/* Set Mode to 1(MD=1) to choose PHASE/ENABLE mode
 *  
 *  PHASE   ENABLE  |  OUT1    OUT2  |  OPERATING MODE    
 *  ----------------+----------------+-----------------------------
 *    0       PWM   |  PWM      L    |  FORWARD/brake at speed PWM%   
 *    1       PWM   |   L      PWM   |  REVERSE/brake at speed PWM%
 *    X        0    |   L       L    |  brake low
*/
/*----------------------------------------------------------------------------------------------------------*/
/*
 * Stop motor
 * Both PWMs: 0
 * Direction: unchanged
 */
void MotorStop(void){
  motorDirection = "stop";
  motorSteering = "straight";
  analogWrite(LEFT_WHEEL_ENABLE,SPEED_MIN);
  analogWrite(RIGHT_WHEEL_ENABLE,SPEED_MIN);
}
/*----------------------------------------------------------------------------------------------------------*/
/*
 * Forward direction
 * Both PWMs: unchanged
 * Direction: forward (phase = 0)
 */
void MotorForward(void){
  motorDirection = "forward";
  digitalWrite(LEFT_WHEEL_PHASE,LOW);
  digitalWrite(RIGHT_WHEEL_PHASE,LOW);  
}
/*----------------------------------------------------------------------------------------------------------*/
/*
 * Reverse direction
 * Both PWMs: unchanged
 * Direction: reverse (phase = 1)
 */
void MotorReverse(void){
  motorDirection = "reverse";
  digitalWrite(LEFT_WHEEL_PHASE,HIGH);
  digitalWrite(RIGHT_WHEEL_PHASE,HIGH); 
}
/*----------------------------------------------------------------------------------------------------------*/
/*
 * Change left motor's spped (PWM)
 * Left PWM: Speed
 * Right PWM: unchanged
 * Direction: unchanged
 */
void MotorLeftSpeed(int Speed) {
  analogWrite(LEFT_WHEEL_ENABLE,Speed);
}
/*----------------------------------------------------------------------------------------------------------*/
/*
 * Change right motor's spped (PWM)
 * Right PWM: Speed
 * Left PWM: unchanged
 * Direction: unchanged
 */
void MotorRightSpeed(int Speed) {
  analogWrite(RIGHT_WHEEL_ENABLE,Speed);
}
/*----------------------------------------------------------------------------------------------------------*/
/*
 *  -----------------------+--------------------------
 *    THROTTLE PERCENTAGE  |   TRANSLATION            
 *  -----------------------+--------------------------
 *           -98           |  SPEED_MAX REVERSE       
 *            0            |      STOP                
 *           100           |  SPEED_MAX FORWARD       
 *  -----------------------+--------------------------
 *                                                    
 *  -----------------------+--------------------------
 *    STEERING PERCENTAGE  |   TRANSLATION            
 *  -----------------------+--------------------------
 *           -98           |    TURN LEFT (MAX ANGLE) 
 *            0            |    STRAIGHT              
 *           100           |    TURN RIGHT (MAX ANGLE)
 *   ----------------------+--------------------------
 */

void MotorControl(int throttlePercentage, int steeringPercentage){
   // Stop the motor
  if (throttlePercentage == 0) {
    MotorStop();
    return;
  }
  // Change direction based on throttle { -98 <--REVERSE--> 0 <--FORWARD--> 100 }
  if (throttlePercentage > 0) {
    MotorForward();
  }
  else {
    MotorReverse();
  }
  /* Convert throttlePecentage (-98-100) (with direction) to speed (0-100) (no direction) and to PWM value (0-255)
   *  
   * Throttle(-98-100)| Speed (0-100)|    PWM (0-255)  | 
   * -----------------+--------------+-----------------+---------------------
   *      -98         |      98      |      250        |  Full speed REVERSE
   *       0          |      0       |       0         |  STOP
   *      100         |      100     |      255        |  Full speed FORWARD
   */
  int throttlePWM = abs(throttlePercentage) * SPEED_MAX / 100;

  
  /* Convert steeringPercentage (-98-100) (with direction) to PWM value (0-255)
   *  
   * Steering(-98-100)|  Left PWM (0-255)  |  Right PWM (0-255)  |
   * -----------------+--------------------+---------------------+----------------------
   *      -98         |    throttlePWM     |         0           |  Full steering LEFT
   *       0          |    throttlePWM     |    throttlePWM      |     go STRAIGHT
   *      100         |       0            |    throttlePWM      |  Full steering RIGHT
   */
  int steeringPWM = (100-abs(steeringPercentage)) * throttlePWM / 100;
  // Go straight
  if (steeringPercentage == 0) {
    motorSteering = "straight";
    MotorLeftSpeed(throttlePWM);
    MotorRightSpeed(throttlePWM);
  }
  // turn right: left motor->throttlePWM, right motor->steeringPWM
  else if (steeringPercentage > 0) {
    motorSteering = "right";
    MotorLeftSpeed(throttlePWM);
    MotorRightSpeed(steeringPWM);
  }
  // turn left: left motor->steeringPWM, right motor->throttlePWM
  else {
    motorSteering = "left";
    MotorLeftSpeed(steeringPWM);
    MotorRightSpeed(throttlePWM);
  } 
}

/* 
 * ==================================================================================================================
 *                                        LIGHT CONTROL TASK
 * ==================================================================================================================
*/
/*
 * Change headlights and tail lights corresponding to the current state of the robot
 * Robot turn --> blink in the turn direction --> LightBlink function
 * Robot direction___stop___headlights off, tail lights on
 *               |___left___left headlight, tail light blink 
 *               |___right__right headlight, tail light blink
 *               
 *   DIRECTION  |  STEERING  |  HEADLIGHT LEFT  |  HEADLIGHT RIGHT  |  TAIL LIGHT LEFT  |  TAIL LIGHT RIGHT  
 * =============|============|==================|===================|===================|====================
 *     STOP     |     ANY    |        ON        |        ON         |       BLINK       |         BLINK      
 * -------------+------------+------------------+-------------------+-------------------+--------------------
 *    FORWARD   |  STRAIGHT  |        ON        |        ON         |        OFF        |          OFF       
 *    FORWARD   |    LEFT    |       BLINK      |        ON         |       BLINK       |          OFF       
 *    FORWARD   |    RIGHT   |        ON        |       BLINK       |        OFF        |         BLINK      
 * -------------+------------+------------------+-------------------+-------------------+--------------------
 *    REVERSE   |  STRAIGHT  |        ON        |        ON         |       BLINK       |         BLINK      
 *    REVERSE   |    LEFT    |        ON        |        ON         |       BLINK       |           ON       
 *    REVERSE   |    RIGHT   |        ON        |        ON         |         ON        |         BLINK      
 * 
 * 
 * 
 * Priority: 1
 * Time dalay: 200ms
 */
void TaskLightControl(void *Parameter) {
  // Initialization OUTPUT pins
  pinMode(HEADLIGHT_LEFT, OUTPUT);
  pinMode(HEADLIGHT_RIGHT, OUTPUT);
  pinMode(TAIL_LIGHT_LEFT, OUTPUT);
  pinMode(TAIL_LIGHT_RIGHT, OUTPUT);

  while (1) {
    /* Forward direction
     * Turn off tail lights
     * __straight__
     * turn on headlights
     * Turn off tail lights
     * __ turn left__
     * blink left headlight
     * blink left tail light
     * __turn right__
     * blink right headlight
     * blink right tail lgiht
     */
    if (motorDirection == "forward") {
      if (motorSteering == "straight") {
        LightOn(HEADLIGHT_LEFT);
        LightOn(HEADLIGHT_RIGHT);
        LightOff(TAIL_LIGHT_LEFT);
        LightOff(TAIL_LIGHT_RIGHT);     
      }
      else if (motorSteering == "left") {
        LightOn(HEADLIGHT_RIGHT);
        LightOff(TAIL_LIGHT_RIGHT);
        LightBlink(HEADLIGHT_LEFT);
        LightBlink(TAIL_LIGHT_LEFT);
      }
      else {
        LightOn(HEADLIGHT_LEFT);
        LightOff(TAIL_LIGHT_LEFT);
        LightBlink(HEADLIGHT_RIGHT);
        LightBlink(TAIL_LIGHT_RIGHT);
      }
    }
    /* Reverse Direction
     *  __straight__
     *  turn on headlight
     *  blink tail lights
     *  __turn left__
     *  turn on right tail light
     *  blink left tail light
     *  __turn right__
     *  turn on left tail light
     *  blink right tail light
     */
    else if (motorDirection == "reverse") {
      LightOn(HEADLIGHT_LEFT);
      LightOn(HEADLIGHT_RIGHT);
      if (motorSteering == "left") {
        LightBlink(TAIL_LIGHT_LEFT);    
        LightOn(TAIL_LIGHT_RIGHT);  
      }
      else if (motorSteering == "right"){
        LightBlink(TAIL_LIGHT_RIGHT);
        LightOn(TAIL_LIGHT_LEFT);
      }
      else {
        LightBlink(TAIL_LIGHT_RIGHT);
        LightBlink(TAIL_LIGHT_LEFT);
      }
    }
    /* Stop (not moving, throttle and steering is 0
     * Turn off headlights
     * Turn on tail lights
     */
    else {
      LightOn(HEADLIGHT_LEFT);
      LightOn(HEADLIGHT_RIGHT);
      LightBlink(TAIL_LIGHT_LEFT);
      LightBlink(TAIL_LIGHT_RIGHT);      
    }
    
    //Delay task 200ms
    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}
/* 
 * ==================================================================================================================
 *                                        LIGHT CONTROL FUNCTIONS
 * ==================================================================================================================
*/

// Toggling (blinking) output pin "light"
void LightBlink(uint8_t light) {
  digitalWrite(light,!digitalRead(light));
}

/*----------------------------------------------------------------------------------------------------------*/
// Turning light on
void LightOn(uint8_t light) {
  digitalWrite(light, HIGH);
}
/*----------------------------------------------------------------------------------------------------------*/
// Turning light off
void LightOff(uint8_t light) {
  digitalWrite(light, LOW);
}
