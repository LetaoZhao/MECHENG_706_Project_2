#include <stdint.h>
#include <Servo.h>

// Refer to Shield Pinouts.jpg for pin locations

// Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Default IR sensor pins, these pins are defined by the Shield
#define IR_41_01 12
#define IR_41_02 A8
#define IR_41_03 A10
// #define IR_2Y_01 14
#define IR_2Y_02 A11
// uncomment if these IR sensors are used
//  #define IR_2Y_03 14
#define IR_2Y_04 A9

const unsigned int MAX_DIST = 23200;

// declare global var
// extern uint16_t currentAngle;
// extern uint16_t sensorValue;
Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor; // create servo object to control Vex Motor Controller 29
Servo right_font_motor; // create servo object to control Vex Motor Controller 29
Servo turret_motor;

// Speed and motor movement
int speed_val = 500;
int speed_change;
int speed_val_low = 150;

// PID
int KP = 100;
float ErrorAngle_Degree = 0.0;
float offset_angle = -0.1;

int SVRF = 0;
int SVRR = 0;
int SVLF = 0;
int SVLR = 0;

int Kp = 1000;
int SV_P = 0;

float TargetAngle_Radius = 0.0;
float ErrorAngle_Radius = 0.0; // TargetAngle - CurrentAngle
float torlance = 10.0;         // degree

// gyro
int sensorPin = A7;            // define the pin that gyro is connected
int sensorValue = 0;           // read out value of sensor
float gyroSupplyVoltage = 5;   // supply voltage for gyro
float gyroZeroVoltage = 0;     // the value of voltage when gyro is zero
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less than
// this value will not be ignored
float gyroRate = 0;      // read out value of sensor in voltage
double currentAngle = 0; // current angle calculated by angular velocity integral on
float GyroTimeNow = 0;
float GyroTimePrevious = 0;

int movement_phase = 0; // use for flow control of the robots programmed movement
int currentState = 0;

// IR Readings
float temp_4102 = 0.0;
float temp_4103 = 0.0;
float temp_2Y02 = 0.0;
float temp_2Y04 = 0.0;

float VALUE_4102 = 0.0;
float VALUE_4103 = 0.0;

float VALUE_2Y04 = 0.0;
float VALUE_2Y02 = 0.0;

float PREVIOUS_2Y04_VALUE = 0;
float PREVIOUS_2Y02_VALUE = 0;

// float Kp_IR_dif = 10;
// float Kp_IR_abs = 15;
float threathod_IR = 10000;

// Sonar Readings
float sonar_reading = 0;
float sonar_reading_prev1 = 0;
float sonar_reading_prev2 = 0;
float sonar_reading_prev3 = 0;
float sonar_reading_prev4 = 0;
float sonar_reading_prev5 = 0;

float sonar_average = 0;
float sonar_average_prev1 = 0;

// move along wall variables
float Kp_IR_abs = 0;
float Ki_IR_abs = 0;
float Kp_IR_dif = 0;
float Ki_IR_dif = 0;

float Kp_GV_dif = 10;
float Ki_GV_dif = 1;