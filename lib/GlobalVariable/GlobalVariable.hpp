#include <stdint.h>
#include <Servo.h>

#define left_front 46
#define left_rear  47
#define right_rear  50
#define right_front  51
#define turret 8

#define TRIG_PIN  48
#define ECHO_PIN 49
#define MAX_DIST 23200


// IR sensor pins
#define IR_41_01 12
#define IR_41_02 A8
#define IR_41_03 A10
// #define IR_2Y_01 14
#define IR_2Y_02 A11
// #define IR_2Y_03 14
#define IR_2Y_04 A9

// Servo motors
extern Servo left_font_motor;
extern Servo left_rear_motor;
extern Servo right_rear_motor;
extern Servo right_font_motor;
extern Servo turret_motor;

// Movement and PID parameters
extern int speed_val;
extern int speed_change;
extern int speed_val_low;
extern int KP;
extern float ErrorAngle_Degree;
extern float offset_angle;
extern int SVRF;
extern int SVRR;
extern int SVLF;
extern int SVLR;
extern int Kp;
extern int SV_P;
extern float TargetAngle_Radius;
extern float ErrorAngle_Radius;
extern float torlance;

// Gyroscopic sensor parameters
extern int sensorPin;
extern int sensorValue;
extern float gyroSupplyVoltage;
extern float gyroZeroVoltage;
extern float gyroSensitivity;
extern float rotationThreshold;
extern float gyroRate;
extern double currentAngle;
extern float GyroTimeNow;
extern float GyroTimePrevious;

// Movement phases and states
extern int movement_phase;
extern int currentState;

// IR sensor readings
extern float temp_4102;
extern float temp_4103;
extern float temp_2Y02;
extern float temp_2Y04;
extern float VALUE_4102;
extern float VALUE_4103;
extern float VALUE_2Y04;
extern float VALUE_2Y02;
extern float PREVIOUS_2Y04_VALUE;
extern float PREVIOUS_2Y02_VALUE;
extern float threathod_IR;

// Sonar readings and averages
extern float sonar_reading;
extern float sonar_reading_prev1;
extern float sonar_reading_prev2;
extern float sonar_reading_prev3;
extern float sonar_reading_prev4;
extern float sonar_reading_prev5;
extern float sonar_average;
extern float sonar_average_prev1;

// PID control for wall following
extern float Kp_IR_abs;
extern float Ki_IR_abs;
extern float Kp_IR_dif;
extern float Ki_IR_dif;
extern float Kp_GV_dif;
extern float Ki_GV_dif;

static double voltage_right[3];
static double voltage_left[3];
static double right_avg;
static double left_avg;
