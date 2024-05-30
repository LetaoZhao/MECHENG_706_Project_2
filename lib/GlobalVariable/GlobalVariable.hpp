#include <stdint.h>
#include <Servo.h>

#define left_front 46
#define left_rear  47
#define right_rear  50
#define right_front  51
#define turret 14

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
extern double fire_heading;

// Movement phases and states
extern int movement_phase;
// enum movement_phase {TurnToFire, FireHoming, TurnAwayFromObject, DriveFree, TurnBack, CheckFire,Extinguish};
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
extern float IR_values_right[30];
extern float IR_right_avg;
extern float IR_values_left[30];
extern float IR_left_avg;
extern float IR_sum_right;
extern float IR_sum_left;

extern float IR_values_right_45[30];
extern float IR_right_45_avg;
extern float IR_values_left_45[30];
extern float IR_left_45_avg;
extern float IR_sum_right_45;
extern float IR_sum_left_45;

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

extern double lr_voltage_right[10];
extern double lr_voltage_left[10];
extern double lr_voltage_mid[10];
extern double lr_voltage_top[10];
extern double lr_right_avg;
extern double lr_left_avg;
extern double lr_mid_avg;
extern double lr_top_avg;

// manual gyro
extern int manual_gyro_count;
extern int manual_gyro_offset;

extern int fires_extuiguished;
