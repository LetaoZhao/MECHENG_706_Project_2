#include <Arduino.h>
#include "GlobalVariable.hpp"
#include <PID_v1.h>
#include <MoveStraightPID/MoveStraightPID.hpp>
#include <SerialComs.hpp>
#include <MotorMovement/MotorMovement.hpp>

void resetGyro()
{
  currentAngle = 0;
  long sum = 0;
  for (int i = 0; i < 100; i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift
  {
    sensorValue = analogRead(sensorPin);
    // Serial1.print("delta sum: ");
    // Serial1.println(sensorValue);
    sum += sensorValue;
    // Serial1.print("sum: ");
    // Serial1.println(sum);
    delay(5);
  }
  gyroZeroVoltage = int(sum / 100); // average the sum as the zero drifting
  // Serial1.println(gyroZeroVoltage);
}

void readGyro1()
{
  // Serial1.println(analogRead(sensorPin));
  GyroTimeNow = (float)millis() / 1000;
  gyroRate = (analogRead(sensorPin) * gyroSupplyVoltage) / 1023; // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * 5);                      // 5 is the number of ms delay the gryozerovoltage is taken over
  float angularVelocity = gyroRate / gyroSensitivity;            // read out voltage divided the gyro sensitivity to calculate the angular velocity
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = -angularVelocity * (GyroTimeNow - GyroTimePrevious); // calculate the angle change based on the angular velocity
    currentAngle += angleChange;                                             // 0.003 is a compensation factor obtained through tuning
  }
  GyroTimePrevious = GyroTimeNow;
}

void readGyroTurn()
{
  // Serial1.println(analogRead(sensorPin));
  GyroTimeNow = (float)millis() / 1000;
  gyroRate = (analogRead(sensorPin) * gyroSupplyVoltage) / 1023; // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * 5);                      // 5 is the number of ms delay the gryozerovoltage is taken over
  float angularVelocity = gyroRate / gyroSensitivity;            // read out voltage divided the gyro sensitivity to calculate the angular velocity
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = -angularVelocity * (GyroTimeNow - GyroTimePrevious);
    if (angleChange != 0)
    {
      currentAngle += angleChange + 0.029 * (abs(angleChange) / angleChange); // 0.003 is a compensation factor obtained through tuning
    }                                                                         // calculate the angle change based on the angular velocity
  }
  GyroTimePrevious = GyroTimeNow;
}

// Define Variables we'll be connecting to
double Setpoint, Input, Output;
// output is a value between 0 and 255
// need to map to 0 being -500 and 255 being 500ca
// Specify the links and initial tuning parameters
double Kp_Turn = 8, Ki_Turn = 0, Kd_Turn = 0;
// TO GO CW use DIRECT and CW
// To GO CCW use REVERSE and CCW
PID myPID(&currentAngle, &Output, &Setpoint, Kp_Turn, Ki_Turn, Kd_Turn, DIRECT);

// void GyroTurn(double target)
// {
//   static unsigned long previous_millis;
//   static char msg;
//   static double speed_value;
//   static unsigned long start_turn;
//   static double tolerance = 5;
//   static double error = 0;
//   static double cumm_error = 0;

//   while (currentAngle < (target - tolerance) || currentAngle > (target + tolerance))
//   {
//     if (millis() - previous_millis > 50)
//     {
//       readGyro1();
//       error = target - currentAngle;
//       cumm_error += error;
//       speed_value = error * Kp_Turn + cumm_error * Ki_Turn;

//       left_font_motor.writeMicroseconds(1500 - saturation(speed_value));
//       left_rear_motor.writeMicroseconds(1500 - saturation(speed_value));
//       right_rear_motor.writeMicroseconds(1500 - saturation(speed_value));
//       right_font_motor.writeMicroseconds(1500 - saturation(speed_value));

//       // Serial1.println(currentAngle);
//       previous_millis = millis();
//       ReadAllSensor();
//       // Serial1.println(saturation(100+speed_value));
//     }
//   }
//   // Serial1.println("Stopping");
//   stop();
//   return;
// }