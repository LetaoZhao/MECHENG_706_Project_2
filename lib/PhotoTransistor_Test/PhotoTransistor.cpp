#include <Arduino.h>
#include <GlobalVariable.hpp>
#include <PID_v1.h>
#include <MotorMovement/MotorMovement.hpp>
#include <Sonar.hpp>
#include <SimpleAvoidence/SimpleAvoidence.hpp>
#include <IR_Read.hpp>

void PhotoTransistor_Initialize() {
  // put your setup code here, to run once:

  pinMode(A13,INPUT); //right sensor sr
  pinMode(A12,INPUT); //left sensor sr
  pinMode(A15, INPUT); //right sensor lr
  pinMode(A14, INPUT); //left sensor lr

  for(int i = 0; i < 10; i++)
  {
    lr_voltage_left[i] = 0;
    lr_voltage_right[i] = 0;
    lr_voltage_mid[i] = 0;
  }

  return;
}


void PhotoTransistor_Read() {
  //run repeatedly
  //voltage is 10 pt array
  for (int i = 9; i >= 1; i--)
  {
    lr_voltage_left[i] = lr_voltage_left[i-1];
    lr_voltage_right[i] = lr_voltage_right[i-1];
    lr_voltage_mid[i] = lr_voltage_mid[i-1];
  }
 
  lr_voltage_right[0] = analogRead(A15) * 0.0049; //5V 10Bit ADC
  lr_voltage_left[0] = analogRead(A14) * 0.0049; //5V 10Bit ADC
  lr_voltage_mid[0] = analogRead(A12) * 0.0049;

  lr_right_avg = 0;
  lr_left_avg = 0;
  lr_mid_avg = 0;
  for(int i = 0; i < 10; i++)
  {
    lr_right_avg += lr_voltage_right[i];
    lr_left_avg += lr_voltage_left[i];
    lr_mid_avg += lr_voltage_mid[i];
  }

  lr_right_avg = lr_right_avg/10;
  lr_left_avg = lr_left_avg/10;
  lr_mid_avg = lr_mid_avg/10;


  // Serial1.print(">Right LR: ");
  // // Serial1.println(lr_voltage_right[0]);
  // Serial1.println(lr_left_avg);
  // Serial1.print(">Left LR: ");
  // // Serial1.println(lr_voltage_left[0]);
  // Serial1.println(lr_right_avg);
  // Serial1.print(">Difference: "),
  // Serial1.println(lr_right_avg-lr_left_avg);
  // delay(10);
}


bool TurnToFire()
{
  PhotoTransistor_Read();
  static float error = 0;
  static float error_kp;
  static float kp = 1;
  error_kp = error*kp;
  if (error_kp < 0) {error_kp = error_kp * -1;} //make sure error is not negative
  static float motor_speed = speed_val_low + error_kp; //add to a speed value to speed up when far away
  if (motor_speed > 500) {motor_speed = 500;} //saturation


  if (lr_right_avg - lr_left_avg < 0.65 && (lr_right_avg > 1 || lr_left_avg > 1)) //to stop erroneous stopping when looking at zeros
  {
    stop();
    return true;
    Serial1.println("found it");
    // stop();
  }
  else if (lr_right_avg-lr_left_avg > 0.6)
  {
    // Serial.println("right bigger than left");
    //go cw
    left_font_motor.writeMicroseconds(1500 + motor_speed);
    right_font_motor.writeMicroseconds(1500 + motor_speed);
    left_rear_motor.writeMicroseconds(1500 + motor_speed);
    right_rear_motor.writeMicroseconds(1500 + motor_speed);

  }
  else if (lr_left_avg-lr_right_avg > 0.6)
  {
    // Serial.println("left bigger than right");
    //go ccw
    left_font_motor.writeMicroseconds(1500 - motor_speed);
    right_font_motor.writeMicroseconds(1500 - motor_speed);
    left_rear_motor.writeMicroseconds(1500 - motor_speed);
    right_rear_motor.writeMicroseconds(1500 -  motor_speed);
  }
  else if ((lr_left_avg < 0.6) & (lr_right_avg < 0.6))
  {
    // Serial.println("zero");
    cw();
    // cw();
  }

  Serial1.print(">Right LR: ");
  Serial1.println(lr_right_avg);
  delay(1);
  Serial1.print(">Left LR: ");
  Serial1.println(lr_left_avg);
  delay(1);
  Serial1.print(">Difference: "),
  Serial1.println(lr_right_avg-lr_left_avg);
  delay(1);
  Serial1.print(">Error: ");
  Serial1.println(error_kp);
  delay(1);

  return false;
}

bool FireHoming()
{
  static float error = 0;
  static float error_kp;
  static float kp = 1000;
  bool found_fire = false;
  if(HC_SR04_range() < 10)
  {
    stop();
    found_fire = true;
  } else
  {
    PhotoTransistor_Read();
    //thresholds for transitioning
    // if (lr_right_avg > threshold || lr_left_avg > threshold)
    // {
    //   right - lr_right;
    //   left = lr_left;
    // }
    // else
    // //use the shortrange sensors
    // {

    // }

    //calculate errors
    error = lr_right_avg - lr_left_avg;
    error_kp = error * kp;
    //left front, left rear, right rear, right front
    float motor_speeds[4] = {-speed_val+error_kp,-speed_val+error_kp,speed_val+error_kp,speed_val+error_kp};
    compute_speed(motor_speeds);
    left_font_motor.writeMicroseconds(1500 + motor_speeds[0]);
    left_rear_motor.writeMicroseconds(1500 + motor_speeds[1]);
    right_rear_motor.writeMicroseconds(1500 + motor_speeds[2]);
    right_font_motor.writeMicroseconds(1500 + motor_speeds[3]);
    Serial1.print(">Error: ");
    Serial1.println(error_kp);

  }

  return found_fire;
}


bool FireHoming_Avoidence()
{
  static float error = 0;
  static float error_kp;
  static float kp = 1000;
  bool found_fire = false;
  double left_IR_dis = (IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02"))/3;
  double right_IR_dis = (IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03"))/3;
  double sonar_dis = (HC_SR04_range()+HC_SR04_range()+HC_SR04_range())/3;
  if((sonar_dis < 15) || (left_IR_dis < 200) || (right_IR_dis < 130))
  {
    
    PhotoTransistor_Read();
    if(lr_mid_avg > 4)
    {
      stop();
      found_fire = true;
    }
    else
    {
      // stop();
      // delay(2000);
      // if(left_IR_dis < 200)
      // {
      //   ccw();
      //   delay(500);
      // }
      // else if(right_IR_dis < 130)
      // {
      //   cw();
      //   delay(500);
      // }
      // else{}
      // stop();
      // delay(2000);
      
      ObjectAvoidence();
      // stop();
      // delay(2000);
    }
        
  }
  else
  {
    PhotoTransistor_Read();
    //thresholds for transitioning
    // if (lr_right_avg > threshold  || lr_left_avg > threshold)
    // {
    //   right - lr_right;
    //   left = lr_left;
    // }
    // else
    // //use the shortrange sensors
    // {

    // }

    //calculate errors
    error = lr_right_avg - lr_left_avg;
    error_kp = error * kp;
    //left front, left rear, right rear, right front
    float motor_speeds[4] = {-speed_val+error_kp,-speed_val+error_kp,speed_val+error_kp,speed_val+error_kp};
    compute_speed(motor_speeds);
    left_font_motor.writeMicroseconds(1500 + motor_speeds[0]);
    left_rear_motor.writeMicroseconds(1500 + motor_speeds[1]);
    right_rear_motor.writeMicroseconds(1500 + motor_speeds[2]);
    right_font_motor.writeMicroseconds(1500 + motor_speeds[3]);
    Serial1.print(">Error: ");
    Serial1.println(error_kp);


  }

  Serial.println(lr_mid_avg);
  // Serial.println(lr_left_avg);
  // Serial.println(lr_right_avg);

  return found_fire;
}

