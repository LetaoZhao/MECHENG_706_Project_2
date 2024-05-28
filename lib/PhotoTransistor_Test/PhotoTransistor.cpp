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
  for (int i = 9; i >= 2; i--)
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
}


bool TurnToFire()
{
  PhotoTransistor_Read();
  if(lr_right_avg-lr_left_avg > 0.3)
  {
    // Serial.println("right bigger than left");
    cw_low();
  }
  else if (lr_left_avg-lr_right_avg > 0.3)
  {
    // Serial.println("left bigger than right");
    ccw_low();
  }
  else if ((lr_left_avg < 0.1) & (lr_right_avg < 0.1))
  {
    // Serial.println("zero");
    cw_low();
  }
  else if (lr_right_avg - lr_left_avg < 0.1 && lr_right_avg > 0.5) //to stop erroneous stopping when looking at zeros
  {
    stop();
    return true;
    // Serial.println("found it");
    // stop();
  }

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
  if((HC_SR04_range() < 10) || IR_sensorReadDistance("41_02") < 200 || IR_sensorReadDistance("41_03") < 100)
  {
    
    PhotoTransistor_Read();
    if(lr_mid_avg > 4)
    {
      stop();
      found_fire = true;
    }
    else
    {
      ObjectAvoidence();
    }
        
  }
  else
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

  Serial.println(lr_mid_avg);
  // Serial.println(lr_left_avg);
  // Serial.println(lr_right_avg);

  return found_fire;
}

