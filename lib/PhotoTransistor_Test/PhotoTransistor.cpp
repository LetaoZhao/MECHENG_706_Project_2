#include <Arduino.h>
#include <GlobalVariable.hpp>
#include <PID_v1.h>
#include <MotorMovement/MotorMovement.hpp>

void PhotoTransistor_Initialize() {
  // put your setup code here, to run once:

  pinMode(A13,INPUT); //right sensor sr
  pinMode(A12,INPUT); //left sensor sr
  pinMode(A15, INPUT); //right sensor lr
  pinMode(A14, INPUT); //left sensor lr

  for(int i = 0; i < 10; i++)
  {
    voltage_left[i] = 0;
    voltage_right[i] = 0;
  }

  return;
}


void PhotoTransistor_Read() {
  // Test is intended to be run repeatedly

  //voltage is 10 pt array
  for (int i = 9; i >= 2; i--)
  {
    voltage_left[i] = voltage_left[i-1];
    voltage_right[i] = voltage_right[i-1];
  }

  voltage_right[0] = analogRead(A15) * 0.0049; //5V 10Bit ADC 
  voltage_left[0] = analogRead(A14) * 0.0049; //5V 10Bit ADC 

  right_avg = 0;
  left_avg = 0;
  for(int i = 0; i < 3; i++)
  {
    right_avg += voltage_right[i];
    left_avg += voltage_left[i];
  }

  right_avg = right_avg/3;
  left_avg = left_avg/3;
}


void TurnToFire()
{
  int found_lightsource = 0;
  while(found_lightsource != 1)
  {
    if(right_avg-left_avg > 0.1)
  {
    // Serial.println("right bigger than left");
    // cw();
  }
  else if (left_avg-right_avg > 0.1)
  {
    // Serial.println("left bigger than right");
    // ccw();
  }
  else if (left_avg < 0.1 & right_avg < 0.1)
  {
    // Serial.println("zero");
    // cw();
  } 
  else if (right_avg - left_avg < 0.1 && right_avg > 0.1) //to stop erroneous stopping when looking at zeros
  {
    found_lightsource = 1;
    // Serial.println("found it");
    // stop();
  }
  PhotoTransistor_Read();
  delay(1);
  }
}
