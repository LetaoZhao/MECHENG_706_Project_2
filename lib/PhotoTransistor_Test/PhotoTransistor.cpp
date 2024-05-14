#include <Arduino.h>
#include <GlobalVariable.hpp>
#include <PID_v1.h>

void PhotoTransistor_Initialize() {
  // put your setup code here, to run once:

  pinMode(A13,INPUT); //right sensor sr
  pinMode(A12,INPUT); //left sensor sr
  pinMode(A15, INPUT); //right sensor lr
  pinMode(A14, INPUT); //left sensor lr

  for(int i = 0; i < 3; i++)
  {
    voltage_left[i] = 0;
    voltage_right[i] = 0;
  }

  return;
}


void PhotoTransistor_Test() {
  // Test is intended to be run repeatedly

  voltage_left[2] =   voltage_left[1];
  voltage_right[2] = voltage_right[1];

  voltage_right[1] = voltage_right[0]; //5V 10Bit ADC 
  voltage_left[1] =   voltage_left[0];

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

