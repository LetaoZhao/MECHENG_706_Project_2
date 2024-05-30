#include <Arduino.h>
#include <Servo.h> //Need for Servo pulse output
#include <GlobalVariable.hpp>
#include <PhotoTransistor.hpp>
#include <MotorMovement/MotorMovement.hpp>

void TurnTurretTo(double angle)
{
    //ccw as positive degrees

    //saturation
    if (angle >= 55) {angle = 55;}
    else if (angle <= -55) {angle = -55;}
    else {}

    //find position value
    int value = 1500 + (int)(angle*10);

    //action
    turret_motor.writeMicroseconds(value);
    delay(500);
}

void TurretToFire()
{
    bool isFind = false;
    double angle = 0;
    TurnTurretTo(angle);

    while(isFind == 0)
    {
        PhotoTransistor_Read();
        if(lr_right_avg - lr_left_avg > 0.1)
        {
            angle = angle - 1;
        }
        else if(lr_left_avg - lr_right_avg > 0.1)
        {
            angle = angle + 1;
        }
        else
        {
            isFind = 1;
        }

        if((angle > 55)||(angle < -55))
        {
            isFind = 1;
        }

        TurnTurretTo(angle);
        delay(50);
    }
}

void Execute_Fire()
{
    start_fan();
    PhotoTransistor_Read();
    int execute_time_count = 0;
    while(lr_mid_avg > 0.3) 
    {
      //while not executed, keep doing that
      execute_time_count++;
      delay(50);
      PhotoTransistor_Read();

      //if execute greater than 10 sec, break
      if(execute_time_count > 200) 
      {
        lr_mid_avg = 0;
      }
    } 
    stop_fan();
}