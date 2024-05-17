#include <Arduino.h>
#include <Servo.h> //Need for Servo pulse output
#include <GlobalVariable.hpp>

void TurnTurretTo(double angle)
{
    //ccw as positive degrees

    //saturation
    if (angle >= 60) {angle = 60;}
    else if (angle <= -60) {angle = -60;}
    else {}

    //find position value
    int value = 1500 + (int)(angle*10);

    //action
    turret_motor.writeMicroseconds(value);
    delay(500);
}