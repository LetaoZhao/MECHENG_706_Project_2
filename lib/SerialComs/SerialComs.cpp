#include <Arduino.h>
#include "GlobalVariable.hpp"
#include <PID_v1.h>
#include <MotorMovement/MotorMovement.hpp>
#include <MoveStraightPID/MoveStraightPID.hpp>
#include <Sonar.hpp>

// This file contains serial functions

// Serial command pasing for serial move
void interpret_command(char command)
{
    // Perform an action depending on the command
    switch (command)
    {
    case 'w': // Move Forward
    case 'W':
        forward();
        break;
    case 's': // Move Backwards
    case 'S':
        reverse();
        break;
    case 'q': // Turn Left
    case 'Q':
        strafe_left(500);
        break;
    case 'e': // Turn Right
    case 'E':
        strafe_right(500);
        break;
    case 'a': // Turn Right
    case 'A':
        ccw();
        break;
    case 'd': // Turn Right
    case 'D':
        cw();
        break;
    case '-': // Turn Right
    case '_':
        speed_change = -100;
        break;
    case '=':
    case '+':
        speed_change = 100;
        break;
    default:
        stop();
        // SerialCom->println("stop");
        break;
    }
}

void ReadAllSensor()
{
    Serial1.print(movement_phase);
    Serial1.print(" ");

    Serial1.print(VALUE_2Y04);
    Serial1.print(" ");

    Serial1.print(VALUE_4102);
    Serial1.print(" ");

    Serial1.print(VALUE_2Y02);
    Serial1.print(" ");

    Serial1.print(VALUE_4103);
    Serial1.print(" ");

    Serial1.print(sonar_reading);
    Serial1.print(" ");

    Serial1.print(currentAngle);
    Serial1.print(" ");

    Serial1.println(" ");
}