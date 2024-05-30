#include <Arduino.h>
#include "GlobalVariable.hpp"
#include <PID_v1.h>
#include "MotorMovement/MotorMovement.hpp"
#include "MoveStraightPID/MoveStraightPID.hpp"
#include "Sonar.hpp"

// This file contains serial functions

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

void print_sensors()
{
    Serial1.print(">Right IR: ");
    Serial1.println(IR_right_avg);
    Serial1.print(">Left IR: ");
    Serial1.println(IR_left_avg);
    Serial1.print(">Sonar: "),
    Serial1.println(sonar_reading);
    Serial1.print(">Right LR: ");
    Serial1.println(lr_left_avg); 
    Serial1.print(">Left LR: ");
    Serial1.println(lr_right_avg);
    Serial1.print(">GyroAngle: ");
    Serial1.println(currentAngle);
    Serial1.print(">Mid IR: ");
    Serial1.println(lr_mid_avg);
    Serial1.print(">Top IR: ");
    Serial1.println(lr_top_avg);
    Serial1.print(">Right Angle IR: ");
    Serial1.println(IR_right_45_avg);
    Serial1.print(">Left Angle IR: ");
    Serial1.println(IR_left_45_avg);

}