#include <Arduino.h>
#include "GlobalVariable.hpp"
#include <PID_v1.h>
#include <MotorMovement/MotorMovement.hpp>
#include <MoveStraightPID/MoveStraightPID.hpp>
#include <Sonar.hpp>
#include <SerialComs.hpp>
#include <IR_Read.hpp>
#include <PhotoTransistor.hpp>


bool MoveToFireUntil_WithAvoidence()
{
    // bool isRunning = 1;
    // bool startAvoidence = 0;

    // float left_distance_IR = 0.0; //voltage (V)
    // float right_distance_IR = 0.0; //voltage (V)

    // while(isRunning) //main loop
    // {
        //check if there is an object in front of the robot
        // left_distance_IR = analogRead(AXXXXXX) * 0.0049; //5V 10Bit ADC 
        // right_distance_IR = analogRead(AXXXXXX) * 0.0049; //5V 10Bit ADC 
        // if((left_distance_IR < XXXXXXX)||(left_distance_IR < XXXXXXX)||(HC_SR04_range() < 10))
        // {
        //     //check if this is fire
        //     PhotoTransistor_Read();
        //     if((lr_right_avg < XXXXXXX)||(lr_left_avg < XXXXXXX))
            // {
            //     return true; //if yes, function ended
            // }
            // else 
            // {
            //     //start avoidence
            //     startAvoidence = true; 
            //     while(startAvoidence)
            //     {
                    // left_font_motor.writeMicroseconds(1500 + motor_speeds[0]);
                    // left_rear_motor.writeMicroseconds(1500 + motor_speeds[1]);
                    // right_rear_motor.writeMicroseconds(1500 + motor_speeds[2]);
//                     // right_font_motor.writeMicroseconds(1500 + motor_speeds[3]);





//                     //cw+
//                     //ccw- 
//                 }
//             }
//         }

//         if(startAvoidence)
//     }

    
//     if()




}