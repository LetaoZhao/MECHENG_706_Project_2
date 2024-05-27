#include <Arduino.h>
#include "GlobalVariable.hpp"
#include <PID_v1.h>
#include <MotorMovement/MotorMovement.hpp>
#include <MoveStraightPID/MoveStraightPID.hpp>
#include <Sonar.hpp>
#include <SerialComs.hpp>
#include <IR_Read.hpp>
#include <PhotoTransistor.hpp>


// bool MoveToFireUntil_WithAvoidence()
// {
//     bool isRunning = 1;
//     bool startAvoidence = 0;

//     float left_distance_IR = 0.0; //voltage (V)
//     float right_distance_IR = 0.0; //voltage (V)

//     while(isRunning) //main loop
//     {
//         //check if there is an object in front of the robot
//         left_distance_IR = analogRead(AXXXXXX) * 0.0049; //5V 10Bit ADC 
//         right_distance_IR = analogRead(AXXXXXX) * 0.0049; //5V 10Bit ADC 
//         if((left_distance_IR < XXXXXXX)||(left_distance_IR < XXXXXXX)||(HC_SR04_range() < 10))
//         {
//             //check if this is fire
//             PhotoTransistor_Read();
//             if((lr_right_avg < XXXXXXX)||(lr_left_avg < XXXXXXX))
//             {
//                 return true; //if yes, function ended
//             }
//             else 
//             {
//                 //start avoidence
//                 startAvoidance = true; 
//                 while(startAvoidence)
//                 {
//                     int speed_vel = Calculate_Turning_Potential();
//                     left_font_motor.writeMicroseconds(1500 + speed_vel);
//                     left_rear_motor.writeMicroseconds(1500 + speed_vel);
//                     right_rear_motor.writeMicroseconds(1500 +speed_vel);
//                     right_font_motor.writeMicroseconds(1500 + speed_vel);
//                     //cw+
//                     //ccw- 
//                 }
//             }
//         }

//     }

    
//     // if()

// }


int Calculate_Turning_Potential()
{
    //return speed value to turn
    double Dis_4102 = IR_sensorReadDistance("41_02");
    double Dis_4103 = IR_sensorReadDistance("41_03");
    double Dis_Sonar = HC_SR04_range();
    if(Dis_4102 > 400) {Dis_4102 = 400;}
    if(Dis_4103 > 400) {Dis_4103 = 400;}
    if(Dis_Sonar > 15) {Dis_Sonar = 15;}

    double left_potential = (15 - Dis_Sonar)*5 + (200 - Dis_4102);
    double right_potential = (15 - Dis_Sonar)*5 + (200 - Dis_4103);

    return (int)(500*(left_potential - right_potential)/275);
}