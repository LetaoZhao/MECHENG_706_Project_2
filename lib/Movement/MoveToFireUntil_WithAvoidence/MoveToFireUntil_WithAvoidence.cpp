#include <Arduino.h>
#include "GlobalVariable.hpp"
#include <PID_v1.h>
#include <MotorMovement/MotorMovement.hpp>
#include <MoveStraightPID/MoveStraightPID.hpp>
#include <Sonar.hpp>
#include <SerialComs.hpp>
#include <IR_Read.hpp>
#include <PhotoTransistor.hpp>


void MoveToFireUntil_WithAvoidence(float TargetDistance, double Power)
{
    //
    bool isRunning = 1;
    enum Moving_Machine_States{
        Forward,
        Avoiding
    };
    Moving_Machine_States Current_State = Forward;

    float SonarDistance = HC_SR04_range(); //cm

    int forward_speed_value = 0;
    



    while(isRunning)
    {
        switch(Current_State)
        {
            case Forward:
            {
                //forward
                PhotoTransistor_Read(); //refresh IR readings

                //PID needed
                forward_speed_value = (int)0;  ///

                left_font_motor.writeMicroseconds(1500 + forward_speed_value);    //-
                left_rear_motor.writeMicroseconds(1500 + forward_speed_value);    //-
                right_rear_motor.writeMicroseconds(1500 + forward_speed_value);   //+
                right_font_motor.writeMicroseconds(1500 + forward_speed_value);   //+

                //detect conditions
                SonarDistance = HC_SR04_range();
                if(SonarDistance < TargetDistance) //if there is an object
                {
                    if(1)//IR say it's not fire)
                    {
                        stop(); //stop moving
                        Current_State = Avoiding; //avoidance
                    }
                    else if(2)//IR say this is a fire)
                    {
                        stop(); //stop moving
                        isRunning = 0; //function finished
                    }
                }
                break;
            }
            case Avoiding:
            {
                //
                break;
            }
        }
    }
    









    //initial
    int initial_speed_valure = 5*Power;
    left_font_motor.writeMicroseconds(1500 - initial_speed_valure);
    left_rear_motor.writeMicroseconds(1500 - initial_speed_valure);
    right_rear_motor.writeMicroseconds(1500 + initial_speed_valure);
    right_font_motor.writeMicroseconds(1500 + initial_speed_valure);

    


    //

    // //main loop
    // while(SonarDistance > TargetDistance)||(//IR say not reach fire)
    // {
    //     PhotoTransistor_Read();
    // }
}