#include <Arduino.h>
#include "GlobalVariable.hpp"
#include <PID_v1.h>
#include <MotorMovement/MotorMovement.hpp>
#include <MoveStraightPID/MoveStraightPID.hpp>
#include <Sonar.hpp>
#include <SerialComs.hpp>
#include <IR_Read.hpp>
#include <PhotoTransistor.hpp>

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

void Fire_Track_Turret()
{
    //
}

bool MoveToFireUntil_WithAvoidence()
{
    bool isRunning = 1;
    bool startAvoidence = 0;

    float left_distance_IR = 0.0; //voltage (V)
    float right_distance_IR = 0.0; //voltage (V)

    static float error = 0;
    static float error_kp;
    static float kp = 1000;

    while(isRunning) //main loop
    {
        //check if there is an object in front of the robot
        left_distance_IR = IR_sensorReadDistance("41_02");
        right_distance_IR = IR_sensorReadDistance("41_03");
        if((left_distance_IR < 150)||(left_distance_IR < 150)||(HC_SR04_range() < 10))
        {
            //check if this is fire
            PhotoTransistor_Read();
            if((lr_right_avg > 4)||(lr_left_avg > 4))
            {
                stop();
                isRunning  = 0;
                return true; //if yes, function ended
            }
            else 
            {
                //start avoidence
                bool startAvoidance = true; 
                while(startAvoidence)
                {
                    //cw+
                    //ccw- 
                    int speed_vel = Calculate_Turning_Potential();
                    left_font_motor.writeMicroseconds(1500 + speed_vel);
                    left_rear_motor.writeMicroseconds(1500 + speed_vel);
                    right_rear_motor.writeMicroseconds(1500 +speed_vel);
                    right_font_motor.writeMicroseconds(1500 + speed_vel);

                    if(speed_vel < 100)
                    {
                        startAvoidance = false;

                        int time_start = millis();
                        int time_gap = millis() - time_start;
                        while (time_gap < 500)
                        {
                            left_font_motor.writeMicroseconds(1500 - 500);
                            left_rear_motor.writeMicroseconds(1500 - 500);
                            right_rear_motor.writeMicroseconds(1500 + 500);
                            right_font_motor.writeMicroseconds(1500 + 500);

                            if((left_distance_IR < 150)||(left_distance_IR < 150)||(HC_SR04_range() < 10))
                            {
                                stop();
                                time_gap += 500;
                            }
                        }
                    }
                }
            }
        }

        //if no object in front, moving to the fire
        PhotoTransistor_Read();
        error = lr_right_avg - lr_left_avg;
        error_kp = error * kp;
        //left front, left rear, right rear, right front
        float motor_speeds[4] = {-speed_val+error_kp,-speed_val+error_kp,speed_val+error_kp,speed_val+error_kp};
        compute_speed(motor_speeds);
        left_font_motor.writeMicroseconds(1500 + motor_speeds[0]);
        left_rear_motor.writeMicroseconds(1500 + motor_speeds[1]);
        right_rear_motor.writeMicroseconds(1500 + motor_speeds[2]);
        right_font_motor.writeMicroseconds(1500 + motor_speeds[3]);
        //Serial1.print(">Error: ");
        //Serial1.println(error_kp);
    }
}