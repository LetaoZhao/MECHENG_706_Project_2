#include <MotorMovement/MotorMovement.hpp>
#include <Sonar.hpp>
#include <Arduino.h>
#include <GyroTurn/GyroTurn.hpp>
#include <PhotoTransistor.hpp>
#include <IR_Read.hpp>
#include <GlobalVariable.hpp>
#include <Gyro.hpp>
#include <Gyro.hpp>

void SimpleAvoidence()
{
    bool isReach = 0;
    float distance_sonar = 0.0;
    bool isObject = 1;
    int strafe_count = 0;

    while (!isReach)
    {
        reverse();
        distance_sonar = HC_SR04_range();

        if (distance_sonar <= 15)
        {
            while (isObject)
            {
                strafe_left(500);
                delay(200);
                distance_sonar = HC_SR04_range();
                strafe_count++;

                if (distance_sonar > 15)
                {
                    isObject = 0;
                    strafe_count = 0;
                }
                else if (strafe_count >= 3)
                {
                    isObject = 0;
                    isReach = 1;
                }
                else{}
            }
        }
    }

    while (true)
    {
        ccw();
    }
}

void Keep_Gyro_Zero()
{
    readGyro1();
    while(abs(currentAngle) >= 5)
    {
        if(currentAngle < 0)
        {
            ccw_low();
        }
        else
        {
            cw_low();
        }
        delay(50);
        readGyro1();
    }
    stop();
}

int Turn_Until_Free()
{
    int turn_time_count = 0;

    float left_distance_IR = (IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02"))/3;
    float right_distance_IR = (IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03"))/3;
    float sonar_distance = HC_SR04_range();

    if((left_distance_IR < 200)||(sonar_distance < 10))
    {
        while((left_distance_IR < 200)||(sonar_distance < 10))
        {
            cw_low();
            left_distance_IR = (IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02"))/3;
            right_distance_IR = (IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03"))/3;
            sonar_distance = HC_SR04_range();
            delay(50);
        }
        turn_time_count--;
    }
    else
    {
        while((right_distance_IR < 100)||(sonar_distance < 10))
        {
            ccw_low();
            left_distance_IR = (IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02"))/3;
            right_distance_IR = (IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03"))/3;
            sonar_distance = HC_SR04_range();
            delay(50);
        }
        turn_time_count++;
    }
    stop();

    return turn_time_count;
}

void ObjectAvoidence(){
    bool isReached = false;
    int avoidenceState = 0;

    int pass_time_count_desirde = 12; //50*count = goStraightTime
    int pass_time_count = 0;

    bool passed = 1;
    int turn_time_count = 0;
    int temp_turn_time_count = 0;


    while(!isReached){
        switch (avoidenceState){
            case 0:
                //turn untill no object in front of robot
                temp_turn_time_count = Turn_Until_Free();
                avoidenceState = 1;
                break;

            case 1: 
                //move pass object
                pass_time_count = 0;
                passed = 1;
                while(pass_time_count < pass_time_count_desirde)
                {
                    reverse();
                    if((HC_SR04_range() < 10)||(IR_sensorReadDistance("41_02") < 200)||(IR_sensorReadDistance("41_03") < 100))
                    {
                        //if another object, avoid it again
                        stop();
                        pass_time_count = 13;
                        avoidenceState = 0;
                        passed = 0;
                    }
                    delay(50);   
                    pass_time_count++;             
                }

                //if passed the object
                if(passed)
                {
                    stop();
                    avoidenceState = 2;
                }       
                break;

            case 2: //turn back to rough fire direction
                turn_time_count= 0;
                if(temp_turn_time_count < 0) //if after turn right, cw
                {
                    while(turn_time_count >= temp_turn_time_count)
                    {
                        ccw_low(); //turn left back
                        delay(50);
                        turn_time_count--;
                    }
                }
                else //if after turn left, ccw
                {
                    while(turn_time_count <= temp_turn_time_count)
                    {
                        cw_low(); //turn right back
                        delay(50);
                        turn_time_count++;
                    }
                }
                avoidenceState = 3;

            case 3: //move forward a bit
                pass_time_count = 0;
                passed = 1;
                while(pass_time_count < pass_time_count_desirde)
                {
                    reverse();
                    if((HC_SR04_range() < 10)||(IR_sensorReadDistance("41_02") < 200)||(IR_sensorReadDistance("41_03") < 100))
                    {
                        //if another object, avoid it again
                        stop();
                        pass_time_count = 13;
                        avoidenceState = 0;
                        passed = 0;
                    }
                    delay(50);   
                    pass_time_count++;             
                }

                //if passed the object
                if(passed)
                {
                    stop();
                    isReached = true; //can traking the fire again
                }       
                break;

            default:
                break;
        }
    }
}





            // case 100:
            //     //turn right 90 degree, it should face the fire
            //     cw();
            //     delay(turn90Mills);
            //     avoidenceState = 101;
            //     break;

            // case 101:
            //     //turn right 90 degree
            //     cw();
            //     delay(turn90Mills);
            //     stop();
            //     if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <200 || IR_sensorReadDistance("41_03") <200){
            //         Serial.print("both sides detected");
            //     }
            //     else{
            //         avoidenceState = 102;
            //     }
            //     break;
            // case 102:
            //     //move forward
            //     temp_time = millis();
            //     while((millis() - temp_time) <= goStraightTime*2)
            //     {
            //         reverse();
            //         if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <200 || IR_sensorReadDistance("41_03") <200)
            //         {
            //             stop();
            //             temp_time = temp_time - goStraightTime*2;
            //         }
            //     }
            //     stop();
            //     avoidenceState = 103;
            //     break;

            // case 103:
            //     //turn left 90 degree
            //     ccw();
            //     delay(turn90Mills);
            //     stop();
            //     delay(1000);
                
            //     if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <200 || IR_sensorReadDistance("41_03") <200){
            //         //if there is still an object  in front, redo the whole state 100-102 to move sidewards again
            //         avoidenceState = 101;
            //     }
            //     else{
            //         avoidenceState = 199;//move a bit forward
            //     }
            //     break;



            // case 999:
            //     //move pass the obstacle
            //     temp_time = millis();
            //     while((millis() - temp_time) <= goStraightTime*2)
            //     {
            //         reverse();
            //         if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <150 || IR_sensorReadDistance("41_03") <150)
            //         {
            //             stop();
            //             temp_time = temp_time - goStraightTime*2;
            //         }
            //     }
            //     delay(goStraightTime);
            //     stop();
            //     isReached = true;
            //     break;

            // case 199:
            //     temp_time = millis();
            //     while((millis() - temp_time) <= goStraightTime*2)
            //     {
            //         reverse();
            //         if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <150 || IR_sensorReadDistance("41_03") <150)
            //         {
            //             stop();
            //             temp_time = temp_time - goStraightTime*2;
            //         }
            //     }
            //     delay(goStraightTime);
            //     while(TurnToFire() == false){
            //         delay(10);
            //     }
            //     stop();
            //     isReached = true;
            //     break;
            

                
                