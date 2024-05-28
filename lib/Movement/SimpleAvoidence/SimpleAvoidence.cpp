#include <MotorMovement/MotorMovement.hpp>
#include <Sonar.hpp>
#include <Arduino.h>
#include <GyroTurn/GyroTurn.hpp>
#include <PhotoTransistor.hpp>
#include <IR_Read.hpp>
#include <GlobalVariable.hpp>

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

int Turn_Until_Free()
{
    int time_start = millis();
    int time_used = 0;
    int direction = 0;

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
            delay(10);
        }
        direction = -1;
    }
    else
    {
        while((right_distance_IR < 100)||(sonar_distance < 10))
        {
            ccw_low();
            left_distance_IR = (IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02")+IR_sensorReadDistance("41_02"))/3;
            right_distance_IR = (IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03")+IR_sensorReadDistance("41_03"))/3;
            sonar_distance = HC_SR04_range();
            delay(10);
        }
        direction = 1;
    }

    stop();
    time_used = millis() - time_start;

    return direction*time_used;
}

void ObjectAvoidence(){
    bool isReached = false;
    int avoidenceState = 0;
    int turn90Mills = 600;

    int goStraightTime = 600;

    int temp_time = 0;
    int temp_turn_time = 0;


    while(!isReached){
        switch (avoidenceState){
            case 0:

                temp_turn_time = Turn_Until_Free();
                avoidenceState = 1;
                break;

            case 1: //move pass object
                while(pass_time_count <= )
                {
                    reverse();
                    if((HC_SR04_range() <10)||(IR_sensorReadDistance("41_02") <200)||(IR_sensorReadDistance("41_03")<100))
                    {
                        stop();
                        temp_time = temp_time - goStraightTime*2;
                    }
                    delay(50);                }
                stop();

                avoidenceState = 2;
                break;

            case 2: //turn back to rough fire direction
                if(temp_turn_time < 0) //if after turn left, ccw
                {
                    cw(); //turn right back
                    delay(-1*temp_turn_time); 
                }
                else //if after turn right, ccw
                {
                    ccw(); //turn right back
                    delay(temp_turn_time); 
                }
                
                //if there is an object
                if((HC_SR04_range() <10)||(IR_sensorReadDistance("41_02") <200)||(IR_sensorReadDistance("41_03")<100))
                {
                    //turn back and go around it again
                    if(temp_turn_time < 0) 
                    {
                        ccw();
                        delay(-1*temp_turn_time); 
                    }
                    else //if after turn right, ccw
                    {
                        cw(); 
                        delay(temp_turn_time); 
                    }
                    avoidenceState = 1;
                }
                else
                {
                    avoidenceState = 3;//move a bit forward
                }
                break;

            case 3:
                temp_time = millis(); //move pass the obstacle
                while((millis() - temp_time) <= goStraightTime*2)
                {
                    reverse();
                    if((HC_SR04_range() <10)||(IR_sensorReadDistance("41_02") <200)||(IR_sensorReadDistance("41_03")<100))
                    {
                        stop();
                        temp_time = temp_time - goStraightTime*2;
                    }
                }
                delay(goStraightTime);
                stop();
                isReached = true;
                break;




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
            

                
                
            default:
                break;
        }
       

    }
    
    

}