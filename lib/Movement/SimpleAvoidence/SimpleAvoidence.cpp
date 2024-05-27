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

void ObjectAvoidence(){
    bool isReached = false;
    int avoidenceState = 0;
    int turn90Mills = 600;

    int goStraightTime = 600;


    while(!isReached){

        unsigned long int currentMills = millis();
        unsigned long int startMills;
        switch (avoidenceState){
            case 0:
            //turn left 90 degree
                ccw();
                delay(turn90Mills);
                stop();
                delay(1000);
                
                if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <150 || IR_sensorReadDistance("41_03") <150){
                    //if left has object, turn right 180 degree
                    avoidenceState = 99;
                    
                }
                else{
                    avoidenceState = 1;
                    startMills = currentMills;
                }
                break;
            case 1:
            //move forward
                reverse();
                delay(goStraightTime);
                stop();
                avoidenceState = 2;
                break;

            case 2:
            //turn right 90 degree
                cw();
                delay(turn90Mills);
                stop();
                delay(1000);
                
                if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <150 || IR_sensorReadDistance("41_03") <150){
                    avoidenceState = 0;
                }
                else{
                    avoidenceState = 999;//move a bit forward
                }
                break;




            case 99:
                //turn right 90 degree, it should face the fire
                cw();
                delay(turn90Mills + 300);
                avoidenceState = 100;
                break;

            case 100:
                //turn right 90 degree
                cw();
                delay(turn90Mills);
                stop();
                if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <150 || IR_sensorReadDistance("41_03") <150){
                    Serial.print("both sides detected");
                }
                else{
                    avoidenceState = 101;
                }
                break;
            case 101:
                //move forward
                reverse();
                delay(goStraightTime);
                stop();
                avoidenceState = 102;
                break;

            case 102:
                //turn left 90 degree
                ccw();
                delay(turn90Mills);
                stop();
                delay(1000);
                
                if((HC_SR04_range() <15) || IR_sensorReadDistance("41_02") <150 || IR_sensorReadDistance("41_03") <150){
                    //if there is still an object  in front, redo the whole state 100-102 to move sidewards again
                    avoidenceState = 100;
                }
                else{
                    avoidenceState = 199;//move a bit forward
                }
                break;



            case 999:
                //move pass the obstacle
                reverse();
                delay(goStraightTime);
                stop();
                isReached = true;
                break;

            case 199:
                reverse();
                delay(goStraightTime);
                while(TurnToFire() == false){
                    delay(10);
                }
                stop();
                isReached = true;
                break;
            

                
                
            default:
                break;
        }
       

    }
    
    

}