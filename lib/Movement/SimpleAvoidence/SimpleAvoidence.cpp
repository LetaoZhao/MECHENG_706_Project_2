#include <MotorMovement/MotorMovement.hpp>
#include <Sonar.hpp>
#include <Arduino.h>
#include <GyroTurn/GyroTurn.hpp>
#include <PhotoTransistor.hpp>
#include <IR_Read.hpp>

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
        Serial.print(avoidenceState);

        unsigned long int currentMills = millis();
        unsigned long int startMills;
        switch (avoidenceState){
            case 0:
                ccw();
                delay(turn90Mills);
                stop();
                delay(1000);
                
                if((HC_SR04_range() < 10) || IR_sensorReadDistance("41_02") < 100 || IR_sensorReadDistance("41_03") < 100){
                    avoidenceState = 100;
                    
                }
                else{
                    avoidenceState = 1;
                    startMills = currentMills;
                }
                break;
            case 1:
                reverse();
                delay(goStraightTime);
                stop();
                avoidenceState = 2;
                break;

            case 2:
                cw();
                delay(turn90Mills);
                stop();
                delay(1000);
                
                if((HC_SR04_range() < 10) || IR_sensorReadDistance("41_02") < 100 || IR_sensorReadDistance("41_03") < 100){
                    avoidenceState = 0;
                }
                else{
                    avoidenceState = 999;//move a bit forward
                }
                break;





            case 100:
                ccw();
                delay(2*turn90Mills);
                stop();
                if((HC_SR04_range() < 10) || IR_sensorReadDistance("41_02") < 100 || IR_sensorReadDistance("41_03") < 100){
                    Serial.print("both sides detected");
                }
                else{
                    avoidenceState = 101;
                }
                break;
            case 101:
                reverse();
                delay(goStraightTime);
                stop();
                avoidenceState = 102;
                break;

            case 102:
                ccw();
                delay(turn90Mills);
                stop();
                delay(1000);
                
                if((HC_SR04_range() < 10) || IR_sensorReadDistance("41_02") < 100 || IR_sensorReadDistance("41_03") < 100){
                    avoidenceState = 100;
                }
                else{
                    avoidenceState = 999;//move a bit forward
                }
                break;



            case 999:
                reverse();
                delay(goStraightTime);
                stop();
                isReached = true;
                break;

            

                
                
            default:
                break;
        }
       

    }
    
    

}