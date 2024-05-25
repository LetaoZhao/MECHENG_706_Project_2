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


    while(!isReached){
        switch (avoidenceState){
            case 0:
                GyroTurn1(90);
                delay(1000);
                
                if((HC_SR04_range() < 10) || IR_sensorReadDistance("4102") < 100 || IR_sensorReadDistance("4103") < 100){
                    avoidenceState = 100;
                }
                else{
                    avoidenceState = 1;
                }
                break;
            case 1:
                reverse();
                delay(300);
                stop();
                avoidenceState = 2;
                break;

            case 2:
                GyroTurn1(-90);
                delay(1000);
                
                if((HC_SR04_range() < 10) || IR_sensorReadDistance("4102") < 100 || IR_sensorReadDistance("4103") < 100){
                    avoidenceState = 0;
                }
                else{
                    avoidenceState = 999;//move a bit forward
                }
                break;





            case 100:
                GyroTurn1(180);
                if((HC_SR04_range() < 10) || IR_sensorReadDistance("4102") < 100 || IR_sensorReadDistance("4103") < 100){
                    Serial1.print("both sides detected");
                }
                else{
                    avoidenceState = 101;
                }
                break;
            case 101:
                reverse();
                delay(300);
                stop();
                avoidenceState = 102;
                break;

            case 102:
                GyroTurn1(90);
                delay(1000);
                
                if((HC_SR04_range() < 10) || IR_sensorReadDistance("4102") < 100 || IR_sensorReadDistance("4103") < 100){
                    avoidenceState = 100;
                }
                else{
                    avoidenceState = 999;//move a bit forward
                }
                break;



            case 999:
                reverse();
                delay(300);
                stop();
                isReached = true;
                break;

            

                
                
            default:
                break;
        }
       

    }
    
    

}