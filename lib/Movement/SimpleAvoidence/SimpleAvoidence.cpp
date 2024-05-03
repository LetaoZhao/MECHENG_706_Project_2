#include <MotorMovement/MotorMovement.hpp>
#include <Sonar.hpp>
#include <Arduino.h>

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