#include <MotorMovement/MotorMovement.hpp>
#include <Sonar.hpp>
#include <Arduino.h>
#include <GyroTurn/GyroTurn.hpp>
#include <PhotoTransistor.hpp>
#include <IR_Read.hpp>
#include <GlobalVariable.hpp>
#include <Gyro.hpp>
#include <Gyro.hpp>
#include <SerialComs.hpp>

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

bool Turn_Until_Free()
{
    // Serial1.println("Avoiding!");
    IR_read_filter();
    readGyro1();
    sonar_reading = HC_SR04_range();
    //check if the object is on the right
    if (IR_left_avg > IR_right_avg)
    {
        //object is on the right
        ccw_low(); //not sure if right direction

        //set a variable here to indicate the avoidance direciton
    }
    //check if the object is on the left
    else if (IR_right_avg > IR_left_avg)
    {
        cw_low();

        //set a variable here to indicate the aviodance direciton
    }
    //in case neither sensor has a reading
    else
    {
        cw_low();
    }

    //check if clear of object
    if((IR_left_avg > 200) && (IR_right_avg > 200) && (sonar_reading > 15))
    {
        stop();
        return true;
        //update the state
    }
    print_sensors();
    return false;
}
    
    // IR_read_filter();

    // if (IR_right_avg < 100)
    // {
    //     while(IR_right_avg < 100)
    //     {   
    //         ccw_low();

    //         //get reasings
    //         sonar_reading = HC_SR04_range();
    //         IR_read_filter();
    //         turn_time_count++;
    //         //print
    //         // print_sensors();
    //         delay(50);
    //     }
    // }
    // else if (IR_left_avg < 200)
    // {
    //     while(IR_left_avg < 200)
    //     {   
    //         cw_low();
    //         IR_read_filter();
    //         sonar_reading = HC_SR04_range();
    //         turn_time_count--;
    //         // print_sensors();
    //         delay(50);
            
    //     }
    // }
    // else
    // {
    //     while(sonar_reading < 10)
    //     {
    //         Serial1.println("Sonar Turn");
    //         ccw_low();
    //         sonar_reading = HC_SR04_range();
    //         turn_time_count++;
    //         IR_read_filter();
    //         // print_sensors();
    //         delay(50);

    //      }
    // stop();
    // }   

    // return turn_time_count;
// }

bool Drive_Until_Free()
{
    //simple drive forward function
    sonar_reading = HC_SR04_range();
    IR_read_filter();



    // check for objects
    if((sonar_reading < 15) || (IR_left_avg < 300) || (IR_right_avg < 300) || (IR_left_45_avg < 300) || (IR_right_45_avg < 300))
    {

        stop();
        //there is an object
        return false;
    }
    else
    {
        reverse();
    }
    return true;
}


void ObjectAvoidence(){
    stop();
    bool isReached = false;
    int avoidenceState = 0;

    int pass_time_count_desirde = 10; //50*count = goStraightTime
    int pass_time_count = 0;

    bool passed = 1;
    int turn_time_count = 0;
    //int temp_turn_time_count = 0;


    while(!isReached){
        Serial1.println(avoidenceState);
        switch (avoidenceState){
            case 0:
                //turn until no object in front of robot
                manual_gyro_count = 0;
                turn_time_count = Turn_Until_Free();
                avoidenceState = 1;
                break;

            case 10:
                turn_time_count = Turn_Until_Free();
                avoidenceState = 1;
                break;

            case 1: 
                //move pass object
                pass_time_count = 0;
                passed = 1;
                while(pass_time_count < pass_time_count_desirde)
                {
                    reverse_low();
                    if((HC_SR04_range() < 17)||(IR_sensorReadDistance("41_02") < 200)||(IR_sensorReadDistance("41_03") < 130))
                    {
                        //if another object, avoid it again
                        stop();
                        pass_time_count = 13;
                        avoidenceState = 10;
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
                if(manual_gyro_count < 0)
                {
                    while(manual_gyro_count <= 0)
                    {
                        ccw_low();
                        manual_gyro_count++;
                        delay(50);
                    }
                }
                else
                {
                    while(manual_gyro_count >= 0)
                    {
                        cw_low();
                        manual_gyro_count--;
                        delay(50);
                    }
                }
                stop();
                avoidenceState = 3;

            case 3: //move forward a bit
                pass_time_count = 0;
                passed = 1;
                while(pass_time_count < pass_time_count_desirde)
                {
                    reverse_low();
                    if((HC_SR04_range() < 17)||(IR_sensorReadDistance("41_02") < 200)||(IR_sensorReadDistance("41_03") < 130))
                    {
                        //if another object, avoid it again
                        stop();
                        pass_time_count = 13;
                        avoidenceState = 10;
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
            

                
                