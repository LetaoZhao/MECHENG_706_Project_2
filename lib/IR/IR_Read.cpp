#include <Arduino.h>
#include "GlobalVariable.hpp"
#include <PID_v1.h>



// This file contains the IR sensor conversion.

//----------------------Sensor Reading & conversion to mm-------------------------

// double IR_sensorReadDistance(const char* sensor)
// // input can be : "41_01", "04_02", "02_01", "02_02", "02_03", "02_04",
// // return distance in mm
// {
//   double distance;
//   double sensor_value;
//   if (sensor == "41_01")
//   {
//     sensor_value = analogRead(IR_41_01);
//     distance = 27592 * pow(sensor_value, -1.018);
//   }
//   else if (sensor == "41_02")
//   {
//     sensor_value = analogRead(IR_41_02);
//     distance = 7935.4 * pow(sensor_value, -0.827);
//   }
//   else if (sensor == "41_03")
//   {
//     sensor_value = analogRead(IR_41_03);
//     distance = 30119 * pow(sensor_value, -1.039);
//   }
//   // else if(sensor == "2Y_01")
//   // {
//   //   sensor_value = analogRead(IR_2Y_01);
//   //   distance = 1888777 * pow(sensor_value, -1.237);
//   // }
//   else if (sensor == "2Y_02")
//   {
//     sensor_value = analogRead(IR_2Y_02);
//     distance = 92838 * pow(sensor_value, -1.097);
//   }
//   // else if(sensor = "2Y_03")
//   // {
//   //   sensor_value = analogRead(IR_2Y_03);
//   //   distance = 7927.4 * pow(sensor_value, -0.687);
//   // }
//   else if (sensor = "2Y_04")
//   {
//     sensor_value = analogRead(IR_2Y_04);
//     distance = 50857 * pow(sensor_value, -0.994);
//   }
//   else
//   {
//     Serial1.println("Invalid sensor");
//     distance = 0;
//   }
//   return distance;
// }

// double IR_sensorReadDistance(const char* sensor)
// // input can be : "//41_01", "41_02", "41_03", "//2Y_01", "2Y_02", "//2Y_03", "2Y_04",
// // return distance in mm
// {
//   double distance;
//   double sensor_value;
//   double temp_reading = 0.0;
//   // if (sensor == "41_01")  //old
//   // {
//   //   sensor_value = analogRead(IR_41_01);
//   //   distance = 27592 * pow(sensor_value, -1.018);
//   // }
//   // if (sensor == "41_02")  //old
//   // {
//   //   sensor_value = analogRead(IR_41_02);
//   //   distance = 7935.4 * pow(sensor_value, -0.827);
//   // }
//   if (sensor == "41_02")  //new
//   {
//     for (int i = 0; i <= 9; i++)
//     {
//       temp_reading = temp_reading + analogRead(IR_41_02);
//     }
//     sensor_value = temp_reading/10;
//     distance = 21147 * pow(sensor_value, -0.985) - 7;
//   }
//   else if (sensor == "41_03") //old
//   {
//     sensor_value = analogRead(IR_41_03);
//     distance = 30119 * pow(sensor_value, -1.039);
//   }
//   // else if(sensor == "2Y_01") //old
//   // {
//   //   sensor_value = analogRead(IR_2Y_01);
//   //   distance = 1888777 * pow(sensor_value, -1.237);
//   // }
//   else if (sensor == "2Y_02") //old
//   {
//     sensor_value = analogRead(IR_2Y_02);
//     distance = 92838 * pow(sensor_value, -1.097);
//   }
//   // else if(sensor = "2Y_03") //old
//   // {
//   //   sensor_value = analogRead(IR_2Y_03);
//   //   distance = 7927.4 * pow(sensor_value, -0.687);
//   // }
//   // else if (sensor = "2Y_04") //old
//   // {
//   //   sensor_value = analogRead(IR_2Y_04);
//   //   distance = 50857 * pow(sensor_value, -0.994);
//   // }
//   else if (sensor = "2Y_04") //new
//   {
//     for (int i = 0; i <= 9; i++)
//     {
//       temp_reading = temp_reading + analogRead(IR_2Y_04);
//     }
//     sensor_value = temp_reading/10;
//     distance = 155559 * pow(sensor_value, -1.185) + 5;
//   }
//   else
//   {
//     Serial1.println("Invalid sensor");
//     distance = 0;
//   }
//   temp_reading = 0.0;
//   return distance;
// }

double IR_sensorReadDistance(const char *sensor)
{
    // input can be : "//41_01", "41_02", "41_03", "//2Y_01", "2Y_02", "//2Y_03", "2Y_04",
    // return distance in mm
    // The new range for 41_XX is 50(mm) to 250(mm), for 2Y_XX is 100(mm) to 250(mm)

    double distance = 0.0;
    double sensor_value = 0.0;
    //double temp_reading = 0.0;
    //int pinNum = 0;

    // if (sensor == "41_02")  //new
    // {
    //   pinNum = IR_41_02;
    // }
    // else if (sensor == "41_03") //new
    // {
    //   pinNum = IR_41_03;
    // }
    // else if (sensor == "2Y_02") //new
    // {
    //   pinNum = IR_2Y_02;
    // }
    // else if (sensor == "2Y_04") //new
    // {
    //   pinNum = IR_2Y_04;
    // }
    // else
    // {
    //   SerialCom->println("Invalid sensor");
    //   distance = 0;
    // }

    // for (int i = 0; i <= 9; i++)
    // {
    //   temp_reading = temp_reading + analogRead(pinNum);
    //   delay(5);
    // }
    // sensor_value = temp_reading/10;

    // if (sensor == "41_02")  //new
    // {
    //   distance = 21147 * pow(sensor_value, -0.985) - 7;
    //   if(distance >= 200)
    //   {
    //     distance = distance - 20;
    //   }
    //   else if(distance > 150)
    //   {
    //     distance  = distance -10;
    //   }
    // }
    // else if (sensor == "41_03") //new
    // {
    //   distance = 19000 * pow(sensor_value, -0.98) + 10;  //distance = 21022 * pow(sensor_value, -0.967)
    //   if(distance >= 200)
    //   {
    //     distance = distance - 5;
    //   }
    //   if(distance < 200)
    //   {
    //     distance  = distance + 5;
    //   }
    // }
    // else if (sensor == "2Y_02") //new
    // {
    //   distance = 142772 * pow(sensor_value, -1.149) + 10;
    //   if(distance >= 250)
    //   {
    //     distance = distance - 35;
    //   }
    //   else if(distance >= 200)
    //   {
    //     distance = distance - 25;
    //   }
    //   else if(distance >= 150)
    //   {
    //     distance = distance - 18;
    //   }
    //   else if(distance >= 100)
    //   {
    //     distance = distance - 10;
    //   }
    // }
    // else if (sensor == "2Y_04") //new
    // {
    //   distance = 155559 * pow(sensor_value, -1.185) + 5;
    // }
    // else
    // {
    //   SerialCom->println("Invalid sensor");
    //   distance = 0;
    // }

    // temp_reading = 0.0;
    // return distance;

    // if (sensor == "41_01")  //old
    // {
    //   sensor_value = analogRead(IR_41_01);
    //   distance = 27592 * pow(sensor_value, -1.018);
    // }
   if (strcmp(sensor, "41_02") == 0)
{
    sensor_value = analogRead(IR_41_02);
    distance = 7935.4 * pow(sensor_value, -0.827) + 17;
}
else if (strcmp(sensor, "41_03") == 0)
{
    sensor_value = analogRead(IR_41_03);
    distance = 30119 * pow(sensor_value, -1.039);
}
else if (strcmp(sensor, "2Y_02") == 0)
{
    sensor_value = analogRead(IR_2Y_02);
    distance = 92838 * pow(sensor_value, -1.097) + 10;
}
else if (strcmp(sensor, "2Y_04") == 0)
{
    sensor_value = analogRead(IR_2Y_04);
    distance = 50857 * pow(sensor_value, -0.994);
}
else
{
    // SerialCom->println("Invalid sensor"); // Uncomment and adjust according to your setup
    distance = 0;
}
    //temp_reading = 0.0;
    return distance;
}


double IR_filter()
{
    IR_sum_left = IR_sum_left - IR_values_left[29]; //remove the last value of the array from the sum total
    IR_sum_right = IR_sum_right - IR_values_right[29]; //remove the last value of the array from the sum total

    //right shift the array
    for (int i = 29; i >= 1; i--)
    { 
    IR_values_left[i] = IR_values_left[i-1];
    IR_values_right[i] = IR_values_right[i-1];
    }

    //Get new values
    IR_values_left[0] = IR_sensorReadDistance("41_02");
    IR_values_right[0] = IR_sensorReadDistance("41_03");

    //Add new value to the sum
    IR_sum_left += IR_values_left[0];
    IR_sum_right += IR_values_right[0];

    //Calculate teh average
    IR_left_avg = IR_sum_left/30;
    IR_right_avg = IR_sum_right/30;

}

void IR_filter_initialize()
{
    float sum_right = 0;
    float sum_left = 0;
    // fills the filter array with values
  for(int i = 0; i < 30; i++)
  {
    IR_values_left[i] = IR_sensorReadDistance("41_02"); 
    sum_left += IR_values_left[i];
    IR_values_right[i] = IR_sensorReadDistance("41_03");
    sum_right += IR_values_right[i];
  }

  IR_left_avg =sum_left/30;
  IR_right_avg = sum_right/30;
}
