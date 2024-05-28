// README============================================================================================================start
// This is the main code for MECHENG 706, Project 2, G11
// README==============================================================================================================end

// import libraries==================================================================================================start
#include <Arduino.h>
#include <Servo.h> //Need for Servo pulse output
#include <MotorMovement/MotorMovement.hpp>
#include <GlobalVariable.hpp>
#include <SerialComs.hpp>
#include "Gyro.hpp"
#include <Sonar.hpp>
#include <SimpleAvoidence/SimpleAvoidence.hpp>
#include <TurnTurretTo/TurnTurretTo.hpp>
#include <PhotoTransistor.hpp>
#include <MoveToFireUntil_WithAvoidence/MoveToFireUntil_WithAvoidence.hpp>
// import libraries====================================================================================================end

// variables declearation============================================================================================start
// motors


// const byte left_front = 46;
// const byte left_rear = 47;
// const byte right_rear = 50;
// const byte right_front = 51;

// int speed_val = 500;
// int speed_change;

// // ultrasonic sensor
// const unsigned int MAX_DIST = 23200;

// const int TRIG_PIN = 48;
// const int ECHO_PIN = 49;

// // gyro
// int sensorPin = A2;            // define the pin that gyro is connected
// int sensorValue = 0;           // read out value of sensor
// float gyroSupplyVoltage = 5;   // supply voltage for gyro
// float gyroZeroVoltage = 0;     // the value of voltage when gyro is zero
// float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
// float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less than
//                                // this value will not be ignored
// float gyroRate = 0;            // read out value of sensor in voltage
// double currentAngle = 0;       // current angle calculated by angular velocity integral on

// State machine states
enum STATE
{
  INITIALISING,
  RUNNING,
  STOPPED
};

int pos = 0;
int i;
float sum = 0;

// // Serial Pointer
HardwareSerial *SerialCom;
// variables declearation==============================================================================================end

// functions declearation============================================================================================start
// STATE initialising();
// STATE running();
// STATE stopped();

void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();
boolean is_battery_voltage_OK();

void Analog_Range_A4();
void GYRO_reading();

void speed_change_smooth();
// functions declearation==============================================================================================end

STATE initialising()
{
  // initialising
  SerialCom->println("INITIALISING....");
  // delay(1000); // One second delay to see the serial const char* "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  enable_turret();
  enable_fan();
  SerialCom->println("RUNNING STATE...");

  resetGyro();
  SerialCom->println("Test");
  PhotoTransistor_Initialize();
  SerialCom->println("Initialized the photo sensors");

  return RUNNING;
}

STATE running()
{

  static unsigned long previous_millis;

  // read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500)
  { // Arduino style 500ms timed execution statement
    previous_millis = millis();

    // SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC_SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK())
      return STOPPED;
#endif

//     // turret_motor.write(pos);

//   //   if (pos == 0)
//   //   {
//   //     pos = 45;
//   //   }
//   //   else
//   //   {
//   //     pos = 0;
//   //   }
  }

  return RUNNING;
}

// Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped()
{
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  // int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500)
  { // print massage every 500ms
    previous_millis = millis();
    // SerialCom->println("STOPPED---------");

#ifndef NO_BATTERY_V_OK
    // 500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK())
    {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10)
      { // Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    }
    else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

// setup=============================================================================================================start
void setup()
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");

  delay(1000);
  SerialCom->println("Setup....");
  Serial1.begin(115200);
  Serial.begin(115200);

  // setting up gyro
  resetGyro();

  delay(1000); // settling time but no really needed
}
// setup===============================================================================================================end

// main loop=========================================================================================================start
void loop()
{
  static STATE machine_state = INITIALISING;
  // Finite-state machine Code
  // SerialCom->println("looping---------------------------------------------");
  switch (machine_state)
  {
  case INITIALISING:
  {
    machine_state = initialising();
    break;
  }
  case RUNNING:
  {
    movement_phase = 1;
    
    switch (movement_phase)
    {
    case 0:
      if(TurnToFire() == true){
        movement_phase ++;
      }
      // movement_phase++;
      
      break;
    case 1:
      if(FireHoming_Avoidence() == true)
      {
        movement_phase++;
      }
      break;
    case 2:
      digitalWrite(45,HIGH);
      break;
    }
    // machine_state = running();
    // while(FireHoming() == false)
    // {

      Serial1.print(">Right Average: ");
      Serial1.println(lr_right_avg);
      Serial1.print(">Left Average: ");
      Serial1.println(lr_left_avg);
      Serial1.print(">R-L: " );
      Serial1.println(lr_right_avg-lr_left_avg);
      delay(10);
    // }
    break;
  }
  case STOPPED: // Stop of Lipo Battery voltage is too low, to protect Battery
  {
    machine_state = stopped();
    break;
  }
  };
}
// main loop===========================================================================================================end

// background statemachine===========================================================================================start


void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis)
  {
    indexer++;
    if (indexer > 4)
    {
      fast_flash_millis = millis() + 200;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    }
    else
    {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000)
  {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
  {
    speed_val = 1000;
  }
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  // static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  // the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  // to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  // Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if ((Lipo_level_cal > 0 && Lipo_level_cal < 160) || (true)) //===========================================================================================================================
  {
    // previous_millis = millis();
    // SerialCom->print("Lipo level:");
    // SerialCom->print(Lipo_level_cal);
    // SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    // SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  }
  else
  {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else
    {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif

//------------------------------

void Analog_Range_A4()
{
  // SerialCom->print("Analog Range A4:");
  // SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  // SerialCom->print("GYRO A3:");
  // SerialCom->println(analogRead(A3));
}
#endif
// background statemachine=============================================================================================end
