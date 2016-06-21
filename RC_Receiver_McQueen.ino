// Disney "Lightning McQueen 95" toy car, conversion to proper Arduino RC control
// 3.3V, 8MHz Pro Micro, DRV8833 DC motor driver, 2.4GHz NRF24L01 radio module

//
// =======================================================================================================
// BUILD OPTIONS (comment out unneeded options)
// =======================================================================================================
//

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

//
// =======================================================================================================
// INCLUDE LIRBARIES
// =======================================================================================================
//

// Libraries
#include <SPI.h>
#include <RF24.h> // Installed via Tools > Board > Boards Manager > Type RF24
#include <printf.h>
#include <Servo.h>
#include <SimpleTimer.h> // https://github.com/jfturcot/SimpleTimer
#include <PWMFrequency.h> // https://github.com/kiwisincebirth/Arduino/tree/master/libraries/PWMFrequency
#include <DRV8833.h> // https://github.com/TheDIYGuy999/DRV8833
#include <statusLED.h> // https://github.com/TheDIYGuy999/statusLED

#include "readVCC.h"

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// Vehicle address
int vehicleNumber = 1; // This number must be unique for each vehicle!
const int maxVehicleNumber = 5;

// the ID number of the used "radio pipe" must match with the selected ID on the transmitter!
const uint64_t pipeIn[maxVehicleNumber] = { 0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL };

// The size of this struct should not exceed 32 bytes
struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  boolean mode1 = false; // Speed limitation
  boolean mode2 = false;
};

RcData data;

// Hardware configuration: Set up nRF24L01 radio on hardware SPI bus & pins A0 (CE) & A1 (CSN)
RF24 radio(A0, A1);

// Battery
boolean batteryOK = false;

// Create Servo object
Servo steeringServo;

// Status LED objects
statusLED battLED(true); // true = inversed!

// Timer
SimpleTimer timer;

// Initialize DRV8833 H-Bridge
#define motor_in1 5 // define motor pin numbers
#define motor_in2 6
// NOTE: The first pin must always be PWM capable, the second only, if the last parameter is set to "true"
// SYNTAX: IN1, IN2, min. input value, max. input value, neutral position width
// invert rotation direction, true = both pins are PWM capable
DRV8833 Motor(motor_in1, motor_in2, 0, 100, 6, false, true);

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  printf_begin();
  delay(3000);
#endif

  // LED setup
  battLED.begin(17); // pin 17 = RX LED!

  // Radio setup
  radio.begin();
  radio.setChannel(1);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);                  // Ensure autoACK is enabled
  radio.setRetries(1, 5);                  // 1x250us delay (blocking!!), max. 5 retries
  radio.setCRCLength(RF24_CRC_8);          // Use 8-bit CRC for performance

#ifdef DEBUG
  radio.printDetails();
  delay(3000);
#endif

  radio.openReadingPipe(1, pipeIn[vehicleNumber - 1]);
  radio.startListening();

  // Steering servo is on pin 2
  steeringServo.attach(2);

  // Motor PWM frequency
  setPWMPrescaler(motor_in1, 1); // 123Hz = 256,  492Hz = 64, 3936Hz = 8, 31488Hz = 1
  setPWMPrescaler(motor_in2, 1);

  // All axis to neutral position
  data.axis1 = 50;
  data.axis2 = 50;
  data.axis3 = 50;
  data.axis4 = 50;

  timer.setInterval(100, checkBattery); // Check battery voltage every 100ms
}

//
// =======================================================================================================
// LED
// =======================================================================================================
//

void led() {

  // NOTE: The LED 17 (RX) is inversed! OFF means ON etc.!!!!*******************

  //batteryOK = false;

  // Red LED (ON = battery empty, blinking = OK
  if (batteryOK) {
    battLED.flash(140, 150, 500, vehicleNumber); // ON, OFF, PAUSE, PULSES
    //battLED.flash(100, 30, 0, 0); // ON, OFF, PAUSE, PULSES
  } else {
    battLED.off(); // Always ON = battery low voltage
  }
}

//
// =======================================================================================================
// READ RADIO DATA
// =======================================================================================================
//

void readRadio() {

  static unsigned long lastRecvTime = 0;

  if (radio.available()) {
    radio.read(&data, sizeof(RcData));
    lastRecvTime = millis();
#ifdef DEBUG
    Serial.print(data.axis1);
    Serial.print("\t");
    Serial.print(data.axis2);
    Serial.print("\t");
    Serial.print(data.axis3);
    Serial.print("\t");
    Serial.print(data.axis4);
    Serial.println("\t");
#endif
  }

  if (millis() - lastRecvTime > 1000) { // bring all servos to their middle position, if no RC signal is received for 1s!
    data.axis1 = 50; // Aileron (Steering for car)
    data.axis2 = 50; // Elevator
    data.axis3 = 50; // Throttle
    data.axis4 = 50; // Rudder
#ifdef DEBUG
    Serial.println("No Radio Available - Check Transmitter!");
#endif
  }
}

//
// =======================================================================================================
// WRITE STEERING SERVO POSITION
// =======================================================================================================
//

void writeSteeringServo() {
  steeringServo.write(map(data.axis1, 100, 0, 68, 101) ); // 0 - 100% = 86 - 105° ( 90% is the center in theory) R - L
}

//
// =======================================================================================================
// DRIVE MOTOR
// =======================================================================================================
//

void driveMotor() {

  int maxPWM;
  int maxAcceleration;
  
  // Speed limitation (max. is 255)
  if (data.mode1) {
    maxPWM = 170; // Limited
  } else {
    maxPWM = 255; // Full
  }

  // Acceleration & deceleration limitation (ms per 1 step PWM change)
  if (data.mode2) {
    maxAcceleration = 12; // Limited
  } else {
    maxAcceleration = 7; // Full
  }
  
  // ***************** Note! The ramptime is intended to protect the gearbox! *******************
  // SYNTAX: Input value, max PWM, ramptime in ms per 1 PWM increment
  // true = brake active, false = brake in neutral position inactive
  Motor.drive(data.axis3, maxPWM, maxAcceleration, true, false);
}

//
// =======================================================================================================
// CHECK BATTERY VOLTAGE
// =======================================================================================================
//

void checkBattery() {
  float batteryVolt = readVcc() / 1000.0 ;

  if (batteryVolt >= 3.0) {
    batteryOK = true;
#ifdef DEBUG
    Serial.print(batteryVolt);
    Serial.println(" V. Battery OK");
#endif
  } else {
    batteryOK = false;
#ifdef DEBUG
    Serial.print(batteryVolt);
    Serial.println(" V. Battery empty!");
#endif
  }
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

  // Timer
  timer.run();

  // Read radio data from transmitter
  readRadio();

  // Write the steering servo position
  writeSteeringServo();

  // Drive the main motor
  driveMotor();

  // LED
  led();
}

