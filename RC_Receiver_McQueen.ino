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

// Radio channels (126 channels are supported)
byte chPointer = 0; // Channel 1 (the first entry of the array) is active by default
const byte NRFchannel[] {
  1, 2
};

// the ID number of the used "radio pipe" must match with the selected ID on the transmitter!
// 10 ID's are available @ the moment
const uint64_t pipeIn[] = {
  0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL,
  0xE9E8F0F0B6LL, 0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL, 0xE9E8F0F0B0LL
};
const int maxVehicleNumber = (sizeof(pipeIn) / (sizeof(uint64_t)));

// Hardware configuration: Set up nRF24L01 radio on hardware SPI bus & pins A0 (CE) & A1 (CSN)
RF24 radio(A0, A1);

// The size of this struct should not exceed 32 bytes
struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  boolean mode1 = false; // Mode1 (toggle speed limitation)
  boolean mode2 = false; // Mode2 (toggle acc. / dec. limitation)
  boolean momentary1 = false; // Momentary push button
  byte pot1; // Potentiometer
};
RcData data;

// This struct defines data, which are embedded inside the ACK payload
struct ackPayload {
  float vcc; // vehicle vcc voltage
  float batteryVoltage; // vehicle battery voltage
  boolean batteryOk = true; // the vehicle battery voltage is OK!
  byte channel = 1; // the channel number
};
ackPayload payload;

// Battery voltage detection pin
#define BATTERY_DETECT_PIN A3 // The 10k & 10k battery detection voltage divider is connected to pin A3

// Create Servo object
Servo steeringServo;

// Status LED objects
statusLED battLED(true); // true = inversed! (wired from pin to vcc)

// Initialize DRV8833 H-Bridge
#define motor_in1 5 // define motor pin numbers
#define motor_in2 6
// NOTE: The first pin must always be PWM capable, the second only, if the last parameter is set to "true"
// SYNTAX: IN1, IN2, min. input value, max. input value, neutral position width
// invert rotation direction, true = both pins are PWM capable
DRV8833 Motor(motor_in1, motor_in2, 0, 100, 2, false, true);

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
  radio.setChannel(NRFchannel[chPointer]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(pipeIn[vehicleNumber - 1], true); // Ensure autoACK is enabled
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5, 5);                  // 5x250us delay (blocking!!), max. 5 retries
  //radio.setCRCLength(RF24_CRC_8);          // Use 8-bit CRC for performance

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
}

//
// =======================================================================================================
// LED
// =======================================================================================================
//

void led() {

  // NOTE: The on board LED 17 (RX) is used

  // Red LED (ON = battery empty, blinking = OK
  if (payload.batteryOk) {
    battLED.flash(140, 150, 500, vehicleNumber); // ON, OFF, PAUSE, PULSES
  } else {
    battLED.on(); // Always ON = battery low voltage
  }
}

//
// =======================================================================================================
// READ RADIO DATA
// =======================================================================================================
//

void readRadio() {

  static unsigned long lastRecvTime = 0;
  byte pipeNo;

  if (radio.available(&pipeNo)) {
    radio.writeAckPayload(pipeNo, &payload, sizeof(struct ackPayload) );  // prepare the ACK payload
    radio.read(&data, sizeof(struct RcData)); // read the radia data and send out the ACK payload
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

  // Switch channel
  if (millis() - lastRecvTime > 500) {
    chPointer ++;
    if (chPointer >= sizeof((*NRFchannel) / sizeof(byte))) chPointer = 0;
    radio.setChannel(NRFchannel[chPointer]);
    payload.channel = NRFchannel[chPointer];
  }

  if (millis() - lastRecvTime > 1000) { // bring all servos to their middle position, if no RC signal is received during 1s!
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
  steeringServo.write(map(data.axis1, 100, 0, 61, 104) ); // 0 - 100% = 61 - 104° ( 90° is the center in theory) R - L
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
// CHECK RX BATTERY & VCC VOLTAGES
// =======================================================================================================
//

boolean battSense = true;
float cutoffVoltage = 2.9;

void checkBattery() {

  // Every 2000 ms
  static unsigned long lastTrigger;
  if (millis() - lastTrigger >= 2000) {
    lastTrigger = millis();

    // Read both averaged voltages
    payload.batteryVoltage = batteryAverage();
    payload.vcc = vccAverage();

    if (battSense) { // Observe battery voltage
      if (!payload.batteryVoltage >= cutoffVoltage) payload.batteryOk = false;
    }
    else { // Observe vcc voltage
      if (!payload.vcc >= cutoffVoltage) payload.batteryOk = false;
    }
  }
}

// Voltage read & averaging subfunctions -----------------------------------------
// vcc ----
float vccAverage() {
  static int raw[6];

  if (raw[0] == 0) {
    for (int i = 0; i <= 5; i++) {
      raw[i] = readVcc(); // Init array
    }
  }

  raw[5] = raw[4];
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  raw[0] = readVcc();
  float average = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5]) / 6000.0;
  return average;
}

// battery ----
float batteryAverage() {
  static int raw[8];

  if (!battSense) return 0;

  if (raw[0] == 0) {
    for (int i = 0; i <= 7; i++) {
      raw[i] = analogRead(BATTERY_DETECT_PIN); // Init array
    }
  }

  raw[7] = raw[6];
  raw[6] = raw[5];
  raw[5] = raw[4];
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  raw[0] = analogRead(BATTERY_DETECT_PIN);
  float average = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5] + raw[6] + raw[7]) / 1240.0; // 1023steps / 6.6V * 8 = 1240
  return average;
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

  // Read radio data from transmitter
  readRadio();

  // Write the steering servo position
  writeSteeringServo();

  // Drive the main motor
  driveMotor();

  // LED
  led();

  // Battery check
  checkBattery();
}

