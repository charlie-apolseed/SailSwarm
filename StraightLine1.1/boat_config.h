#include <Arduino.h>
#include "MPU9250.h"
#include <ESP32Servo.h>
#include <eeprom_utils.h>


//Details for RC control
const int CH_3_PIN = 25;  // Controls sail trim
int ch_3;
const int CH_1_PIN = 26;  // Controls rudder position
int ch_1;
const int CH_4_PIN = 27;  // Used for processing when user wants to switch from RC mode to Autonomous
int ch_4;

//Details for Servo configuration
Servo rudderServo;
const int rudderPin = 14;  // attach rudder servo to pin 18
int centeredRudder = 115;  // the value at which the rudder is centered.

Servo trimServo;
const int trimPin = 13;    // attach sail servo to pin 17
int closeHauledTrim = 50;  // servo position for closehauled trim.
int runningTrim = 140;     // servo position for running trim.


/* Configure the pins used for RC communication */
void configureRCPins() {
  pinMode(CH_3_PIN, INPUT);
  pinMode(CH_1_PIN, INPUT);
  pinMode(CH_4_PIN, INPUT);
}

/* Set up for the servos */
void initServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  rudderServo.setPeriodHertz(50);  // standard 50 hz servo
  rudderServo.attach(rudderPin, 500, 2300);
  rudderServo.write(centeredRudder);
  trimServo.setPeriodHertz(50);  // standard 50 hz servo
  trimServo.attach(trimPin, 500, 2300);
  trimServo.write(closeHauledTrim);
}

/* Used for setting the servo positions for the trim boundaries */
void setTrimConditions(int closeHauled, int running) {
  closeHauledTrim = closeHauled;
  runningTrim = running;
}

/* Used to specify the value where the rudder is centered */
void setCenteredRudder(int val) {
  centeredRudder = val;
}

void setUpMPU() {
  if (!mpu.setup(0x68)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
  mpu.setMagneticDeclination(3.1);  // Set the magnetic declination (Konstanz, Germany)

#if defined(ESP_PLATFORM) || defined(ESP8266)
  EEPROM.begin(0x80);
#endif

  // Serial.println("Accel Gyro calibration will start in 5sec.");
  // Serial.println("Please leave the device still on the flat plane.");
  // mpu.verbose(true);
  // delay(5000);
  // mpu.calibrateAccelGyro();

  // Serial.println("Mag calibration will start in 5sec.");
  // Serial.println("Please Wave device in a figure eight until done.");
  // delay(5000);
  // mpu.calibrateMag();

  // print_calibration();
  // mpu.verbose(false);

  // // save to eeprom
  // saveCalibration();

  // load from eeprom
  loadCalibration();
}

/*
Helper function to print the information when a callibration is completed.
*/
void print_calibration() {
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("mag bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();
}