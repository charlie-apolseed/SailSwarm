#ifndef BOAT_CONFIG_H
#define BOAT_CONFIG_H

#include <Arduino.h>
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
const int rudderPin = 13;  // attach rudder servo to pin 14
int centeredRudder = 115;  // the value at which the rudder is centered.

Servo trimServo;
const int trimPin = 14;    // attach sail servo to pin 13
int closeHauledTrim = 50;  // servo position for closehauled trim.
int runningTrim = 140;     // servo position for running trim.

//Environment and Sailing Config
int windDir = 50;          // Wind direction, input manually based on readings from wind vane
int windSpeed = 5;         // Wind speed in knots. This is used for calculating time spent during tacks in upwind sailing
int noGoZone = 25;         // Specified absolute value of the no-go zone. Determined through experimentation and dependent on boat.
int basicNoGoZone = 25;    // Absolute value of the no-go zone prior to adjustments based on wind speed.
int headingTolerance = 15;        // Tolerated range of headings (+- degrees specified here)

int pointToPointTolerance = 10; //Tolerated radius of acceptance for reaching point-to-point destination. Units are meters. 
int stationKeepingTolerance = 25; //Tolerated radius of acceptance for station keeping. When this range is exceeded, it will sail back to the initial target point. This must be greater than pointToPointTolerance. Units are meters.

int adjustmentAngle = 30;  // Angle to put the rudder at when adjusting heading
int tackingAngle = 45;     // Angle to put the rudder at when completing a tack
int tackLength = 200; //The amount of time the boat sails on a given tack before executing a maneuver
int basicTackLength = 200; // Time spent on a given tack prior to adjustments based on wind speed.

int targetAngle = 0;  // Target direction of travel with reference to the wind direction. Range of [-180:180] with positive values for port tack    
int targetHeading = 0;  // Target heading of the boat.
int sailEaseAmount = 0; //Amount to ease the sail past the initial trims when heeling. 
int heelingThreshold = 20; //Amount of roll that is acceptable before trim adjustments begin to get made.

bool currentlyTacking = false;

enum PointOfSail {  //enum representing the current point of sail
  CLOSE_HAULED,
  TIGHT_CLOSE_REACH,
  CLOSE_REACH,
  BEAM_REACH,
  BROAD_REACH,
  RUNNING,
  NO_GO_ZONE
};


/**
      Sets the target heading and updates target angle as well. 

      @param newTargetHeading the new target heading
    */
void setTargetHeading(int newTargetHeading) {
  targetHeading = newTargetHeading;
  int relativeAngle = targetHeading - windDir;
  if (relativeAngle > 180) {
    targetAngle = relativeAngle - 360;
  } else if (relativeAngle < -180) {
    targetAngle = relativeAngle + 360;
  } else {
    targetAngle = relativeAngle;
  }
}

/**
Sets the new acceptance radius for points during point-to-point navigation;

  @param newTolerance the new tolerance in meters;
*/
void setPointToPointTolerance(int newTolerance) {
  pointToPointTolerance = newTolerance;
}

/**
      Sets the target angle and updates the target heading as well

      @param newTargetAngle the new target angle for heading
    */
void setTargetAngle(int newTargetAngle) {
  targetAngle = newTargetAngle;
  int newTargetHeading = (windDir + targetAngle) % 360;
  if (newTargetHeading < 0) newTargetHeading += 360;
  targetHeading = newTargetHeading;
}


/**
      Sets the adjustment angle

      @param newAdjustmentAngle the new angle for adjusting heading
    */
void setAdjustmentAngle(int newAdjustmentAngle) {
  adjustmentAngle = newAdjustmentAngle;
}

/**
Method for updating the target angle after the wind direction is changed
*/
void updateTargetAngle() {
  int relativeAngle = targetHeading - windDir;
  if (relativeAngle > 180) {
    targetAngle = relativeAngle - 360;
  } else if (relativeAngle < -180) {
    targetAngle = relativeAngle + 360;
  } else {
    targetAngle = relativeAngle;
  }
}

/**
  Sets the tacking angle

  @param newTackingAngle the new angle for completing a tack
*/
void setTackingAngle(int newTackingAngle) {
  tackingAngle = newTackingAngle;
}

/**
  Sets the tack length. ACCOUNTS FOR WINDSPEED
*/
void setTackLength() {
  tackLength = basicTackLength + (200 / windSpeed); //TODO: fine tune the windspeed effect
}

/**
  Sets the basic tack length without accounting for the wind. Then calls setTackLength to make this adjustment. 

  @param newBasicTackLength the new time the boat should be on a given tack. 200 cooresponds to roughly 20 seconds.
*/
void setBasicTackLength(int newBasicTackLength) {
  basicTackLength = newBasicTackLength;
  setTackLength();
}

/**
  Sets the new headingTolerance value

  @param newTolerance the new value for the headingTolerance
*/
void setHeadingTolerance(int newTolerance) {
  headingTolerance = newTolerance;
}

/**
  Sets the new no-go-zone. ACCOUNTS FOR WINDSPEED
*/
void setNoGoZone() {
  noGoZone = basicNoGoZone + (20 / windSpeed); //TODO: fine tune the windspeed effect
}

/**
  Sets the basic no-go-zone without accounting for the wind speed. Then calls setNoGoZone to make this adjustment. 

  @param newBasicNoGoZone the absolute angle of one side of the no-go zone. 
*/
void setBasicNoGoZone(int newBasicNoGoZone) {
  basicNoGoZone = newBasicNoGoZone;
  setNoGoZone();
}


/** Used for setting the servo positions for the trim boundaries. ACCOUNTS FOR WINDSPEED

    @param closeHauled the new servo position for closehauled trim
    @param running the new servo position for running trim
*/
void setTrimConditions(int closeHauled, int running) {
  closeHauledTrim = closeHauled + (windSpeed / 2); //TODO Fine tune the effect of windSpeed
  runningTrim = running;
}

/**
  Sets the new speed of the wind. Updates noGoZone, trim conditions, and tack length to account for the change. 

  @param newSpeed the speed of the wind in knots
*/
void setWindSpeed(int newSpeed) {
  int previousWindSpeed = windSpeed;
  windSpeed = newSpeed;
  setNoGoZone();
  setTackLength();
  setTrimConditions((closeHauledTrim - (previousWindSpeed / 2)), runningTrim);
}

/**
  Sets the new direction of the wind

  @param newDir the new direction of the wind
*/
void setWindDir(int newDir) {
  windDir = newDir;
}

/** Gets the amount the sail should be eased */
int getSailEaseAmount() {
  return sailEaseAmount;
}

/** Increases the amount the sail should be eased */
void increaseSailEase() {
  sailEaseAmount += 10;
}

/** Decreases the amount the sail should be eased */
void decreaseSailEase() {
  if (sailEaseAmount > 0) {
    sailEaseAmount += -10;
  }
}

/** Sets the heeling threshold.
*/
void setHeelingThreshold(int newThreshold) {
  heelingThreshold = newThreshold;
}


/** Configure the pins used for RC communication */
void configureRCPins() {
  pinMode(CH_3_PIN, INPUT);
  pinMode(CH_1_PIN, INPUT);
  pinMode(CH_4_PIN, INPUT);
}

/** Set up for the servos */
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

/** 
  Used to specify the value where the rudder is centered 

  @param val the new servo position representing a centered rudder
*/
void setCenteredRudder(int val) {
  centeredRudder = val;
}

/** Helper function to print the information when a callibration is completed. */
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
#endif // BOAT_CONFIG_H