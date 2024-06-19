#include "MPU9250.h"
#include <ESP32Servo.h>
#include <eeprom_utils.h>


MPU9250 mpu;

Servo rudderServo;
int rudderPos = 90;        // variable to store the servo position. 90 represents centered rudder, 0 represents all the way to port, 180 all the way to starboard.
const int rudderPin = 18;  // attach rudder servo to pin 18

Servo trimServo;
int trimPos = 90;        // variable to store the servo position. 90 represents centered rudder, 0 represents all the way to port, 180 all the way to starboard.
const int trimPin = 17;  // attach rudder servo to pin 18

const int adjustmentAngle = 20;  // Angle to put the rudder at when adjusting heading
const int tackingAngle = 45;     // Angle to put the rudder at when completing a tack

float pitch;
float roll;
float yaw;
float heading;  // The heading of the boat, given in 360 degrees
int counter = 1;

int windDir = 50;         // Wind direction, input manually based on readings from wind vane
const int noGoZone = 45;  // Specified absolute value of the no-go zone. Determined through experimentation and dependent on boat.
bool onStarboard;         // Boolean representing the tack that the boat is on
int targetAngle = -60;    // Target direction of travel with reference to the wind direction. Range of [-180:180] with positive values for port tack, negative for starboard.
int tolerance = 15;       // Tolerated range of headings (+- degrees specified here)

int luffing[4];    // Too close to the wind direction
int offCourse[4];  // Veering too far away from wind direction

enum PointOfSail {
  CLOSE_HAULED,
  CLOSE_REACH,
  BEAM_REACH,
  BROAD_REACH,
  RUNNING,
  NO_GO_ZONE
};

PointOfSail pointOfSail;  //enum representing the current point of sail




void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  //Init servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  rudderServo.setPeriodHertz(50);  // standard 50 hz servo
  rudderServo.attach(rudderPin, 500, 2300);
  rudderServo.write(90);
  trimServo.setPeriodHertz(50);  // standard 50 hz servo
  trimServo.attach(trimPin, 500, 2300);
  trimServo.write(90);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  // Set the magnetic declination (Konstanz, Germany)
  mpu.setMagneticDeclination(3.1);
  //Set quaternion filter to 3;
  // mpu.setFilterIterations(3);


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

void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 100) {
      pitch = mpu.getPitch();
      roll = mpu.getRoll();
      yaw = mpu.getYaw();

      // Adjust yaw by adding 270 degrees
      heading = yaw + 270.0;

      // Normalize heading to be within 0 to 360 degrees
      if (heading >= 360.0) {
        heading -= 360.0;
      } else if (heading < 0.0) {
        heading += 360.0;
      }

      // Calculate the angle relative to the wind direction
      int relativeHeading = fmod(heading - windDir + 360.0, 360.0);
      if (relativeHeading > 180) {
        relativeHeading -= 360;
      }

      pointOfSail = determinePointOfSail(relativeHeading);  //Assign pointOfSail
      setSailTrim();
      trimServo.write(trimPos);

      //Check to see if the attempted path is in the no-go zone.
      if (pointOfSail == NO_GO_ZONE) {
        /*Transform the target heading into segments of close-hauled sailing 
        (sailing as close as possible to the wind while still maintaining a fast speed). 
        Default time on each tack is 30 seconds (ratio is adjusted depending on desired angle).*/
        float headingRatio = targetAngle / 45.0;
        int timeOnPort = 300 + (300 * headingRatio);
        int timeOnStarboard = 300 - (300 * headingRatio);

        if ((counter % 600) == timeOnStarboard || (counter % 600) == 0) {
          executeTack();
        } else if ((counter % 600) > timeOnStarboard) {
          set_heading_tolerance(noGoZone + tolerance, tolerance);
        } else {
          set_heading_tolerance(-1 * (noGoZone + tolerance), tolerance);
        }
      } else {
        set_heading_tolerance(targetAngle, tolerance);
      }

      //ON STARBOARD TACK
      if (onStarboard) {
        if ((heading > luffing[0] && heading < luffing[1]) || (heading > luffing[2] && heading < luffing[3])) {  //SAILING TOO HIGH. NEED TO BEAR OFF
          Serial.println("I'm luffing, bearing off now!");
          rudderPos = 90 - adjustmentAngle;
          rudderServo.write(rudderPos);
        } else if ((heading > offCourse[0] && heading < offCourse[1]) || (heading > offCourse[2] && heading < offCourse[3])) {  //SAILING OFF COURSE. NEED TO HEAD UP
          Serial.println("I'm sailing too deep, heading up now!");
          rudderPos = 90 + adjustmentAngle;
          rudderServo.write(rudderPos);
        } else if (rudderPos != 90) {
          rudderPos = 90;
          rudderServo.write(rudderPos);
        }
      }
      // ON PORT TACK
      else {
        if ((heading > luffing[0] && heading < luffing[1]) || (heading > luffing[2] && heading < luffing[3])) {  //SAILING TOO HIGH. NEED TO BEAR OFF
          Serial.println("I'm luffing, bearing off now!");
          rudderPos = 90 + adjustmentAngle;
          rudderServo.write(rudderPos);
        } else if ((heading > offCourse[0] && heading < offCourse[1]) || (heading > offCourse[2] && heading < offCourse[3])) {  //SAILING OFF COURSE. NEED TO HEAD UP
          Serial.println("I'm sailing too deep, heading up now!");
          rudderPos = 90 - adjustmentAngle;
          rudderServo.write(rudderPos);
        } else if (rudderPos != 90) {
          rudderPos = 90;
          rudderServo.write(rudderPos);
        }
      }


      print_info();
      counter += 1;
      prev_ms = millis();
    }
  }
}

/*
Helper function to determine the current point of sail. No go zone is set to be 45 deg
*/
PointOfSail determinePointOfSail(int angle) {
  int absAngle = abs(angle);
  int pointOfSailWidth = (180 - noGoZone) / 5;
  if (absAngle <= noGoZone) {
    return NO_GO_ZONE;
  } else if (absAngle <= noGoZone + pointOfSailWidth) {
    return CLOSE_HAULED;
  } else if (absAngle <= noGoZone + 2 * pointOfSailWidth) {
    return CLOSE_REACH;
  } else if (absAngle <= noGoZone + 3 * pointOfSailWidth) {
    return BEAM_REACH;
  } else if (absAngle <= noGoZone + 4 * pointOfSailWidth) {
    return BROAD_REACH;
  } else {
    return RUNNING;
  }
}

void setSailTrim() {
  PointOfSail pos = pointOfSail;
  int closeHauledTrim = 0;  //Assuming that for the RC boat servo, the closehauled trim is when the servo is set to 0.
  int runningTrim = 180;    //Assuming that for the RC boat servo, the running trim is when the servo is set to 180.
  int sailTrimIncrement = (runningTrim - closeHauledTrim) / 4;
  switch (pointOfSail) {
    case CLOSE_HAULED:
      trimPos = closeHauledTrim;
      return;
    case NO_GO_ZONE:
      trimPos = closeHauledTrim;
      return;
    case CLOSE_REACH:
      trimPos = sailTrimIncrement;
      return;
    case BEAM_REACH:
      trimPos = 2 * sailTrimIncrement;
      return;
    case BROAD_REACH:
      trimPos = 3 * sailTrimIncrement;
      return;
    case RUNNING:
      trimPos = runningTrim;
      return;
    default:
      trimPos = closeHauledTrim;
      return;
  }
}

/*Helper function to take the point of sail and get a readable string */
const char* pointOfSailToString(PointOfSail pos) {
  switch (pos) {
    case CLOSE_HAULED:
      return "Close Hauled";
    case CLOSE_REACH:
      return "Close Reach";
    case BEAM_REACH:
      return "Beam Reach";
    case BROAD_REACH:
      return "Broad Reach";
    case RUNNING:
      return "Running";
    case NO_GO_ZONE:
      return "No Go Zone";
    default:
      return "Unknown";
  }
}

/*
Helper function to determine what the range of acceptable headings are. 
*/
void set_heading_tolerance(int targetAngle, int tolerance) {
  int targetHeading = (windDir + targetAngle) % 360;
  if (targetHeading < 0) targetHeading += 360;

  int luffingBoundary;
  int offCourseBoundary;

  if (targetAngle < 0) {  // ON STARBOARD TACK
    onStarboard = true;
    offCourseBoundary = (targetHeading - tolerance) % 360;
    if (offCourseBoundary < 0) offCourseBoundary += 360;

    luffingBoundary = (targetHeading + tolerance) % 360;
    if (targetHeading + tolerance > 360) {  // Case for NE wind and NW targetHeading
      luffing[0] = luffingBoundary;
      luffing[1] = windDir;
      luffing[2] = INT_MAX;
      luffing[3] = INT_MIN;

      offCourse[0] = windDir;
      offCourse[1] = offCourseBoundary;
      offCourse[2] = INT_MAX;
      offCourse[3] = INT_MIN;
    } else {  // NW wind and targetHeading
      luffing[0] = luffingBoundary;
      luffing[1] = 361;
      luffing[2] = 0;
      luffing[3] = windDir;

      offCourse[0] = windDir;
      offCourse[1] = offCourseBoundary;
      offCourse[2] = INT_MAX;
      offCourse[3] = INT_MIN;
    }
  } else {  // ON PORT TACK
    onStarboard = false;
    luffingBoundary = (targetHeading - tolerance) % 360;
    if (luffingBoundary < 0) luffingBoundary += 360;

    if (windDir > luffingBoundary) {  // NW wind and NE targetHeading
      luffing[0] = windDir;
      luffing[1] = 361;
      luffing[2] = 0;
      luffing[3] = luffingBoundary;

      offCourse[0] = (targetHeading + tolerance) % 360;
      offCourse[1] = windDir;
      offCourse[2] = INT_MAX;
      offCourse[3] = INT_MIN;
    } else {  // NE wind and targetHeading
      luffing[0] = windDir;
      luffing[1] = luffingBoundary;
      luffing[2] = INT_MAX;
      luffing[3] = INT_MIN;

      offCourse[0] = (targetHeading + tolerance) % 360;
      offCourse[1] = INT_MAX;
      offCourse[2] = 0;
      offCourse[3] = windDir;
    }
  }
}

/* 
Helper function to execute a tack. Assumes that the boat has sufficient speed to make it through the no-go zone.
*/
void executeTack() {
  //Length of tack in seconds
  int tackDelay = 10;
  int tackDelayCounter = 0;
  if (onStarboard) {
    rudderPos = 90 + tackingAngle;
    rudderServo.write(rudderPos);
    Serial.println("Tacking from starboard to port!");
    //Working under the assumption that the tack takes about 10 seconds to complete. Ultimately, this will be dependent on wind and boat speed.
  } else {
    //Put the rudder all the way to port, perhaps trim sail slightly as this is done.
    rudderPos = 90 - tackingAngle;
    rudderServo.write(rudderPos);
    Serial.println("Tacking from port to starboard!");
    //Working under the assumption that the tack takes about 10 seconds to complete. Ultimately, this will be dependent on wind and boat speed.
  }
  while (tackDelayCounter < tackDelay * 10) {
    tackDelayCounter += 1;
    mpu.update();
    pitch = mpu.getPitch();
    roll = mpu.getRoll();
    yaw = mpu.getYaw();
    delay(100);
  }
}

/*
Helper function to print the current information about the boat.
*/
void print_info() {
  Serial.print("Wind: ");
  Serial.print(windDir);
  Serial.print("  Target angle:");
  Serial.print(targetAngle);
  Serial.print(" Heading: ");
  Serial.print(heading, 2);
  Serial.print("\t Luffing: [");
  for (int i = 0; i < 4; i++) {
    Serial.print(luffing[i]);
    if (i < 3) Serial.print(", ");
  }
  Serial.print("]\t Offcourse: [");
  for (int i = 0; i < 4; i++) {
    Serial.print(offCourse[i]);
    if (i < 3) Serial.print(", ");
  }
  Serial.print("]\t Current Tack: ");
  if (onStarboard) {
    Serial.print("Starboard (S), ");
  } else {
    Serial.print("Port (P), ");
  }
  Serial.println(pointOfSailToString(pointOfSail));
}


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
