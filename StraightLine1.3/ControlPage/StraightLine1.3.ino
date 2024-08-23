/**********************
This program is designed to facilitate the autonomous navigation of a model sailboat. 
It uses an ESP32-WROOM-32E for a microcontroller, a MPU-9250 as a 9-axis IMU for obtaining heading information,
and servo motors to control the rudder and trim. The sensor data is hosted on a local server using the ESP32, 
and a visualization of the relevant information can be found by navigating to the IP-address of the ESP32. 

This project was completed by Charlie Apolinsky for use in Pranav Kedia's research on swarm robotics at the 
University of Konstanz. 

Date: June 18, 2024
Bugs: None known
Author: Charlie Apolinsky
**********************/
#include <Arduino.h>
#include "MPU9250.h"
#include <ESP32Servo.h>
#include <eeprom_utils.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <boat_config.h>
#include <location.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

#define I2C_SDA 33
#define I2C_SCL 32

MPU9250 mpu;
// Create a TinyGPS++ object
TinyGPSPlus gps;

// Set up hardware serial
HardwareSerial mySerial(1);  // Using UART1 on ESP32

const char *ssid = "ESP32-Access-Point";
const char *password = "123456789";

//Vars for type of control
bool manualControl = true;
bool pointToPoint = false;
bool autonomousMode = false;
bool stationKeeping = false;

int rudderPos = centeredRudder;  // variable to store the servo position. 135 represents centered rudder
int trimPos = closeHauledTrim;   //The current trim of the sail

float roll;
float yaw;
float heading;          // The heading of the boat, given in 360 degrees found through the IMU
float gpsHeading;       //The heading found using the gps library
int trueTargetHeading;  //The current heading the boat is targeting
int headingAdjustment = 270;

double distanceToDestination;  //distance to the next target for point to point
double courseToDestination;    //heading to the next target for point to point

float currentSpeed;      //The current speed of the boat in m/s found through GPS.
float averageSpeed = 3;  //Calculated using a simple exponential moving average.
float alpha = 0.2;       //parameter for determining how responsive the moving average for the speed is.

int counter = 1;  //Counter used for autonomous sailing

bool onStarboard;  // Boolean representing the tack that the boat is on
char *curAction;

int luffing[4];    // Too close to the wind direction
int offCourse[4];  // Veering too far away from wind direction

PointOfSail pointOfSail;  //enum representing the current point of sail

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(2000);

  initServos();
  configureRCPins();
  setCenteredRudder(115);
  setWindDir(0);
  setWindSpeed(5);
  setTrimConditions(30, 140);
  setBasicNoGoZone(20);
  setHeadingTolerance(8);
  setPointToPointTolerance(10);
  setTackingAngle(45);
  setBasicTackLength(200);
  setAdjustmentAngle(30);
  setTargetHeading(290);
  setHeelingThreshold(20);
  trueTargetHeading = targetHeading;

  //Init ESP32
  setUpMPU();

  //Init communication with GPS
  mySerial.begin(38400, SERIAL_8N1, 16, 17);  // baud rate, configuration, RX pin, TX pin

  // Setting the ESP as an access point
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  //Return all the information about the boat.
  server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", formatInfo());
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });

  // Send a POST request to <IP>/coords with form fields for the new coordinates
  server.on("/coords", HTTP_POST, [](AsyncWebServerRequest *request) {
    String message;

    if (request->hasParam("pointB", true)) {
      String newCoords = request->getParam("pointB", true)->value();
      int commaIndexB = newCoords.indexOf(',');
      double newCoordLat = newCoords.substring(0, commaIndexB).toDouble();
      double newCoordLng = newCoords.substring(commaIndexB + 1).toDouble();
      addCoordinate(newCoordLat, newCoordLng);
      setMode("PTP");
      message = "(" + newCoords + ") was added to the targets list.\n Current target is: (" + getCurrentTarget().lat + "," + getCurrentTarget().lng + ").";
    } else {
      message = "No coordinates were recieved.";
    }
    //Get and set the course to the target destination
    courseToDestination = TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      getCurrentTarget().lat,
      getCurrentTarget().lng);
    setTargetHeading(courseToDestination);

    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", message);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });

  //Update no go zone or tack length
  server.on("/tacking-parameters", HTTP_POST, [](AsyncWebServerRequest *request) {
    String message = "";
    bool updated = false;

    if (request->hasParam("noGoZone", true)) {
      int newNoGoZone = request->getParam("noGoZone", true)->value().toInt();
      int previousNoGoZone = noGoZone;
      setBasicNoGoZone(newNoGoZone);
      message = "No-go-zone was changed from " + String(previousNoGoZone) + " to " + String(noGoZone) + ".\n";
      bool updated = true;
    }

    if (request->hasParam("tackLength", true)) {
      int newTackLength = request->getParam("tackLength", true)->value().toInt();
      int previousTackLength = tackLength;
      setBasicTackLength(newTackLength);
      message += "tackLength was changed from " + String(previousTackLength) + " to " + String(tackLength) + ".\n";
      bool updated = true;
    }


    if (!updated) {
      message = "Nothing was updated.";
    }


    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", message);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });


  // Send a POST request to <IP>/post with form fields newWindDir and newTargetAngle or for switching control
  server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request) {
    String message = "";
    bool updated = false;

    if (request->hasParam("newWindDir", true)) {
      // Retrieve the new wind direction parameter
      int newWindDir = request->getParam("newWindDir", true)->value().toInt();
      setWindDir(newWindDir);
      updateTargetAngle();
      message += "New wind direction set to: " + String(newWindDir) + ".\n";
      updated = true;
    }

    if (request->hasParam("newWindSpeed", true)) {
      int newWindSpeed = request->getParam("newWindSpeed", true)->value().toInt();
      message += "Wind speed was changed from " + String(windSpeed) + " to " + String(newWindSpeed) + ".\n";
      setWindSpeed(newWindSpeed);
      updated = true;
    }


    if (request->hasParam("newTargetHeading", true)) {
      int newTargetHeading = request->getParam("newTargetHeading", true)->value().toInt();
      setTargetHeading(newTargetHeading);
      setMode("A");
      message += "New target heading set to: " + String(newTargetHeading);
      updated = true;
    } else if (request->hasParam("newTargetAngle", true)) {
      int newTargetAngle = request->getParam("newTargetAngle", true)->value().toInt();
      setTargetAngle(newTargetAngle);
      setMode("A");
      message = "New target angle set to: " + String(newTargetAngle);
      updated = true;
    }

    if (request->hasParam("headingAdjustment", true)) {
      int newHeadingAdjustment = request->getParam("headingAdjustment", true)->value().toInt();
      headingAdjustment = newHeadingAdjustment;
      message += "New heading adjustment: " + String(headingAdjustment) + "\n";
      updated = true;
    }

    if (request->hasParam("switchControl", true)) {
      updated = true;
      if (manualControl) {
        message = "Method of control successfully switched to autonomous";
        setMode("A");
      } else {
        message = "Method of control successfully switched to remote control";
        setMode("RC");
      }
    } else if (request->hasParam("stationKeeping", true)) {
      updated = true;
      if (stationKeeping) {
        setMode("RC");
        message = "Terminating Station Keeping. Now in RC mode";
      } else {
        setCurrentTarget(getCurrentPosition().lat, getCurrentPosition().lng);
        setMode("SK");
        message = "Beginning Station Keeping";
      }
    } else if (request->hasParam("pointToPoint", true)) {
      updated = true;
      if (pointToPoint) {
        setMode("RC");
        message = "Terminating Point to Point. Now in RC mode";
      } else {
        setMode("PTP");
        message = "Resuming Station Keeping";
      }
    }

    if (request->hasParam("clearTargetCoordinates", true)) {
      updated = true; 
      message = "Targets list was cleared";
      clearCoordinatesList();
    }
 
    if (!updated) {
      message = "Nothing was updated or changed.";
    }

    handleTrim();
    checkForValidPath();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", message);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });

  // Handle CORS preflight requests
  server.on("/post", HTTP_OPTIONS, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(204);
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    response->addHeader("Access-Control-Allow-Headers", "Content-Type");
    request->send(response);
  });

  // Start server
  server.begin();
}

//MAIN LOGIC OF THE PROGRAM
void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 100) {
      //UPDATE MPU
      roll = mpu.getPitch();
      yaw = mpu.getYaw();
      // Adjust yaw to get real world heading.
      heading = yaw + headingAdjustment;
      // Normalize heading to be within 0 to 360 degrees
      if (heading >= 360.0) {
        heading -= 360.0;
      } else if (heading < 0.0) {
        heading += 360.0;
      }

      //PROCESS GPS INFORMATION
      processGPS();

      if (!manualControl) {
        printInfo();
        if (pointToPoint) {  //POINT TO POINT MODE
          atDestination();
        } else if (stationKeeping) {  //STATION KEEPING MODE
          if (distanceToDestination >= stationKeepingTolerance) {
            addCoordinate(getCurrentTarget().lat, getCurrentTarget().lng);
            setMode("PTP");
          } else {
            sailInLoop();
          }
        }  //SAIL AUTONOMOUSLY
        handleTrim();
        checkForValidPath();
        handleSteering();
        rudderServo.write(rudderPos);
        printInfo();
        counter += 1;
      } else {
        processRC();
      }
      switchManualControl();
      prev_ms = millis();
    }
  }
}

/*Updates the location, speed, point-to-point target heading, and point-to-point distance*/
void processGPS() {
  while (mySerial.available() > 0) {
    char c = mySerial.read();
    gps.encode(c);
  }
  distanceToDestination =
    TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      getCurrentTarget().lat,
      getCurrentTarget().lng);
  courseToDestination =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      getCurrentTarget().lat,
      getCurrentTarget().lng);
  currentSpeed = gps.speed.mps();
  averageSpeed = (currentSpeed * alpha) + (averageSpeed * (1 - alpha));
  gpsHeading = gps.course.deg();

  setCurrentPosition(gps.location.lat(), gps.location.lng());
}

/*Checks to see if the boat is at the next target coordinate. If this is the last target, it will enter station keeping mode. If it is not at the target, it sets the proper heading to reach it. */
void atDestination() {
  if (distanceToDestination <= pointToPointTolerance) {
    if (updateCoordinatesList()) {
      Serial.println("FINAL target reached");
      setMode("SK");
      return;
    }
    Serial.println("Target reached");
  }
  setTargetHeading(courseToDestination);
  return;
}

/* Has the boat sail in an infinity circle */
void sailInLoop() {
  int loopLength = 200 + (200 / windSpeed);  // Total length of the loop (units are roughly 1/10 seconds)
  int loopCount = counter % loopLength;
  if (loopCount < (.3 * loopLength)) {
    setTargetAngle(50);
  } else if (loopCount < (.35 * loopLength)) {
    setTargetAngle(90);
  } else if (loopCount < (.4 * loopLength)) {
    setTargetAngle(160);
  } else if (loopCount < (.45 * loopLength)) {
    setTargetAngle(-160);
  } else if (loopCount < (.5 * loopLength)) {
    setTargetAngle(-90);
  } else if (loopCount < (.8 * loopLength)) {
    setTargetAngle(-50);
  } else if (loopCount < (.85 * loopLength)) {
    setTargetAngle(-90);
  } else if (loopCount < (.9 * loopLength)) {
    setTargetAngle(-160);
  } else if (loopCount < (.95 * loopLength)) {
    setTargetAngle(160);
  } else if (loopCount < (loopLength)) {
    setTargetAngle(90);
  }
}

/* Checks if the target path is in the no-go zone. If so, transforms into segments of close-hauled sailing */
void checkForValidPath() {
  if (abs(targetAngle) <= noGoZone) {
    // Default time on each tack is about 20 seconds (ratio is adjusted depending on desired angle).
    float headingRatio = targetAngle / noGoZone;
    int timeOnPort = tackLength + (tackLength * headingRatio);
    int timeOnStarboard = tackLength - (tackLength * headingRatio);
    // Serial.println(counter);
    if ((counter % (2 * tackLength)) == timeOnStarboard || (counter % (2 * tackLength)) == 0) {
      if ((pointOfSail == CLOSE_HAULED) || (pointOfSail == TIGHT_CLOSE_REACH)) {
        executeTack();
      } else {
        counter = counter - 1;
      }
    } else if ((counter % (2 * tackLength)) > timeOnStarboard) {
      set_heading_tolerance(noGoZone + headingTolerance, headingTolerance);
    } else {
      set_heading_tolerance(-1 * (noGoZone + headingTolerance), headingTolerance);
    }
  } else {
    set_heading_tolerance(targetAngle, headingTolerance);
  }
}

/* Function for processing the current heading and trimming the sail to maximize speed */
void handleTrim() {
  int relativeHeading = fmod(heading - windDir + 360.0, 360.0);
  if (relativeHeading > 180) {
    relativeHeading -= 360;
  }

  pointOfSail = determinePointOfSail(relativeHeading);  //Assign pointOfSail
  if (abs(roll) > heelingThreshold) {                   //If the boat is heeling too much, ease the sail
    increaseSailEase();
  } else {
    decreaseSailEase();
  }
  setSailTrim();
  trimServo.write(trimPos);  //Trim the sail to the appropriate angle to maximize speed.
}

/** Function for processing the current heading and adjusting the rudder to reach the target angle 
  @return returns true if the boat is sailing on course.
*/
bool handleSteering() {
  //ON STARBOARD TACK
  if (onStarboard) {
    if ((heading > luffing[0] && heading < luffing[1]) || (heading > luffing[2] && heading < luffing[3])) {  //SAILING TOO HIGH. NEED TO BEAR OFF
      Serial.println("I'm luffing, bearing off now!");
      curAction = "Bearing Off";
      rudderPos = centeredRudder + adjustmentAngle;
      return false;
    } else if ((heading > offCourse[0] && heading < offCourse[1]) || (heading > offCourse[2] && heading < offCourse[3])) {  //SAILING OFF COURSE. NEED TO HEAD UP
      Serial.println("I'm sailing too deep, heading up now!");
      curAction = "Heading Up";
      rudderPos = centeredRudder - adjustmentAngle;
      return false;
    } else if (rudderPos != centeredRudder) {
      rudderPos = centeredRudder;
      curAction = "Sailing Straight";
      return true;
    }
  }
  // ON PORT TACK
  else {
    if ((heading > luffing[0] && heading < luffing[1]) || (heading > luffing[2] && heading < luffing[3])) {  //SAILING TOO HIGH. NEED TO BEAR OFF
      Serial.println("I'm luffing, bearing off now!");
      curAction = "Bearing Off";
      rudderPos = centeredRudder - adjustmentAngle;
      return false;
    } else if ((heading > offCourse[0] && heading < offCourse[1]) || (heading > offCourse[2] && heading < offCourse[3])) {  //SAILING OFF COURSE. NEED TO HEAD UP
      Serial.println("I'm sailing too deep, heading up now!");
      curAction = "Heading Up";
      rudderPos = centeredRudder + adjustmentAngle;
      return false;
    } else if (rudderPos != centeredRudder) {
      rudderPos = centeredRudder;
      curAction = "Straight";
      return true;
    }
  }
}

/* For when the user wants to use remote control */
void processRC() {
  ch_3 = pulseIn(CH_3_PIN, HIGH, 25000);
  ch_1 = pulseIn(CH_1_PIN, HIGH, 25000);
  delay(20);
  rudderPos = map(ch_3, 1240, 1840, centeredRudder - tackingAngle, centeredRudder + tackingAngle);
  trimPos = map(ch_1, 915, 2100, closeHauledTrim, runningTrim);
  rudderServo.write(rudderPos);
  trimServo.write(trimPos);
}

/* Detects if the user wants to switch to manual control */
void switchManualControl() {
  ch_4 = pulseIn(CH_4_PIN, HIGH, 25000);
  if (ch_4 > 1750) {
    for (int i = 0; i < 50; i++) {
      ch_4 = pulseIn(CH_4_PIN, HIGH, 25000);
      if (ch_4 < 1750) {
        return;
      }
      delay(20);
    }
    if (manualControl) {
      setMode("A");
    } else {
      setMode("RC");
    }
  }
}


/* Helper function to determine the current point of sail. No go zone is set as default to be 45 degrees */
PointOfSail determinePointOfSail(int angle) {
  int absAngle = abs(angle);
  int pointOfSailWidth = (180 - noGoZone) / 5;
  if (absAngle <= noGoZone) {
    return NO_GO_ZONE;
  } else if (absAngle <= noGoZone + pointOfSailWidth) {
    return CLOSE_HAULED;
  } else if (absAngle <= noGoZone + 1.5 * pointOfSailWidth) {
    return TIGHT_CLOSE_REACH;
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

/*
Helper function to take the current point of sail and set the appropriate trim angle
*/
void setSailTrim() {
  PointOfSail pos = pointOfSail;
  int easeAdjustment = getSailEaseAmount();
  int sailTrimIncrement = (runningTrim - closeHauledTrim) / 4;
  switch (pointOfSail) {
    case CLOSE_HAULED:
      trimPos = closeHauledTrim + easeAdjustment;
      return;
    case NO_GO_ZONE:
      trimPos = closeHauledTrim + easeAdjustment;
      return;
    case TIGHT_CLOSE_REACH:
      trimPos = closeHauledTrim + easeAdjustment;
      return;
    case CLOSE_REACH:
      trimPos = closeHauledTrim + sailTrimIncrement + easeAdjustment;
      return;
    case BEAM_REACH:
      trimPos = closeHauledTrim + 2 * sailTrimIncrement + easeAdjustment;
      return;
    case BROAD_REACH:
      trimPos = closeHauledTrim + 3 * sailTrimIncrement + easeAdjustment;
      return;
    case RUNNING:
      trimPos = runningTrim + easeAdjustment;
      return;
    default:
      trimPos = closeHauledTrim + easeAdjustment;
      return;
  }
}

/*
Helper function to determine what the range of acceptable headings are for a given course and tolerance. 
*/
void set_heading_tolerance(int targetAngle, int headingTolerance) {
  int luffingBoundary;
  int offCourseBoundary;
  trueTargetHeading = (windDir + targetAngle) % 360;
  if (trueTargetHeading < 0) trueTargetHeading += 360;
  if (targetAngle < 0) {  // ON STARBOARD TACK
    onStarboard = true;
    offCourseBoundary = (trueTargetHeading - headingTolerance) % 360;
    if (offCourseBoundary < 0) offCourseBoundary += 360;
    luffingBoundary = (trueTargetHeading + headingTolerance) % 360;

    if (trueTargetHeading > 360 - headingTolerance) {  //NW target heading, NE luffing boundary and wind
      luffing[0] = luffingBoundary;
      luffing[1] = windDir;
      luffing[2] = INT_MAX;
      luffing[3] = INT_MIN;
      offCourse[0] = windDir;
      offCourse[1] = offCourseBoundary;
      offCourse[2] = INT_MAX;
      offCourse[3] = INT_MIN;
    } else {
      if (luffingBoundary > windDir) {  //NW target heading and luffing boundary, NE wind
        luffing[0] = luffingBoundary;
        luffing[1] = 361;
        luffing[2] = 0;
        luffing[3] = windDir;
        offCourse[0] = windDir;
        offCourse[1] = offCourseBoundary;
        offCourse[2] = INT_MAX;
        offCourse[3] = INT_MIN;
      } else {  //All NE
        luffing[0] = luffingBoundary;
        luffing[1] = windDir;
        luffing[2] = INT_MAX;
        luffing[3] = INT_MIN;
        if (offCourseBoundary > windDir) {  //NW offCourse
          offCourse[0] = windDir;
          offCourse[1] = offCourseBoundary;
          offCourse[2] = INT_MAX;
          offCourse[3] = INT_MIN;
        } else {  // NE offCourse
          offCourse[0] = 0;
          offCourse[1] = offCourseBoundary;
          offCourse[2] = windDir;
          offCourse[3] = 361;
        }
      }
    }
  } else {  // ON PORT TACK
    onStarboard = false;
    offCourseBoundary = (trueTargetHeading + headingTolerance) % 360;
    luffingBoundary = (trueTargetHeading - headingTolerance) % 360;
    if (luffingBoundary < 0) luffingBoundary += 360;
    if (trueTargetHeading < headingTolerance) {  //NE targetHeading, NW wind and luffing boundary
      luffing[0] = windDir;
      luffing[1] = luffingBoundary;
      luffing[2] = INT_MAX;
      luffing[3] = INT_MIN;
      offCourse[0] = offCourseBoundary;
      offCourse[1] = windDir;
      offCourse[2] = INT_MAX;
      offCourse[3] = INT_MIN;
    } else {
      if (windDir > luffingBoundary) {  // NE targetHeading and luffing boundary, NW wind
        luffing[0] = windDir;
        luffing[1] = 361;
        luffing[2] = 0;
        luffing[3] = luffingBoundary;
        offCourse[0] = offCourseBoundary;
        offCourse[1] = windDir;
        offCourse[2] = INT_MAX;
        offCourse[3] = INT_MIN;
      } else {  // NE targetHeading, luffing boundary, and wind
        luffing[0] = windDir;
        luffing[1] = luffingBoundary;
        luffing[2] = INT_MAX;
        luffing[3] = INT_MIN;
        if (offCourseBoundary < trueTargetHeading) {
          offCourse[0] = offCourseBoundary;
          offCourse[1] = windDir;
          offCourse[2] = INT_MAX;
          offCourse[3] = INT_MIN;
        } else {
          offCourse[0] = offCourseBoundary;
          offCourse[1] = 361;
          offCourse[2] = 0;
          offCourse[3] = windDir;
        }
      }
    }
  }
}

/* 
Helper function to execute a tack. Assumes that the boat has sufficient speed to make it through the no-go zone.
*/
void executeTack() {
  if (onStarboard) {
    rudderPos = centeredRudder - tackingAngle;
    set_heading_tolerance(noGoZone + headingTolerance, headingTolerance);
    rudderServo.write(rudderPos);
  } else {
    //Put the rudder all the way to port, perhaps trim sail slightly as this is done.
    rudderPos = centeredRudder + tackingAngle;
    set_heading_tolerance(-1 * (noGoZone + headingTolerance), headingTolerance);
    rudderServo.write(rudderPos);
  }
  while (!handleSteering()) {
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 100) {
        handleTrim();
        yaw = mpu.getYaw();
        // Adjust yaw by adding 270 degrees
        heading = yaw + 270.0;
        // Normalize heading to be within 0 to 360 degrees
        if (heading >= 360.0) {
          heading -= 360.0;
        } else if (heading < 0.0) {
          heading += 360.0;
        }
        Serial.print("Executing tack. ");
        curAction = "Tacking";
        Serial.println(heading);
      }
    }
  }
}


/**
Method used for updating the current mode of sailing.
*/
void setMode(char *mode) {
  if (mode == "A") {
    autonomousMode = true;
    manualControl = false;
    stationKeeping = false;
    pointToPoint = false;
  } else if (mode == "RC") {
    autonomousMode = false;
    manualControl = true;
    stationKeeping = false;
    pointToPoint = false;
  } else if (mode == "SK") {
    autonomousMode = false;
    manualControl = false;
    stationKeeping = true;
    pointToPoint = false;
  } else if (mode == "PTP") {
    autonomousMode = false;
    manualControl = false;
    stationKeeping = false;
    pointToPoint = true;
  } else {  //base case just set to remote control
    autonomousMode = false;
    manualControl = true;
    stationKeeping = false;
    pointToPoint = false;
  }
}




/*Helper function to take the point of sail and get a readable string */
const char *pointOfSailToString(PointOfSail pos) {
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


/** Function used to initialize the MPU and load calibration data. Portion of code can be uncommented if user wants to recalibrate the MPU*/
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
Helper function to print the current information about the boat.
*/
void printInfo() {
  String info;
  info += "roll: ";
  info += String(roll, 2);
  info += "Wind: ";
  info += windDir;
  info += "  Target heading: ";
  info += targetHeading;
  info += " Heading: ";
  info += String(heading, 2);
  info += " Trim pos: ";
  info += trimPos;
  info += " Luffing: [";
  for (int i = 0; i < 4; i++) {
    info += luffing[i];
    if (i < 3) info += ", ";
  }
  info += "]\t Offcourse: [";
  for (int i = 0; i < 4; i++) {
    info += offCourse[i];
    if (i < 3) info += ", ";
  }
  info += "] Current Tack: ";
  if (onStarboard) {
    info += "(S), ";
  } else {
    info += "(P), ";
  }
  info += pointOfSailToString(pointOfSail);
  info += "Lat, Lng: (";
  info += String(getCurrentPosition().lat, 6) + "," + String(getCurrentPosition().lng, 6) + ")";
  Serial.println(info);
}

/* Helper function to get a string with all the information about the boad */
String formatInfo() {
  DynamicJsonDocument doc(1024);

  doc["lat"] = String(getCurrentPosition().lat, 7);
  doc["lng"] = String(getCurrentPosition().lng, 7);

  doc["windDir"] = windDir;
  doc["windSpeed"] = windSpeed;

  doc["targetAngle"] = targetAngle;
  doc["targetHeading"] = trueTargetHeading;

  doc["coordinatesList"] = getCoordinatesList();
  doc["currentTarget"] = "(" + String(getCurrentTarget().lat, 6) + "," + String(getCurrentTarget().lng, 6) + ")";

  doc["distanceToDestination"] = String(distanceToDestination / 1000, 6);
  doc["GPSCourseToDestination"] = String(courseToDestination, 6);
  doc["rudderPos"] = rudderPos;
  doc["trimPos"] = trimPos;
  doc["heading"] = String(heading, 2);
  doc["currentTack"] = onStarboard ? "S" : "P";
  doc["currentSpeed"] = String(currentSpeed, 2);
  doc["avgSpeed"] = String(averageSpeed, 2);
  doc["pointOfSail"] = pointOfSailToString(pointOfSail);
  if (manualControl) {
    doc["mode"] = "Remote Control";
    doc["curAction"] = "N/A";
    doc["pointOfSail"] = "N/A";
    doc["currentTack"] = "N/A";
    doc["targetHeading"] = "N/A";
  } else if (pointToPoint) {
    doc["mode"] = "Point to Point";
    doc["curAction"] = "N/A";
  } else if (stationKeeping) {
    doc["mode"] = "Station Keeping";
    doc["curAction"] = "N/A";
  } else {
    doc["mode"] = "Autonomous";
    doc["curAction"] = curAction;
  }
  String info;
  serializeJson(doc, info);

  return info;
}
