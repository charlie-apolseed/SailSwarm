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
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>
#include <boat_config.h>

#define I2C_SDA 33
#define I2C_SCL 32


MPU9250 mpu;



//Vars for RC control
bool manualControl = true;

int rudderPos = centeredRudder;  // variable to store the servo position. 135 represents centered rudder
int trimPos = closeHauledTrim;          //The current trim of the sail


const char *ssid = "Cyber-Physical-Systems";  //Replace with the wifi SSID
const char *password = "Torweg-Sisal2-Hm";    //Replace with the wifi Pass

const int adjustmentAngle = 30;  // Angle to put the rudder at when adjusting heading
const int tackingAngle = 45;     // Angle to put the rudder at when completing a tack

float pitch;
float roll;
float yaw;
float heading;  // The heading of the boat, given in 360 degrees
int counter = 1;

int windDir = 50;         // Wind direction, input manually based on readings from wind vane
int windSpeed = 5; // Wind speed in knots. This is used for calculating time spent during tacks in upwind sailing
const int noGoZone = 45;  // Specified absolute value of the no-go zone. Determined through experimentation and dependent on boat.
bool onStarboard;         // Boolean representing the tack that the boat is on
int targetAngle = 0;    // Target direction of travel with reference to the wind direction. Range of [-180:180] with positive values for port tack, negative for starboard.
int targetHeading;
char *curAction;
int tolerance = 15;  // Tolerated range of headings (+- degrees specified here)

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

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 300;

// Initialize LittleFS (This is used for handling the creation of the webpage to be served)
void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  Serial.println("LittleFS mounted successfully");
}

// Initialize WiFi. If connection is not established within 5 seconds, restart the ESP and try again.
void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  int retryCount = 0;
  const int maxRetries = 5;     // Number of retries before resetting
  const int retryDelay = 1000;  // Delay between retries in milliseconds

  while (WiFi.status() != WL_CONNECTED && retryCount < maxRetries) {
    delay(retryDelay);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi. Retrying...");
    delay(3000);    // Wait before retrying
    ESP.restart();  // Restart the ESP32 to retry connection
  }
}

//Get Sensor Readings and return JSON object
String getSensorReadings() {
  //TODO: IMPLEMENT FURTHER METRICS
  readings["heading"] = String(heading);
  readings["windDir"] = String(windDir);
  readings["targetHeading"] = String(targetHeading);
  if (onStarboard) {
    readings["currentTack"] = String(pointOfSailToString(pointOfSail)) + String(" (S)");
  } else {
    readings["currentTack"] = String(pointOfSailToString(pointOfSail)) + String(" (P)");
  }
  readings["headingTolerance"] = String(tolerance);
  readings["currentAction"] = String(curAction);
  String jsonString = JSON.stringify(readings);
  return jsonString;
}



void setup() {
  Serial.begin(115200);
  initLittleFS();
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(2000);

  //Init servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  rudderServo.setPeriodHertz(50);  // standard 50 hz servo
  rudderServo.attach(rudderPin, 500, 2300);
  rudderServo.write(centeredRudder);
  trimServo.setPeriodHertz(50);  // standard 50 hz servo
  trimServo.attach(trimPin, 500, 2300);
  trimServo.write(closeHauledTrim);


  configureRCPins();
  setCenteredRudder(115);
  setTrimConditions(50, 140);

  //Init ESP32
  if (!mpu.setup(0x68)) {  // change to your own address
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

  /*******
//
//
//
  connectToWiFi();

  //Init Webserver
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {  // Web Server Root URL
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");  //Serve static pages (css, js)

  // For sending data
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {  // Request for the latest sensor readings
    String json = getSensorReadings();
    request->send(200, "application/json", json);
    json = String();
  });

  // Handle POST requests to /update-windDir
  server.on("/windDir", HTTP_POST, [](AsyncWebServerRequest *request) {
    String newDir;
    if (request->hasParam("windDir")) {
      newDir = request->getParam("windDir")->value();
      int windDirection = newDir.toInt();

      Serial.print("Received new wind direction: ");
      Serial.println(windDirection);
      windDir = windDirection;

      request->send(200, "text/plain", "Wind direction updated successfully.");
    } else {
      request->send(400, "text/plain", "Invalid request");
    }
  });

  events.onConnect([](AsyncEventSourceClient *client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();  // Start server
//
//
//
*******/
}



void loop() {
  /******
  if ((millis() - lastTime) > timerDelay) {
    // Send Events to the client with the Sensor Readings Every .5 seconds
    events.send("ping", NULL, millis());
    events.send(getSensorReadings().c_str(), "new_readings", millis());
    lastTime = millis();
  }
  *******/
  if (manualControl) {
    Serial.println("IN RC MODE");
    processRC();
    switchManualControl();
  } else if (mpu.update()) {
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
      trimServo.write(trimPos);  //Trim the sail to the appropriate angle to maximize speed.

      //Check to see if the attempted path is in the no-go zone.
      if (abs(targetAngle) <= noGoZone) {
        /*Transform the target heading into segments of close-hauled sailing 
        (sailing as close as possible to the wind while still maintaining a fast speed). 
        Default time on each tack is 10 seconds (ratio is adjusted depending on desired angle).*/
        float headingRatio = targetAngle / 45.0;
        int tackLength = 100;
        int timeOnPort = tackLength + (tackLength * headingRatio);
        int timeOnStarboard = tackLength - (tackLength * headingRatio);

        if ((counter % (2 * tackLength)) == timeOnStarboard || (counter % (2 * tackLength)) == 0) {
          executeTack();
        } else if ((counter % (2 * tackLength)) > timeOnStarboard) {
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
          curAction = "Bearing Off";
          rudderPos = centeredRudder + adjustmentAngle;
          rudderServo.write(rudderPos);
        } else if ((heading > offCourse[0] && heading < offCourse[1]) || (heading > offCourse[2] && heading < offCourse[3])) {  //SAILING OFF COURSE. NEED TO HEAD UP
          Serial.println("I'm sailing too deep, heading up now!");
          curAction = "Heading Up";
          rudderPos = centeredRudder - adjustmentAngle;
          rudderServo.write(rudderPos);
        } else if (rudderPos != centeredRudder) {
          rudderPos = centeredRudder;
          curAction = "Straight";
          rudderServo.write(rudderPos);
        }
      }
      // ON PORT TACK
      else {
        if ((heading > luffing[0] && heading < luffing[1]) || (heading > luffing[2] && heading < luffing[3])) {  //SAILING TOO HIGH. NEED TO BEAR OFF
          Serial.println("I'm luffing, bearing off now!");
          curAction = "Bearing Off";
          rudderPos = centeredRudder - adjustmentAngle;
          rudderServo.write(rudderPos);
        } else if ((heading > offCourse[0] && heading < offCourse[1]) || (heading > offCourse[2] && heading < offCourse[3])) {  //SAILING OFF COURSE. NEED TO HEAD UP
          Serial.println("I'm sailing too deep, heading up now!");
          curAction = "Heading Up";
          rudderPos = centeredRudder + adjustmentAngle;
          rudderServo.write(rudderPos);
        } else if (rudderPos != centeredRudder) {
          rudderPos = centeredRudder;
          curAction = "Straight";
          rudderServo.write(rudderPos);
        }
      }

      print_info();  //For use when connected via usb
      counter += 1;
      prev_ms = millis();
      switchManualControl();
    }
  }
}

/* For when the user wants to use remote control */
void processRC() {
  ch_3 = pulseIn(CH_3_PIN, HIGH, 25000);
  ch_1 = pulseIn(CH_1_PIN, HIGH, 25000);
  delay(20);
  rudderPos = map(ch_1, 1245, 1845, centeredRudder - tackingAngle, centeredRudder + tackingAngle);
  trimPos = map(ch_3, 940, 2120, closeHauledTrim, runningTrim);
  rudderServo.write(rudderPos);
  trimServo.write(trimPos);
}

/* Detects if the user wants to switch to manual control */
void switchManualControl() {
  ch_4 = pulseIn(CH_4_PIN, HIGH, 25000);
  delay(20);
  if (ch_4 > 1800) {
    for (int i = 0; i < 50; i++) {
      ch_4 = pulseIn(CH_4_PIN, HIGH, 25000);
      if (ch_4 < 1800) {
        return;
      }
      delay(20);
    }
    if (manualControl == true) {
      manualControl = false;
    } else {
      manualControl = true;
    }
  }
}


/*
Helper function to determine the current point of sail. No go zone is set as default to be 45 degrees
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

/*
Helper function to take the current point of sail and set the appropriate trim angle
*/
void setSailTrim() {
  PointOfSail pos = pointOfSail;
  int sailTrimIncrement = (runningTrim - closeHauledTrim) / 4;
  switch (pointOfSail) {
    case CLOSE_HAULED:
      trimPos = closeHauledTrim;
      return;
    case NO_GO_ZONE:
      trimPos = closeHauledTrim;
      return;
    case CLOSE_REACH:
      trimPos = closeHauledTrim + sailTrimIncrement;
      return;
    case BEAM_REACH:
      trimPos = closeHauledTrim + 2 * sailTrimIncrement;
      return;
    case BROAD_REACH:
      trimPos = closeHauledTrim + 3 * sailTrimIncrement;
      return;
    case RUNNING:
      trimPos = runningTrim;
      return;
    default:
      trimPos = closeHauledTrim;
      return;
  }
}

/*
Helper function to determine what the range of acceptable headings are for a given course and tolerance. 
*/
void set_heading_tolerance(int targetAngle, int tolerance) {
  targetHeading = (windDir + targetAngle) % 360;
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
  //Length of tack in 1/10 seconds. The delay is 5 seconds for 5 knots of wind.
  int tackDelay = (5 / windSpeed) * 50;
  int tackDelayCounter = 0;
  if (onStarboard) {
    rudderPos = centeredRudder - tackingAngle;
    rudderServo.write(rudderPos);
    Serial.println("Tacking from starboard to port!");
    //Working under the assumption that the tack takes about 10 seconds to complete. Ultimately, this will be dependent on wind and boat speed.
  } else {
    //Put the rudder all the way to port, perhaps trim sail slightly as this is done.
    rudderPos = centeredRudder + tackingAngle;
    rudderServo.write(rudderPos);
    Serial.println("Tacking from port to starboard!");
    //Working under the assumption that the tack takes about 10 seconds to complete. Ultimately, this will be dependent on wind and boat speed.
  }
  while (tackDelayCounter < tackDelay) {
    tackDelayCounter += 1;
    mpu.update();
    pitch = mpu.getPitch();
    roll = mpu.getRoll();
    yaw = mpu.getYaw();
    delay(100);
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

/*
Helper function to print the current information about the boat.
*/
void print_info() {
  Serial.print("Wind: ");
  Serial.print(windDir);
  Serial.print("  Target heading:");
  Serial.print(targetHeading);
  Serial.print(" Heading: ");
  Serial.print(heading, 2);
  Serial.print(" Trim pos: ");
  Serial.print(trimPos);
  Serial.print(" Luffing: [");
  for (int i = 0; i < 4; i++) {
    Serial.print(luffing[i]);
    if (i < 3) Serial.print(", ");
  }
  Serial.print("]\t Offcourse: [");
  for (int i = 0; i < 4; i++) {
    Serial.print(offCourse[i]);
    if (i < 3) Serial.print(", ");
  }
  Serial.print("] Current Tack: ");
  if (onStarboard) {
    Serial.print("(S), ");
  } else {
    Serial.print("(P), ");
  }
  Serial.println(pointOfSailToString(pointOfSail));
}


