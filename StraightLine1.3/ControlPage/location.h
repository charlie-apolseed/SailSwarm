#ifndef LOCATION_H
#define LOCATION_H


#include <boat_config.h>
#include <Arduino.h>

struct Coordinate {
  double lat;
  double lng;
};
const int maxCoordinates = 10;
Coordinate coordinates[maxCoordinates];
int currentCoordinatesSize = 0; 

Coordinate curPosition = {47.6952370, 9.1937710};
Coordinate curTarget = {0, 0};

void addCoordinate(double lat, double lng) {
  if (currentCoordinatesSize < maxCoordinates) {
    coordinates[currentCoordinatesSize].lat = lat;
    coordinates[currentCoordinatesSize].lng = lng;
    currentCoordinatesSize++;
  } else {
    coordinates[currentCoordinatesSize].lat = lat;
    coordinates[currentCoordinatesSize].lng = lng;
  }
  curTarget = coordinates[0];
}

void setCurrentPosition(double lat, double lng) {
  curPosition.lat = lat;
  curPosition.lng = lng;
}

void setCurrentTarget(double lat, double lng) {
  curTarget.lat = lat;
  curTarget.lng = lng;
}

/*Get the coordinates for the current target */
Coordinate getCurrentTarget() {
  return curTarget;
}

Coordinate getCurrentPosition() {
  return curPosition;
}

String getCoordinatesList() {
  String message = "";
  for (int i = 0; i < currentCoordinatesSize; i++) {
    message += "(" + String(coordinates[i].lat, 6) + "," + String(coordinates[i].lng, 6) + ")\n";
  }
  return message;
}

/**
Updates the target coordinates list. If there are no more points to target, returns true and triggers station keeping to begin.

  @return true if the boat has reached its final destination. 
*/
bool updateCoordinatesList() {
  if (currentCoordinatesSize == 0) {
    return true;
  }
  currentCoordinatesSize -= 1;
  for (int i = 0; i < currentCoordinatesSize; i++) {
    coordinates[i] = coordinates[i + 1];
  } 
  coordinates[currentCoordinatesSize] = {0, 0};
  if (currentCoordinatesSize == 0) {
    return true;
  }
  curTarget = coordinates[0];
  return false; 
}

void clearCoordinatesList() {
  for (int i = 0; i < maxCoordinates; i++) {
    coordinates[i] = {0, 0};
  }
  currentCoordinatesSize = 0;
  curTarget = {0, 0};
}




#endif  // LOCATION_H