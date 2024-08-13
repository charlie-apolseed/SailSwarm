#ifndef LOCATION_H
#define LOCATION_H


#include <boat_config.h>
#include <Arduino.h>


double xCoordPointA = 47.6952370;  //Default is the Limno center
double yCoordPointA = 9.1937710;
double xCoordPointB;
double yCoordPointB;

//Variable representing the acceptable error from gps location
double gpsTolerance;

// Setter for xCoordPointA
void setXCoordPointA(double x) {
  xCoordPointA = x;
}

// Setter for yCoordPointA
void setYCoordPointA(double y) {
  yCoordPointA = y;
}

// Getter for xCoordPointA
double getXCoordPointA() {
  return xCoordPointA;
}

// Getter for yCoordPointA
double getYCoordPointA() {
  return yCoordPointA;
}

// Setter for xCoordPointB
void setXCoordPointB(double x) {
  xCoordPointB = x;
}

// Setter for yCoordPointB
void setYCoordPointB(double y) {
  yCoordPointB = y;
}

// Setter for gpsTolerance
void setGpsTolerance(double tolerance) {
  gpsTolerance = tolerance;
}

/** Determines the target heading based on the 2 input points 

  @return the new targetHeading
*/
int pointToPointHeading() {

  double dx = xCoordPointB - xCoordPointA;
  double dy = yCoordPointB - yCoordPointA;
  double theta = atan2(dy, dx);

  // Convert from radians to degrees
  theta = theta * 180 / PI;

  // Adjust the angle to start from 0 degrees at the positive y-axis and go clockwise
  theta = 90 - theta;


  // Normalize theta to [0, 360) range
  if (theta < 0) {
    theta += 360;
  } else if (theta >= 360) {
    theta -= 360;
  }
  Serial.print("Dx: ");
  Serial.print(dx, 8);
  Serial.print(" Dy: ");
  Serial.print(dy, 8);
  Serial.print("Getting new heading:");
  Serial.println(theta);
  return round(theta);
}

/**
Checks to see if the boat is at the desired location (point B). 

  @return true if the boat is in the correct place.
*/
bool atDestination() {

  if (((xCoordPointA > xCoordPointB - gpsTolerance) && (xCoordPointA < xCoordPointB + gpsTolerance)) &&
      //At the correct x position
      ((yCoordPointA > yCoordPointB - gpsTolerance) && (yCoordPointA < yCoordPointB + gpsTolerance))) {
    return true;
  }
return false;
}






#endif  // LOCATION_H