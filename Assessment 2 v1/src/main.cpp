#include <Arduino.h>
#include <EEPROM.h>
#include "SearchAlgorithms.h"
#include "linesensor.h"

// Define number of waypoints
#define NUM_WAYPOINTS 10

// Define the sensor dead time
#define SENSOR_DEAD_TIME 1000

// Call the classes
SearchAlgorithms searchAlgorithms;
LineSensorClass lineSensor;
// KinematicsClass kinematicsrun;

// Store the number of waypoints found
int numWaypointsFound = 0;

// Structure to store coordinates waypoint coordinates for EEPROM storage
struct waypointCoordinates {
  float x;
  float y;
};

// Array to store the sensed waypoints
waypointCoordinates sensedWaypoints[NUM_WAYPOINTS];

// Store the last time the waypoint was sensed
unsigned long lastWaypointTime = 0;

// Define functions
void storeWaypoints();

void setup() {

  // Run the line sensor initialisation
  lineSensor.init();

  // Start the serial monitor
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  delay(2000);
  
}

void loop() {
  

}

// Function to store the sensed waypoints when the robot is in the search pattern
void storeWaypoints() {

  // if statement to store data if the line sensor detects a waypoint
  if (lineSensor.LineDetected() && numWaypointsFound < NUM_WAYPOINTS && millis() - lastWaypointTime > SENSOR_DEAD_TIME) {

    // Store the time the waypoint was sensed
    lastWaypointTime = millis();

    // Store the waypoint coordinates
    // sensedWaypoints[numWaypointsFound].x = kinematicsrun.x;
    // sensedWaypoints[numWaypointsFound].y = kinematicsrun.y;

  };

}
