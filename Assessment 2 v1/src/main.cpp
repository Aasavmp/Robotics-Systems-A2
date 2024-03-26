#include <Arduino.h>
#include <EEPROM.h>
#include "SearchAlgorithms.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"

// Define number of waypoints
#define NUM_WAYPOINTS 10

// define timestamps
#define kinematics_time_interval 20
unsigned long controltimestamp;

// Define the sensor dead time
#define SENSOR_DEAD_TIME 1000

// Call the classes
SearchAlgorithms searchAlgorithms;
LineSensorClass lineSensor;
Kinematics_c kinematicsrun;
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
  kinematicsrun.initialise();

  // Start the serial monitor
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  delay(2000);

  controltimestamp = millis();
  
}

void loop() {

  unsigned long currentMillis = millis();



      // Update kinematics every 20 ms
  if (currentMillis - controltimestamp >= kinematics_time_interval) {
    controltimestamp = currentMillis;
    // Call the updateKinematics method of the Kinematics instance
    kinematicsrun.update(count_leftenc, count_rightenc);
    // x, y, theta_1, total_distance = updateKinematics();
  }
  





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
