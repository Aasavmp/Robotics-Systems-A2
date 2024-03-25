#include <Arduino.h>
#include <EEPROM.h>
#include "SearchAlgorithms.h"
#include "linesensor.h"

// Define number of waypoints
#define NUM_WAYPOINTS 10

// Call the classes
SearchAlgorithms searchAlgorithms;
LineSensorClass lineSensor;

// Structure to store coordinates waypoint coordinates for EEPROM storage
struct waypointCoordinates {
  float x;
  float y;
};

// Array to store the sensed waypoints
waypointCoordinates sensedWaypoints[NUM_WAYPOINTS];

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
