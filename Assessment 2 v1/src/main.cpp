#include <Arduino.h>
#include "SearchAlgorithmsClass.h"
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
SearchAlgorithmsClass searchAlgorithms;
LineSensorClass lineSensor;
Kinematics_c kinematicsrun;

// Store the number of waypoints found
int numWaypointsFound = 0;
int algorithminterval = 0;

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

  // Get the coordinates of the search algorithms (amplitude, wavelength)
  searchAlgorithms.sinSearch(100, 1000);
  // searchAlgorithms.squareWaveSearch(100, 1000);
  // searchAlgorithms.randomSearch();
  
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


  //  logic to check has robot arrived at next waypoint, if so count up find the next waypoint, calculate the angle to turn to and enact the turn

  
  //  turn this into a line of code that finds the difference between x and xi and y and yi in kinematics run.update
  //  so that this can be an inequality and when this difference gets less than 0.1 or equivalent value
  // if (kinematicsrun.x = waypointCoordinates.x[algorithminterval] && y = waypointCoordinates.y[algorithminterval]) 
  // algorithminterval =+ 1;
  // kinematicsrun.target_angle( waypointCoordinates.x[algorithminterval], waypointCoordinates.y[algorithminterval])
  //   if (kinematicsrun.theta_turn < 0) {
  //     setMotorpower(-20, 20)
  //   }
  //   if (kinematicsrun.theta_turn > 0) {
  //     setMotorpower(20, -20)
  //   }
  //   if (abs(kinematicsrun.theta_turn) < 0.10) {
  //     setMotorpower(0, 0)
  //   }

  //   //  enact drive forward pid control function






  





}

// Function to store the sensed waypoints when the robot is in the search pattern
void storeWaypoints() {

  // if statement to store data if the line sensor detects a waypoint
  if (lineSensor.LineDetected() && numWaypointsFound < NUM_WAYPOINTS && millis() - lastWaypointTime > SENSOR_DEAD_TIME) {

    // Store the time the waypoint was sensed
    lastWaypointTime = millis();

    // Store the waypoint coordinates
    sensedWaypoints[numWaypointsFound].x = kinematicsrun.x;
    sensedWaypoints[numWaypointsFound].y = kinematicsrun.y;

  };

}
