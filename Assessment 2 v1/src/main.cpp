#include <Arduino.h>
#include "SearchAlgorithmsClass.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "Motors.h"

// Define number of waypoints
#define NUM_WAYPOINTS 10

// define timestamps
#define kinematics_time_interval 20
#define rotational_time_interval 10
#define searchtimeint 15000 // 1 minute search time
#define PID_INTERVAL 100

// Define the sensor dead time
#define SENSOR_DEAD_TIME 1000

// Independent variables to change
float set_pid_bias = 5;
float turn_power_factor = 1;
float search_amplitude = 0.1;
float search_wavelength = 0.2;
int search_algorithm = 2; // 1 = sin, 2 = square, 3 = random

// Define time stamps
unsigned long controltimestamp;
unsigned long rotational_timestamp;
unsigned long searchtimestamp;
unsigned long pid_timestamp;
unsigned long statemilliseconds;
unsigned long elapsed_time;
unsigned long start_time;
unsigned long end_time;

// defining required variables
float velocity_left;
float velocity_right;
float leftpwm;
float rightpwm;
float speed_feedback_left;
float speed_feedback_right;
float feedback_heading;
float feedback_heading_fast;

// speed PID definifintions
const float Kpspeed = 8; // Adjust as needed
const float demand_speed = 4; // Demand value
const float Kispeed = 0.0001; // Integral gain

// heading PID definitions
const float kp_heading = 4; // Adjust as needed
const float ki_heading = 2; 

const float kp_heading_fast = 9; // Adjust as needed
const float ki_heading_fast = 0.7; 

// Define the PID bias
float pid_bias = 0;
int max_turnpwm = 100;

// Define the turn threshold
float turnThresholdOne = 0.2;
float turnThresholdTwo = 0.1;

// Define the algorithm threshold for when the robot reaches its target algorithm point
float algorithmThreshold = 0.01;

// Call the classes
SearchAlgorithmsClass searchAlgorithms;
LineSensorClass lineSensor;
Kinematics_c kinematicsrun;
PIDController pidControllerleft(Kpspeed,Kispeed);
PIDController pidControllerright(Kpspeed,Kispeed);
PIDController pidControllerheading(kp_heading,ki_heading);
PIDController pidControllerheading_fast(kp_heading_fast,ki_heading_fast);

float heading_home_feedback = 0;

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
void searchState();

void setup() {
  
  // Run the line sensor initialisation
  lineSensor.init();
  kinematicsrun.initialise();
  pidControllerleft.initialize();
  pidControllerright.initialize();
  pidControllerheading.initialize();
  pidControllerheading_fast.initialize();

  // Initialise the search algorithms
  searchAlgorithms.init(search_algorithm, search_amplitude, search_wavelength);

  // Start the serial monitor
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  delay(2000);

  controltimestamp = millis();
  start_time = millis();

}

void loop() {

  // If the robot hasn't found all the waypoints, run the entire search algorithm and run for a set time run the search state
  if (numWaypointsFound < NUM_WAYPOINTS && millis()-start_time < searchtimeint && algorithminterval < RESOLUTION) {
    
    // Run the search state
    searchState();

    // Save the end time
    end_time = millis();

  } else {

    // Set the motors to stop
    setMotorPower(0, 0);

    Serial.print("Search complete in: ");
    Serial.print(end_time-start_time);
    Serial.println(" milliseconds");

    // Print the sensed waypoints
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
      Serial.print(numWaypointsFound);
      Serial.print(",");
      Serial.print(sensedWaypoints[i].x);
      Serial.print(",");
      Serial.println(sensedWaypoints[i].y);
    }

    delay(2000);

  }
  
}

// Function for the search state
void searchState() {

  // Get the current time
  unsigned long currentMillis = millis();

  // Update kinematics every 20 ms
  if (currentMillis - controltimestamp >= kinematics_time_interval) {
    controltimestamp = currentMillis;
    // Call the updateKinematics method of the Kinematics instance
    kinematicsrun.update(count_leftenc, count_rightenc, searchAlgorithms.x[algorithminterval], searchAlgorithms.y[algorithminterval]);
    // x, y, theta_1, total_distance = updateKinematics();
  }

  // Update rotational speed every 10 ms
  if (currentMillis - rotational_timestamp >= rotational_time_interval) {
    rotational_timestamp = currentMillis;

    // Call the rotationalspeed method of the PIDController instance
    pidControllerleft.rotationalspeed(currentMillis);
    pidControllerright.rotationalspeed(currentMillis);
  } 
 
  //  logic to check has robot arrived at next search point, if so count up find the next search point, calculate the angle to turn to and enact the turn
  if (kinematicsrun.xdif < algorithmThreshold && kinematicsrun.xdif > -algorithmThreshold && kinematicsrun.ydif < algorithmThreshold && kinematicsrun.ydif > -algorithmThreshold) {
      
    // increment the algorithm interval
    algorithminterval += 1;

    // Reset the heading feedback integral term
    pidControllerheading.integral = 0;
    pidControllerheading_fast.integral = 0;

  }

  // calculate the angle to turn to
  kinematicsrun.targetangle( searchAlgorithms.x[algorithminterval], searchAlgorithms.y[algorithminterval] );

  // Set the pid timestamp
  pidControllerheading.prev_time = millis();
  pidControllerheading_fast.prev_time = millis();
  pidControllerleft.prev_time = millis();
  pidControllerright.prev_time = millis();

  // Update the PID heading controller
  pidControllerheading.update(kinematicsrun.target_angle, kinematicsrun.theta);
  pidControllerheading_fast.update(kinematicsrun.target_angle, kinematicsrun.theta);

  // Get the heading feedback
  float heading_feedback = pidControllerheading.getProportionalTerm();
  float heading_feedback_fast = pidControllerheading_fast.getProportionalTerm();

  // Set the heading PID and bias
  if (abs(kinematicsrun.theta_turn) > turnThresholdOne) {
    heading_feedback = heading_feedback_fast;
    pid_bias = 1;
  } else if (abs(kinematicsrun.theta_turn) > turnThresholdTwo) {
    heading_feedback = heading_feedback_fast;
    pid_bias = set_pid_bias;
  } else {
    pid_bias = set_pid_bias;
  }

  // Set the left and right PWM values
  leftpwm = pid_bias - (heading_feedback*turn_power_factor);
  rightpwm = pid_bias + (heading_feedback*turn_power_factor);

  // Update the PID speed controller
  pidControllerleft.update(leftpwm, pidControllerleft.lpf_l);
  pidControllerright.update(rightpwm, pidControllerright.lpf_r);

  // Set the motor power
  setMotorPower(pidControllerleft.p_term, pidControllerright.p_term);

  // Sense and store waypoints
  storeWaypoints();

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

    // Increment the number of waypoints found
    numWaypointsFound += 1;

  };

}
