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
#define searchtimeint 1000
#define PID_INTERVAL 100
// Define the sensor dead time
#define SENSOR_DEAD_TIME 1000

unsigned long controltimestamp;
unsigned long rotational_timestamp;
unsigned long searchtimestamp;
unsigned long pid_timestamp;
unsigned long statemilliseconds;
unsigned long elapsed_time;


// defining required variables
float velocity_left;
float velocity_right;
float leftpwm;
float rightpwm;
float speed_feedback_left;
float speed_feedback_right;
float feedback_heading;

// speed PID definifintions

const float Kpspeed = 8; // Adjust as needed
const float demand_speed = 4; // Demand value
const float Kispeed = 0.0001; // Integral gain

const float kp_heading = 1; // Adjust as needed
const float ki_heading = 0.0001; 
float demand_heading = 0; /// Demand value

int pid_bias = 5;
int max_turnpwm = 100;

float turnThreshold = 0.5;

// Call the classes
SearchAlgorithmsClass searchAlgorithms;
LineSensorClass lineSensor;
Kinematics_c kinematicsrun;
PIDController pidControllerleft(Kpspeed,Kispeed);
PIDController pidControllerright(Kpspeed,Kispeed);
PIDController pidControllerheading(kp_heading,ki_heading);

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


void setup() {
  
  // Run the line sensor initialisation
  lineSensor.init();
  kinematicsrun.initialise();
  pidControllerleft.initialize();
  pidControllerright.initialize();

  // Start the serial monitor
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  delay(2000);

  controltimestamp = millis();

  // Get the coordinates of the search algorithms (amplitude, wavelength)
  // searchAlgorithms.sinSearch(100, 1000);
  searchAlgorithms.squareWaveSearch(0.1, 0.05);
  // searchAlgorithms.randomSearch();
  
}

void loop() {

  // Get the current time
  unsigned long currentMillis = millis();

  // Update kinematics every 20 ms
  if (currentMillis - controltimestamp >= kinematics_time_interval) {
    controltimestamp = currentMillis;
    // Call the updateKinematics method of the Kinematics instance
    kinematicsrun.update(count_leftenc, count_rightenc);
    // x, y, theta_1, total_distance = updateKinematics();
  }

  // Update rotational speed every 10 ms
  if (currentMillis - rotational_timestamp >= rotational_time_interval) {
    rotational_timestamp = currentMillis;

    // Call the rotationalspeed method of the PIDController instance
    pidControllerleft.rotationalspeed(currentMillis);
    pidControllerright.rotationalspeed(currentMillis);
  } 

  Serial.print(algorithminterval);
   Serial.print(",");
  Serial.print(kinematicsrun.x);
  Serial.print(",");
  Serial.print(kinematicsrun.y);
  Serial.print(",");
  Serial.print(searchAlgorithms.x[algorithminterval]);
  Serial.print(",");
  Serial.print(searchAlgorithms.y[algorithminterval]);
  Serial.print(",");  
  Serial.print(kinematicsrun.target_angle);
  Serial.print(",");
  Serial.println(kinematicsrun.theta_turn);
 
  //  logic to check has robot arrived at next waypoint, if so count up find the next waypoint, calculate the angle to turn to and enact the turn
  //  turn this into a line of code that finds the difference between x and xi and y and yi in kinematics run.update
  //  so that this can be an inequality and when this difference gets less than 0.1 or equivalent value
  if (kinematicsrun.x == searchAlgorithms.x[algorithminterval] && kinematicsrun.y == searchAlgorithms.y[algorithminterval]) {
      
    //  find coordinates of next waypoint
    algorithminterval =+ 1;

  }

  // calculate the angle to turn to
  kinematicsrun.targetangle( searchAlgorithms.x[algorithminterval], searchAlgorithms.y[algorithminterval]);
  
  if (kinematicsrun.theta_turn < -turnThreshold) {
        setMotorPower(20, -20);
  }
  if (kinematicsrun.theta_turn > turnThreshold) {
        setMotorPower(-20, 20);
        
  }
  if (abs(kinematicsrun.theta_turn) < turnThreshold) {
    pidControllerheading.prev_time = millis();
    pidControllerleft.prev_time = millis();
    pidControllerright.prev_time = millis();

    heading_home_feedback += kinematicsrun.angle;
  
    
    pidControllerheading.update(0 , heading_home_feedback);

    float heading_feedback = pidControllerheading.getProportionalTerm();
    // Serial.print(heading_feedback);
    // Serial.print(",");

    leftpwm = pid_bias - (heading_feedback);
    rightpwm = pid_bias + (heading_feedback);


    pidControllerleft.update(leftpwm, pidControllerleft.lpf_l);
    pidControllerright.update(rightpwm, pidControllerright.lpf_r);

  

    setMotorPower(pidControllerleft.p_term, pidControllerright.p_term);
  // setMotorPower(50,50);

  }

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

  };

}
