// Class for the line sensors
#include <Arduino.h>

#ifndef LINESENSOR_H
#define LINESENSOR_H

#define EMIT_PIN 11 // IR emitter pin
#define LS_LEFT_PIN 12 // Left most IR sensor pin DN1
#define LS_MIDLEFT_PIN 18 // Middle left IR sensor pin DN2
#define LS_MIDDLE_PIN 20 // Middle IR sensor pin DN3
#define LS_MIDRIGHT_PIN 21 // Middle right IR sensor pin DN4
#define LS_RIGHT_PIN 22 // Right most IR sensor pin DN5

#define MAX_SENSORS 5 // Number of line sensors
#define MAX_CENTER_SENSORS 3 // Number of center line sensors

// Define the array index for each line sensor
#define LS_LEFT_INDEX 0
#define LS_MIDLEFT_INDEX 1
#define LS_MIDDLE_INDEX 2
#define LS_MIDRIGHT_INDEX 3
#define LS_RIGHT_INDEX 4

// define black and white thresholds for each sensor
#define LS_LEFT_THRESHOLD 1200
#define LS_MIDLEFT_THRESHOLD 1200
#define LS_MIDDLE_THRESHOLD 1200
#define LS_MIDRIGHT_THRESHOLD 1200
#define LS_RIGHT_THRESHOLD 1200

// Start class definition
class LineSensorClass {
    public:

        // Store pin numbers in an array for easy access
        int ls_pins[MAX_SENSORS] = {LS_LEFT_PIN, LS_MIDLEFT_PIN, LS_MIDDLE_PIN, LS_MIDRIGHT_PIN, LS_RIGHT_PIN};

        // Store the time elapsed for each sensor
        float parallel_reading_all[MAX_SENSORS];
        float parallel_reading_center[MAX_CENTER_SENSORS];

        // Create an array to store the line sensor thresholds
        int ls_thresholds_all[MAX_SENSORS] = {LS_LEFT_THRESHOLD, LS_MIDLEFT_THRESHOLD, LS_MIDDLE_THRESHOLD, LS_MIDRIGHT_THRESHOLD, LS_RIGHT_THRESHOLD};
        int ls_thresholds_center[MAX_CENTER_SENSORS] = {LS_MIDLEFT_THRESHOLD, LS_MIDDLE_THRESHOLD, LS_MIDRIGHT_THRESHOLD};

        // Constructor
        LineSensorClass() {

        }

        // Initialise the line sensor
        void init() {

            // Set initial pin states
            pinMode(EMIT_PIN, INPUT); // Set IR emitter input for off state
            pinMode(LS_LEFT_PIN, INPUT);
            pinMode(LS_MIDLEFT_PIN, INPUT); 
            pinMode(LS_MIDDLE_PIN, INPUT); 
            pinMode(LS_MIDRIGHT_PIN, INPUT); 
            pinMode(LS_RIGHT_PIN, INPUT); 

        }

        // Turn on IR emitter and set to sensor mode (not bumper mode, low)
        void turnOnEmitter() {
            pinMode(EMIT_PIN, OUTPUT);
            digitalWrite(EMIT_PIN, HIGH);
        }

        // Turn off IR emitter
        void turnOffEmitter() {
            pinMode(EMIT_PIN, INPUT);
        }

        
        // Function to read all 5 line sensors in a parallel arrangement
        void ParallelLineSensorRead() {

            // Turn on the IR emitter
            turnOnEmitter();
        
            // Charge all the line sensor capacitors for 10ms
            for (int i = 0; i < MAX_SENSORS; i++) {
                pinMode(ls_pins[i], OUTPUT);
                digitalWrite(ls_pins[i], HIGH);
            }
            delay(10);

            // Set all the line sensors to input mode
            for (int i = 0; i < MAX_SENSORS; i++) {
                pinMode(ls_pins[i], INPUT);
            }

            // Start the timer
            unsigned long start_time = micros();

            // Set an array to store the time elapsed for each sensor
            float time_elapsed[MAX_SENSORS] = {0, 0, 0, 0, 0};

            // Set a variable to update how many sensors are still high
            int SensorsStillReading = MAX_SENSORS;

            // Loop until all the line sensors go low
            while (SensorsStillReading > 0) {

                // Loop through all the sensors
                for (int i = 0; i < MAX_SENSORS; i++) {

                    // If the sensor has not got a reading continue
                    if (time_elapsed[i] == 0) {

                        // If the sensor has gone low
                        if (digitalRead(ls_pins[i]) == LOW) {

                            // Stop the timer
                            unsigned long end_time = micros();

                            // Calculate the time taken and store it in the array
                            time_elapsed[i] = (float) (end_time - start_time);

                            // Update the number of sensors still reading
                            SensorsStillReading--;

                        } 
                    }
                }

                // Add a time check here to break out of the loop if it takes too long
                if (micros() - start_time > 10000) {

                    // Determine which sensors have not got a reading
                    for (int i = 0; i < MAX_SENSORS; i++) {
                        if (time_elapsed[i] == 0) {
                            time_elapsed[i] = 10000;
                        }
                    }

                    // Break out of the loop
                    break;

                }
            }

            // Store the time elapsed in the class variable
            for (int i = 0; i < MAX_SENSORS; i++) {
                this->parallel_reading_all[i] = time_elapsed[i];

                // append the center sensors to the center sensor array
                if (i == LS_MIDLEFT_INDEX || i == LS_MIDDLE_INDEX || i == LS_MIDRIGHT_INDEX) {
                    this->parallel_reading_center[i-1] = time_elapsed[i];
                }
            }

            // Turn off the IR emitter
            turnOffEmitter();
            
        }

        // Function to determine if a line has been detected
        bool LineDetected() {

            // Start the line sensors
            ParallelLineSensorRead();

            // Run through the central line sensors and check if they are on black
            for (int i = 0; i < MAX_CENTER_SENSORS; i++) {
                if (parallel_reading_center[i] > ls_thresholds_center[i]) {
                return true;
                }
            }

            // If no line is detected then return false
            return false;

        }

};

#endif