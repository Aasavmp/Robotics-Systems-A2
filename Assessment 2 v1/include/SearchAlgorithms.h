// Class to get coordinares of the different search algorithms
#include <Arduino.h>
#include <math.h>

#ifndef _SEARCHALGORITHMS_H
#define _SEARCHALGORITHMS_H

#define RESOLUTION 1000
#define X_MIN 0 // mm
#define X_MAX 1000 // mm
#define Y_MIN -500 // mm
#define Y_MAX 500 // mm

class SearchAlgorithms {

    public:

        // Variables to store the coordinates of the search algorithms
        float x[RESOLUTION];
        float y[RESOLUTION];

        // Constructor
        SearchAlgorithms() {

        }

        // Function for a sinusoidal search pattern
        void sinSearch(float amplitude, float wavelength) {

            // Loop through the resolution
            for (int i = 0; i < RESOLUTION; i++) {

                // Calculate the x and y coordinates
                x[i] = X_MIN + i * (X_MAX - X_MIN) / RESOLUTION;
                y[i] = amplitude * sin(x[i] * ((2*PI) / wavelength));

            };
                
        }

        // Function to get the coordinates of a square search pattern
        void squareWaveSearch(float amplitude, float wavelength) {

            // Loop through the resolution
            for (int i = 0; i < RESOLUTION; i++) {

                // Calculate the x and y coordinates
                x[i] = X_MIN + i * (X_MAX - X_MIN) / RESOLUTION;
                y[i] = amplitude * (fmod(x[i], wavelength) < wavelength / 2 ? 1 : -1); 

            }

        }

        // Function to get the coordinates of a random search pattern
        void randomSearch() {

            // How do we compare a random search pattern to one of the fixed patterns?
            // Do we store the time taken or distance travelled to complete the fixed search patterns and enforce that time on the random search pattern and then compare the total waypoints found?

        }

        
};

#endif