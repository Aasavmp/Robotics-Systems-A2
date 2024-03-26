// Class to get coordinares of the different search algorithms
#include <Arduino.h>
#include <math.h>

#ifndef _SEARCHALGORITHMS_H
#define _SEARCHALGORITHMSClass_H

#define RESOLUTION 100
#define X_MIN 0 // mm
#define X_MAX 1000 // mm
#define Y_MIN -500 // mm
#define Y_MAX 500 // mm

class SearchAlgorithmsClass {

    public:

        // Variables to store the coordinates of the search algorithms
        float x[RESOLUTION];
        float y[RESOLUTION];

        // Constructor
        SearchAlgorithmsClass() {

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

            // Set the first point to 0,0
            x[0] = 0;
            y[0] = 0;

            // Loop through the random resolution
            for (int i = 1; i < RESOLUTION; i++) {

                // Calculate the x and y coordinates
                x[i] = random(X_MIN, X_MAX);
                y[i] = random(Y_MIN, Y_MAX);

            }

        }

        
};

#endif