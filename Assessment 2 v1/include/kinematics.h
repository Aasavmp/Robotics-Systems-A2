
#include <Arduino.h>
// #include <Encoder.h>
// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Class to track robot position.
class Kinematics_c {
  public:

    int wheelradius = 0.0325;
    float x = 0;
    float y = 0;
    float theta = 0;
    volatile long oldcount_leftenc = 0;
    volatile long oldcount_rightenc = 0;
    float leftcountchange;
    float rightcountchange;
    float left_distance;
    float right_distance;
    float theta_degrees;
    float total_distance;
    float theta_back;
    float angle;
    float wheel_distance = 0.0878;
    float target_angle;
    float theta_turn;

  
    // Constructor, must exist.
    Kinematics_c() {

    } 

    void initialise() {
      // put your setup code here, to run once:
      // pinMode( 30, OUTPUT );
      setupEncoder0();
      setupEncoder1();
    }



    // Use this function to update
    // your kinematics
    void update(int count_leftenc, int count_rightenc) {
        // get the encoder values from the ISR function


      // subtract new count from old count for left and right and assign it to a variable
      // if the difference is greater than 358 then calculate the distance and angle and update the x, y and theta values
      // then update the oldcount_leftenc and oldcount_rightenc to the current count_leftenc and count_rightenc
        leftcountchange = count_leftenc - oldcount_leftenc;
        rightcountchange = count_rightenc - oldcount_rightenc;
        oldcount_leftenc = count_leftenc;
        oldcount_rightenc = count_rightenc;

        float left_rotationchange = (leftcountchange / 358.5)  ;
        // Serial.print("=the change in left rotation is");
        // Serial.println(left_rotationchange);
        
        left_distance = left_rotationchange * 0.1021017;
        // Serial.print("=the change in left distance is");
        // Serial.println(left_distance);

        float right_rotationchange = (rightcountchange / 358.5)  ;
        right_distance = right_rotationchange * 0.1021017;
        // Serial.print("=the change in right distance is");
        // Serial.println(right_distance);
        float distance = (left_distance + right_distance) / 2;
        angle = (right_distance-left_distance) / wheel_distance;

        theta += angle;
        float x_change = distance * cos(theta);
        float y_change = distance * sin(theta);
        
        x += x_change;
        y += y_change;
        


        if (theta > PI){
            theta -= 2*PI;
        }
        else if (theta < -PI){
            theta += 2*PI;
        }
        theta_degrees = theta * 180 / PI;

        // calculate a total distance travelled

        total_distance = sqrt((x*x) + (y*y));

    }


    void targetangle(float targetx, float targety){
        target_angle = atan2(targety - y, targetx - x);
        theta_turn = atan2(targety - y, targetx - x) - theta;

        if (target_angle < -PI){
            target_angle += 2*PI;
        }
        else if (target_angle > PI){
            target_angle -= 2*PI;
        }
    }

    void angletohome(){
        
        if (y> 0){
            theta_back = -theta - PI/2 - atan2(x , abs(y));
        }
        else if (y < 0){
            theta_back = -theta + PI/2 + atan2(x , abs(y));
        } 
        if (theta_back < -PI){
            theta_back += 2*PI;
        }
        else if (theta_back > PI){
            theta_back -= 2*PI;
        }
    }

};

#endif