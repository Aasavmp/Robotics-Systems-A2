#include <Arduino.h>
// #include <sensors.h>  
// #include <Motors.h>
// #include <bangbang.h>
# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

# define FWD LOW
# define REV HIGH


void setMotorPower( float left_pwm, float right_pwm ) {

  // Check for valid input. if the magintude is greater than 255, return only 255
  

  // if ( left_pwm < -255 || left_pwm > 255 ) {
  //   Serial.println("Invalid left_pwm value.");
  //   // turn on left led 
    

  //   return;
  // }
  // if ( right_pwm < -255 || right_pwm > 255 ) {
  //   Serial.println("Invalid right_pwm value.");
  //   return;
  // }
  if ( left_pwm < -255 ) {
    left_pwm = -255;
  }
  if ( right_pwm < -255 ) {
    right_pwm = -255;
  }
  if ( left_pwm > 255 ) {
    left_pwm = 255;
  }
  if ( right_pwm > 255 ) {
    right_pwm = 255;
  }


  // Set direction.
  if ( left_pwm < 0 ) {
    digitalWrite( L_DIR_PIN, REV );
  } else {
    digitalWrite( L_DIR_PIN, FWD );
  }
  if ( right_pwm < 0 ) {
    digitalWrite( R_DIR_PIN, REV );
  } else {
    digitalWrite( R_DIR_PIN, FWD );
  }

  // Set power.
  analogWrite( L_PWM_PIN, abs(left_pwm) );
  analogWrite( R_PWM_PIN, abs(right_pwm) );

}

void followLine(float sensor_2, float sensor_4, float bias_pwm, float max_turnpwm) {
    float sum = sensor_2 + sensor_4;
    float sensor_2_weighted = (sensor_2 / sum) * 2;
    float sensor_4_weighted = (sensor_4 / sum) * 2;
    float difference = sensor_2_weighted - sensor_4_weighted;

    float leftpwm = bias_pwm - (difference * max_turnpwm);
    float rightpwm = bias_pwm + (difference * max_turnpwm);

    // Print debug information
    // Serial.print("Left PWM: ");
    // Serial.print(leftpwm);
    // Serial.print(", Right PWM: ");
    // Serial.println(rightpwm);

    // Set motor power
    setMotorPower(leftpwm, rightpwm);
}

// // move forward function
// void moveForward() {
//   setMotorPower( 30, 30 );
// }