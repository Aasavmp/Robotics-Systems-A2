#define distancepercount 0.00014
#define anglepercount 0.01755
#define PID_INTERVAL 100

#ifndef _PID_H
#define _PID_H

extern float velocity_left;
extern float velocity_right;

class PIDController {
private:
   

    float alpha;

public:
    PIDController(float kp, float ki) : Kp(kp), Ki(ki), p_term(0.0), integral(0.0),
                                         controltimestamp(0), prev_count_leftenc(0), prev_count_rightenc(0),
                                         prev_time(0) {}

    float lpf_l;
    float lpf_r;
    unsigned long controltimestamp;
    long prev_count_leftenc;
    long prev_count_rightenc;
    float Kp;
    float Ki;
    float p_term;
    float integral;
    unsigned long prev_time;
    float error;

    void initialize() {
        p_term = 0.0;
        controltimestamp = 0;
        prev_count_leftenc = 0;
        prev_count_rightenc = 0;
        prev_time = 0;
        integral = 0.0;
        lpf_l = 0.0;
        lpf_r = 0.0;
        alpha = 0.7;
    }

    void rotationalspeed(unsigned long currentMillis) {
        int currentpidtime = currentMillis - controltimestamp;
        controltimestamp = currentMillis;

        // Calculate the change in encoder counts
        int change_leftenc = count_leftenc - prev_count_leftenc;
        int change_rightenc = count_rightenc - prev_count_rightenc;

        prev_count_leftenc = count_leftenc;
        prev_count_rightenc = count_rightenc;

        // Calculate the velocity of the left and right motors
        velocity_left = (change_leftenc * anglepercount) / (currentpidtime *0.001);
        velocity_right = (change_rightenc * anglepercount) / (currentpidtime *0.001);

        lpf_l = ( lpf_l * alpha  ) + ( velocity_left * ( 1.0 - alpha ) );
        lpf_r = ( lpf_r * alpha  ) + ( velocity_right * ( 1.0 - alpha ) );


       
        
    }

    void update(float demand, float measurement) {
        // Calculate elapsed time
        unsigned long current_time = millis();
        // Serial.print("current time= ");
        // Serial.print(current_time);
        // Serial.print(", prev time= ");
        // Serial.print(prev_time);
        int delta_t = (current_time - prev_time); // Convert to seconds

        // Serial.print("delta_t= ");
        // Serial.println(delta_t);

        // Serial.println(delta_t);
        prev_time = current_time;

        // Calculate error signal for left motor
        error = demand - measurement;
        // do PI control

        // Serial.println(error);

        float integral_change = error * delta_t;
        // Serial.println("integral change= ");
        // Serial.println(integral_change);


        integral += integral_change;
        

        p_term = (Kp * error) + (Ki * integral);

        
        // Serial.print(p_term);
        // Serial.print("=");
        // Serial.print(Kp);
        // Serial.print("*");
        // Serial.print(error);
        // Serial.print("+");
        // Serial.print(Ki);
        // Serial.print("*");
        // Serial.println(integral);



    }

    // Getters for the proportional terms
    float getProportionalTerm() {
        return p_term;
    }
};

#endif