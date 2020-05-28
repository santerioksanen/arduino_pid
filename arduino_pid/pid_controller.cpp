#ifndef TEST
#include "Arduino.h"
#else
#include <stdint.h>
#endif
#include "pid_controller.h"
#define MS_TO_SEC 1000

#ifdef TEST
uint32_t millis_counter = 0;
uint32_t millis() {
    return millis_counter;
}
uint32_t increment_millis_by(uint32_t inc){
    millis_counter += inc;
}
#endif

/* Constructor 
*/
PID::PID(uint32_t Sample_time,
        double Kp,
        double Ki,
        double Kd,
        double Min_output, 
        double Max_output,
        double Initial_output) {
    PID::SetTunings(Kp, Ki, Kd);
    PID::SetMinMaxOutput(Min_output, Max_output);

    last_output = Initial_output;
    output_sum = Initial_output;
    sample_time = Sample_time;
    last_measurement = 0;
    
    last_time = millis();
}

void PID::SetMinMaxOutput(double Min_output, double Max_output){
    min_output = Min_output;
    max_output = Max_output;
}

double PID::Compute(double set_point, double measurement){

    // Mock millis if testing
    uint32_t now = millis();
    
    uint32_t time_change = (now - last_time);
    if(time_change < sample_time){
        return last_output;
    }
    else {
        double error = set_point - measurement;
        double d_measurement = measurement - last_measurement;

        output_sum += (error * ki * time_change / MS_TO_SEC);
        if(output_sum > max_output){
            output_sum = max_output;
        } if(output_sum < min_output){
            output_sum = min_output;
        }
        
        double output = error * kp;     // Proportional part
        output -= (d_measurement * MS_TO_SEC / time_change * kd); // Derivative part
        output += output_sum;           // Integral part
        
        if(output > max_output){
            output = max_output;
        } if(output < min_output){
            output = min_output;
        }
        
        last_time = now;
        last_measurement = measurement;
        last_output = output;
        return output;
    }
}

/*
    Set tuning (Kp, Ki, Kd)
    Ki and Kd are optional, defaulting to zero
*/
bool PID::SetTunings(double Kp){
    return PID::SetTunings(Kp, 0, 0);
}

bool PID::SetTunings(double Kp, double Ki){
    return PID::SetTunings(Kp, Ki, 0);
}

bool PID::SetTunings(double Kp, double Ki, double Kd){
    if(Kp < 0 || Ki < 0 || Kd < 0) return false;

    ki = Ki;
    kp = Kp;
    kd = Kd;
}