#include "Arduino.h"
#include "pid_controller.h"
#define MS_TO_SEC 1000

/* Constructor 
*/
PID::PID(uint32_t sample_time,
        double Kp,
        double Ki,
        double Kd,
        double min_output, 
        double max_output,
        double initial_output) {
    PID::SetTunings(Kp, Ki, Kd);
    PID::SetMinMaxOutput(min_output, max_output);

    last_output = initial_output;
    output_sum = initial_output;
    sample_time = sample_time;
    last_measurement = 0;
    last_time = millis() - sample_time;
}

void PID::SetMinMaxOutput(double min_output, double max_output){
    min_output = min_output;
    max_output = max_output;
}

double PID::Compute(double set_point, double measurement){
    uint32_t now = millis();
    uint32_t time_change = (now - last_time);
    if(time_change < sample_time){
        return last_output;
    }
    else {
        double error = set_point - measurement;
        double d_measurement = measurement - last_measurement;

        output_sum += (error * ki * time_change / MS_TO_SEC);
        //output_sum = min(output_sum, max_output);   // Limit to max
        //output_sum = max(output_sum, min_output);    // Limit to min
        
        double output = error * kp;     // Proportional part
        output -= (d_measurement * MS_TO_SEC / time_change * kd); // Derivative part
        output += output_sum;           // Integral part
        
        //output = min(output, max_output);
        //output = max(output, min_output);
        
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