#ifndef pid_controller_h
#define pid_controller_h

class PID{

    public:
    
        // Common functions
        PID(uint32_t sample_time,
            double Kp,
            double Ki,
            double Kd,
            double min_output,
            double max_output,
            double initial_output);

        double Compute(double set_point, double measurement);

        bool SetTunings(double Kp);
        bool SetTunings(double Kp, double Ki);
        bool SetTunings(double Kp, double Ki, double Kd);

        void SetMinMaxOutput(double min_output, double max_output);
    
    private:
        // variables
        double kp;      // Proportional gain
        double ki;      // Integral gain
        double kd;      // Derivate gain

        double output_sum;   // Output ouput_sum
        double last_measurement; 
        double last_output;
        uint32_t last_time;   // Last output calculation
        uint32_t sample_time;
        double min_output;
        double max_output;
};

#endif