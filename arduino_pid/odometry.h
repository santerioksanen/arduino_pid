class Odometry{
    public:
        Odometry(double X, double Y, double Orientation, double Wheelbase, double Diameter, double Max_steering_angle, double Tick_to_rot_ratio);
        void Update(int32_t Revolution_count, double Steering_angle);
        double GetX();
        double GetY();
        double GetDistance();
        double GetOrientation();
    
    private:
        int32_t revolution_count;
        double wheelbase, diameter, orientation, x, y, rev_to_dist, max_steering_angle, prev_steering_angle;
};