#include <math.h>
#include <stdint.h>
#include "odometry.h"

Odometry::Odometry(double X, double Y, double Orientation, double Wheelbase, double Diameter, double Max_steering_angle, double Tick_to_rot_ratio){
    wheelbase = Wheelbase;
    diameter = Diameter;
    rev_to_dist = Diameter * M_PI / Tick_to_rot_ratio;
    orientation = Orientation;
    x = X;
    y = Y;
    max_steering_angle = Max_steering_angle;
    revolution_count = 0;
    prev_steering_angle = 0;
}

void Odometry::Update(int32_t Revolution_count, double Steering_angle){
    // Perform update
    double steering_angle = Steering_angle; // For future to model slowness of turning of wheels
    double c_x, c_y, r, beta;

    // Update location
    if(Revolution_count != revolution_count){
        double dist = (Revolution_count-revolution_count) * rev_to_dist;
        double alfa = Steering_angle * max_steering_angle;
        if(steering_angle == 0){
            // We're going straight ahead
            x += cos(orientation) * dist;
            y += sin(orientation) * dist;
        } else {
            r = wheelbase/tan(alfa);        // turning radius, negative turns to right, positive to left
            c_x = x - r * sin(orientation); // x-coordinate of circle center
            c_y = y + r * cos(orientation); // y-coordinate of circle center

            beta = dist/r;         // How many radians we turned

            x = c_x + sin(orientation + beta) * r;
            y = c_y - cos(orientation + beta) * r;

            orientation += beta;
            while(orientation < 0){
                orientation += (2 * M_PI);
            }
            while(orientation >= (2*M_PI)){
                orientation -= (2*M_PI);
            }
        }
    }
    
    prev_steering_angle = steering_angle;
    revolution_count = Revolution_count;
}

double Odometry::GetX(){
    return x;
}

double Odometry::GetY(){
    return y;
}

double Odometry::GetOrientation(){
    return orientation;
}

double Odometry::GetDistance(){
    return revolution_count * rev_to_dist;
}