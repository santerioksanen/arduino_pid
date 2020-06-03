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
}

void Odometry::Update(int32_t Revolution_count, double Steering_angle){
    // Perform update
    if(Revolution_count != revolution_count){
        if(Steering_angle == 0){
            // We're going straight ahead
            x += cos(orientation) * (Revolution_count-revolution_count) * rev_to_dist;
            y += sin(orientation) * (Revolution_count-revolution_count) * rev_to_dist;
        } else if(Steering_angle < 0){
            // We're turning right
        } else{
            // We're turning left
        }
    }
    
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