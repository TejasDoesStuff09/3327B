

#include "TrapezoidalProfile.hpp"
#include <cmath>

// Constructor: initializer list sets some members, body computes the rest
TrapezoidalProfile::TrapezoidalProfile(double distance, double maxVel, double maxAccel)
    : distance_(std::abs(distance)),
      maxAccel_(maxAccel),
      sign_(distance < 0 ? -1 : 1),
      peakDistance_(std::abs(distance)/2) //distance from initial position at which the robot reaches the highest speed
      
{
    //derived from kinematic equation V = sqrt(2a/\x)
    //triangle case
    if (maxVel>std::sqrt(2 * peakDistance_ * maxAccel_)) {
        peakVel_ = std::sqrt(2 * peakDistance_ * maxAccel_);
        accelTime_ = std::sqrt(2 * peakDistance_ / maxAccel_);
        cruiseTime_ = 0;
        totalTime_ = 2*accelTime_ + cruiseTime_;
    }
    //trapezoid case
    else {
        peakVel_ = maxVel;
        accelDistance_ = (peakVel_ * peakVel_) / (2 * maxAccel_);
        accelTime_ = std::sqrt(2 * accelDistance_ / maxAccel_);
        cruiseTime_ = (distance_ - 2 * accelDistance_) / peakVel_;
        totalTime_ = 2 * accelTime_ + cruiseTime_;
    }
}

// Note the return type comes first, then ClassName::, and const stays at the end
State TrapezoidalProfile::trapezoidState(double t) const {
    /* returns a sturct of position, velocity, and acceleration for a time t according to the trapezoidal profile
    */
    // initialize pos, velocity, acceleration in a State
    State s{0.0, 0.0, 0.0};  
    //catch out of bound times
    if (t>totalTime_) { return State {distance_*sign_, 0.0, 0.0}; }
    if (t<0) { t = 0;}
    //assume acceleration is either a constant positive value or a constant negative value, or zero,  depending on profile phase
    if (t<accelTime_) {
        //acceleration phase 
        s.acceleration = maxAccel_; 
        s.velocity = maxAccel_ * t; 
        s.pos = 0.5 * maxAccel_ * t * t; //0.5at^2 + vt
    }
    else if (t>(accelTime_ + cruiseTime_)) {
        //past halfway point(deceleration)
        double currentDecelTime = t-accelTime_ - cruiseTime_; //segment of time after acceleration and cruise has ended
        s.acceleration = -1 * maxAccel_;
        s.velocity = peakVel_ - maxAccel_ * (currentDecelTime);  // v + at
        s.pos = 0.5 * maxAccel_ * (accelTime_) * (accelTime_) + (peakVel_*cruiseTime_) + (peakVel_*(currentDecelTime) + (-0.5 * maxAccel_ * (currentDecelTime) * (currentDecelTime))); //0.5at^2 + vt, but for three segments where acceleration changes(accel, cruise, deccel)
    }
    else {
        //cruising
        s.acceleration = 0;
        s.velocity  = peakVel_; 
        s.pos = (0.5 * maxAccel_ * accelTime_ * accelTime_) + peakVel_ * (t - accelTime_); //0.5at^2 + vt for two segments
    }
    //apply signage 
    s.pos *= sign_;          
    s.velocity *= sign_;
    s.acceleration *= sign_;
    return s;
}