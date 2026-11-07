#pragma once

struct State {
    double pos;
    double velocity;
    double acceleration;
};

class TrapezoidalProfile  {
    //REMIDNER TO PUT COMMENTS HERE
    public:
        TrapezoidalProfile(double distance, double maxVel, double maxAccel); 
        State trapezoidState(double t) const; //explain this goofy ash syntax
        double duration() const { return totalTime_; }

    private:
        double distance_, peakDistance_, maxAccel_, peakVel_, accelDistance_;
        double accelTime_, cruiseTime_, totalTime_;
        int sign_;

};