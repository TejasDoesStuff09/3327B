#pragma once
#include "pros/motors.hpp"

class Lift{

  private:
    //I'm storing references to both lift motors so the class can control the motors that are already set up in the robot code instead of creating new motors.
    pros::Motor& leftMotor;
    pros::Motor& rightMotor;

  public:
    //This constructor takes in the two motors so I can connect them to the lift when I create the Lift object
    Lift(pros::Motor& leftMotor, pros::Motor& rightMotor);

    //it takes a number of inches to move the lift. A positive number means up and a negative number means down.
    void moveInches(double inches);


};
