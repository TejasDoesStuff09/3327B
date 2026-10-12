namespace {

    //Im using pi to convert between degrees and radians because the trig functions in C++ use radians
    constexpr double pi=3.14159265358979323846;
    //These are placeholders for now because we don't have the lift yet.
    constexpr double armlength =12.0;
    constexpr double stages=2.0;
    // This is the ratio I found on google for the motor's encoder rotation and the lift arm's rotation. We need to calculate this using the gearing and linkage of our lift.
    constexpr double motorToArmRatip=1.0;
    //This is the angle of the lift arm when the motor encoder reads zero. I also found this on Google. We need to measure this so the program knows the lift's starting position.
    constexpr double starting angle=30.0;

  // im connecting the two motor references to the Lift object so the methods in this class can use both the motors
  Lift::Lift(pros::Motor& leftMotor, pros::Motor& rightMotor)
      : leftMotor(left), rightMotor(right) {
  }

  //method to move the lift specified number of inches
  void Lift::moveInches(double inches){
    //we need to determine the relationship between motor encoder rotation and lift height before completing this method
  }

 };
