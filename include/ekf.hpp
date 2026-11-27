#pragma once
#include <vector>
#include <optional>
#include <cmath>

// ---------------------------------------------------------------------------
// Conventions
//   Units:           all lengths in inches, all angles in radians.
//   Field frame:     origin at the center of the field; the walls sit at about +/-70.2 in.
//                    +x points right and +y points up (toward the 0 degree wall) in the
//                    audience-view field drawings.
//   Pose.theta:      0 points along +x, positive is counter-clockwise.
//   Robot frame:     origin at the bot center; a = forward, b = left.
//   SensorPos.alpha: the sensor's facing angle relative to the robot's heading
//                    (counter-clockwise, 0 = straight ahead).
//   SensorPos.h:     height of the sensor above the floor.
// ---------------------------------------------------------------------------

// inches, radians, since std::cos and std::sin radians.
struct Pose {
    double x, y, theta;
    Pose operator+(const Pose& other) const {   // adds x and y coords; takes theta from the right-hand side (other)
        return {x + other.x, y + other.y, other.theta};
    }
};

// using coordinate segments to construct the field.
struct Segment {
    double x1, y1, x2, y2;
};

// sensor offsets, measured from bot center.
struct SensorPos {
    double a, b, alpha, h;
};

// in order to input corners for the polygon function.
struct Point {
    double x, y;
};

struct SensorModel {
    SensorPos pos;
    std::vector<Segment> map;  // a particular sensor's internal map of the field at their height.
};

// helper functions, to construct segments for the map:

// helper function to construct a polygon from its corners.
// will be helpful in constructing maps of the goals.
// corners must be listed in order around the shape (either direction)
// returns empty list if fewer than three corners are given
std::vector<Segment> constructPolygon(const std::vector<Point>& corners);

// function that returns a list of segments. That list of segments is the field map.
std::vector<Segment> constructFieldMap(double sensorHeight);

// EKF only needs p(z|x), where z is the sensor's noisy reading and x is a position.
// This computes h(x), the theoretic reading at position x.
// We will later fit a noisy distribution around h(x) to compute p(z|x)
//   inside: return std::nullopt;  when nothing is in range
//           return best;          when there's a hit
std::optional<double> expectedDistance(const Pose& p, const SensorModel& sensor);

//trig helper methods so there's less clutter
inline const double cosine(const double theta) {
    return std::cos(theta);
}
inline const double sine(const double theta) {
    return std::sin(theta);
}

//----------------------------------------------------------------------------------------------------------
//EKF STUFF
//----------------------------------------------------------------------------------------------------------

//3x3 matrix used for covariance P (how unsure we are about x and y)
struct Matrix3 {
    double m[3][3]; 
};

// function to create a 3x3 matrix with a,b,c down diagonal and zeros everywhere else
Matrix3 diagonalMatrix(double a,double b,double c);

// u_t: what the odometry says the robot did since the last timestep
struct Motion {
    double d; //signed distance the robot's vertical tracking wheel moved along the timestep
    double dTheta;  //change in heading along the timestep, from imu
};

//wraps an angle into the range (-pi,pi)
double wrapAngle(double angle);

// ---------------------------------------
// p(z|x): measurement model
//  Real reading z is approximated as a bell curve centered on h(x). 
//  This function returns variance R, of the bell curve
//  It is modeled on the sensor's range as well as map error itself(field measurement tolerance)

double measurmentNoise(double expectedInches);

//---------------------------------------------------------------------------
// p(x_t | x_t-1, u_t): the motion model
//   predictPose:    where we think the robot is after moving u from pose x
//   motionJacobian: F, how a small error in x gets carried through the move
//   processNoise:   Q, extra uncertainty added by the move itself (wheel slip, drift)
// ---------------------------------------------------------------------------
Pose predictPose(const Pose& x, const Motion& u);
Matrix3 motionJacobian(const Pose& x, const Motion& u);
Matrix3 processNoise(const Motion& u);
// ---------------------------------------------------------------------------
// H: how much the expected reading h(x) changes when x, y, or theta is nudged.
//   Found by nudging each one slightly and re-running expectedDistance.
//   Returns false if a nudged pose has no valid reading (then skip the update).
// ---------------------------------------------------------------------------
bool measurementJacobian(const Pose& x, const SensorModel& sensor, double H[3]);

// ---------------------------------------------------------------------------
// The filter itself. It holds the belief: the pose x_ (best guess) and the
// covariance P_ (how unsure we are).
//
// On the robot, every loop:
//     ekf.predict({d, dTheta});                          // from the wheels and IMU
//     ekf.update(0, mmToInches(frontSensor.get()));      // once per sensor reading
// ---------------------------------------------------------------------------
class Ekf {
        public: 
        // startPose: where we place the robot. bel(x_theta), formally. 
        // startCovariance; how sure you are of that initial position
        // The sensors and their maps are copied once here so the EKF owns them
        Ekf(const Pose& startPose, const Matrix3& startCovariance, const std::vector<SensorModel>& sensors);

        // the motion step. Moves the beleif forward using odometry
        void predict(const Motion& u);

        // the measurement step for one sensor. Returns true if reading was used, 
        // false if it was skipped(no valid expected rading, out of range, or gated out)
        bool update(int sensorIndex, double readingInches);
        
        //
        Pose pose() const;
        Matrix3 covariance() const;
    private: 
        Pose x_;
        Matrix3 P_;
        std::vector<SensorModel> sensors_;
};

// ---------------------------------------------------------------------------
// Hardware conversions (the glue between PROS numbers and the units above)
// ---------------------------------------------------------------------------
double mmToInches(double millimeters);

// distance a tracking wheel rolled, from its rotation in degrees
double wheelRotationToInches(double rotationDegrees, double wheelDiameterInches);

// distance the robot CENTER moved forward, from a vertical tracking wheel.
//   wheelOffsetLeft: how far the wheel sits to the left of the robot center (negative if right).
//   A wheel off the center rolls extra when the robot turns; this removes that.
double forwardDistanceFromWheel(double wheelInches, double wheelOffsetLeft, double dTheta);

// IMU heading (degrees, clockwise, 0 = facing the 0 degree wall) -> our theta (radians)
double headingToTheta(double headingDegrees);
// our theta (radians) -> IMU-style heading in degrees, 0 to 360
double thetaToHeading(double thetaRadians);
// change in IMU turn (degrees, clockwise = positive) -> our dTheta (radians, counter-clockwise = positive)
double imuTurnToDTheta(double turnDegreesClockwise);


//trig helper methods so there's less clutter
inline const double cosine(const double theta) {
    return std::cos(theta);
}
inline const double sine(const double theta) {
    return std::sin(theta);
}

