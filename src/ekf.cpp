#include "ekf.hpp"

std::vector<Segment> constructPolygon(const std::vector<Point>& corners) {
    int numCorners = corners.size();
    if (numCorners<3) {
        return {};
    }
    std::vector<Segment> polygon;
    for (int i=0; i<numCorners; i++) {
        int nextIndex = (i+1)%numCorners;//so that we can connect the last point to the first point
        Segment s = {corners[i].x,corners[i].y,corners[nextIndex].x,corners[nextIndex].y}; //access two adjacent points and construct segment
        polygon.push_back(s); //add segment to the polygon
    }
    return polygon;
}

std::vector<Segment> constructFieldMap(double sensorHeight) {
    
    if (sensorHeight>8.77) {
        return constructPolygon({{-70.2,70.2},{70.2,70.2},{70.2,-70.2},{-70.2,-70.2}}); //wall and loaders only
        //use only the points at the edges of the field
        // see the field diagrams in the vex manual for field/coordinate orientation
        //REMINDER TO MEASURE THAT PART OF THE FIELD
    }
    else {
        //including middle goal
    }
}

const Pose projectToField(const Pose& s,const double theta) {
    //rotates a vector to field coordinates. Helper method to figure out where sensors are.
    const double halfPI = std::numbers::pi/2.0;
    Pose r = {0,0,s.theta + theta};
    r.x = s.x*cosine(theta)+s.y*cosine(halfPI+theta);
    r.y = s.x*sine(theta) + s.y*sine(halfPI+theta);   
    return r;
     

}
// ---------------------------------------------------------------------------------------------
// (refer to header file)

namespace {   // (names inside this block are only visible inside this file)
// [CLAUDE] the real sensor can only see between these distances (VEX spec: 20 mm to 2000 mm)
const double kMinRange = 0.79;
const double kMaxRange = 78.74;
}

std::optional<double> expectedDistance(const Pose& p, const SensorModel& s) {
    //should return the expected reading h(x) given the position of the bot, x, for a certain sensor.
    Pose sensor = {s.pos.a,s.pos.b,s.pos.alpha};
    Pose sensorPose = p + projectToField(sensor,p.theta); //position of sensor w.r.t the field
    // sensorPose.x and .y are the sensor's position. sensorPose.theta is the direction the beam points.

   

    // The beam is a ray: (sensor position) + t * (direction), with t >= 0.
    // direction has length 1, so t is the distance in inches.
    double dirX = cosine(sensorPose.theta);
    double dirY = sine(sensorPose.theta);

    // Check the ray against every segment and keep the closest hit.
    bool foundHit = false;
    double nearest = 0.0;

    for (const Segment& seg : s.map) {
        // the segment is: (start) + u * (edge), with u from 0 (start) to 1 (end)
        double edgeX = seg.x2 - seg.x1;
        double edgeY = seg.y2 - seg.y1;

        // The ray and segment meet where  sensor + t*dir = start + u*edge.
        // That is two equations (x and y) with two unknowns (t and u).
        // "cross(a, b)" below means a.x*b.y - a.y*b.x. Solving the pair of equations
        // with it gives:  t = cross(w, edge) / cross(dir, edge)
        //                 u = cross(w, dir)  / cross(dir, edge)
        // where w is the arrow from the sensor to the segment's start.
        double denominator = dirX*edgeY - dirY*edgeX;      // cross(dir, edge)
        if (std::abs(denominator) < 1e-9) {
            continue;   // the ray runs parallel to this segment, so it can't hit it
        }

        double wX = seg.x1 - sensorPose.x;
        double wY = seg.y1 - sensorPose.y;
        double t = (wX*edgeY - wY*edgeX) / denominator;    // distance along the ray
        double u = (wX*dirY - wY*dirX) / denominator;      // how far along the segment

        // a real hit is ahead of the sensor (t >= 0) and lands ON the segment (u between 0 and 1)
        if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
            if (!foundHit || t < nearest) {
                foundHit = true;
                nearest = t;
            }
        }
    }

    // The real sensor can only see between kMinRange and kMaxRange.
    if (!foundHit || nearest < kMinRange || nearest > kMaxRange) {
        return std::nullopt;
    }
    return nearest;
}


// ===========================================================================
// NUMBERS WE MAY WANT TO CHANGE
// Everything below uses these names, so change them here and nowhere else.
// Numbers marked GUESS were not measured. Tune them on the robot.
// ===========================================================================
namespace {

const double kPi = std::numbers::pi;

// --- center goal, read off the manual's goal drawing (approximate) ---
const double kGoalTopHeight = 8.77;      // above this height the sensor beam passes over the goal
const double kGoalBoxHeight = 5.52;      // below this height the goal is a straight-sided box
const double kGoalBaseWidth = 5.61;      // width of the box part
const double kGoalTopWidth = 3.49;       // width at the very top

// --- distance sensor noise (inches) ---
const double kSensorSigmaPerInch = 0.025;   // GUESS: VEX says about 5% error, we treat that as about 2 sigma
const double kSensorSigmaMin = 0.6;      // VEX says +/-15 mm up close
const double kMapSigma = 0.58;           // the field map itself is only good to about +/-1 in

// --- odometry noise (the motion model) ---
const double kPosSigmaPerInch = 0.02;    // GUESS: 2% of the distance driven
const double kPosSigmaFloor = 0.01;      // GUESS: small noise even when not moving (inches)
const double kHeadingSigmaPerRad = 0.01; // GUESS: 1% of the angle turned
const double kHeadingSigmaFloor = 0.0005; // GUESS: slow IMU drift per step (radians)

// --- outlier gating ---
// If (reading - expected)^2 / S is bigger than this, the reading is thrown away.
// 6.63 means "less than a 1% chance if the filter is right" (chi-square, 1 degree of freedom).
const double kGateThreshold = 6.63;

// --- finite differences ---
const double kNudge = 1e-4;              // how far we nudge x, y, theta to find H

}


// ===========================================================================
// 5. [CLAUDE] MAP HELPERS
// ===========================================================================

// The center goal as a square, as wide as the real goal is at the sensor's height.
// Below kGoalBoxHeight it is a box. Above that it narrows in a straight line to the top.
static std::vector<Segment> centerGoalAtHeight(double sensorHeight) {
    double width = kGoalBaseWidth;
    if (sensorHeight > kGoalBoxHeight) {
        double fractionUp = (sensorHeight - kGoalBoxHeight) / (kGoalTopHeight - kGoalBoxHeight);
        width = kGoalBaseWidth - (kGoalBaseWidth - kGoalTopWidth) * fractionUp;
    }
    double half = width / 2.0;
    // the center goal sits at the center of the field (0, 0)
    return constructPolygon({{-half, half}, {half, half}, {half, -half}, {-half, -half}});
}

// TODO: PUT YOUR MEASURED LOADER CORNERS HERE.
// Each loader is a polygon: list its corners IN ORDER around the shape, in field
// coordinates (origin at the field center), measured at your sensor height. Example for ONE loader:
//
//     std::vector<Segment> loader1 = constructPolygon({{-70.2, 20.0}, {-66.4, 20.0}, {-66.4, 24.0}, {-70.2, 24.0}});
//     loaders.insert(loaders.end(), loader1.begin(), loader1.end());
//
// Until you fill this in, the map has NO loaders.
static std::vector<Segment> loaderSegments() {
    std::vector<Segment> loaders;
    return loaders;
}


// ===========================================================================
// 6. [CLAUDE] p(z|x): HOW NOISY THE REAL READING IS AROUND h(x)
//    z ~ bell curve with mean h(x) and variance R. This returns R.
// ===========================================================================

double measurementNoise(double expectedInches) {
    // the sensor's own noise grows with distance
    double sensorSigma = kSensorSigmaPerInch * expectedInches;
    if (sensorSigma < kSensorSigmaMin) {
        sensorSigma = kSensorSigmaMin;
    }
    // the map's error is a separate source of error, so the VARIANCES add
    return sensorSigma*sensorSigma + kMapSigma*kMapSigma;
}


// ===========================================================================
// 7. [CLAUDE] THE MOTION MODEL  p(x_t | x_t-1, u_t)
// ===========================================================================

double wrapAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

Matrix3 diagonalMatrix3(double a, double b, double c) {
    Matrix3 result = {{{a, 0, 0}, {0, b, 0}, {0, 0, c}}};
    return result;
}

// Where is the robot after moving u? We assume it drove straight along the AVERAGE of its
// starting and ending heading (this is what makes a turning move come out right).
Pose predictPose(const Pose& x, const Motion& u) {
    double midHeading = x.theta + u.dTheta / 2.0;
    Pose next;
    next.x = x.x + u.d * cosine(midHeading);
    next.y = x.y + u.d * sine(midHeading);
    next.theta = wrapAngle(x.theta + u.dTheta);
    return next;
}

// F: how the NEW pose changes if the OLD pose is nudged. Row 1 is x, row 2 is y, row 3 is theta.
// Only the heading matters for x and y, because the heading decides which way we drove.
Matrix3 motionJacobian(const Pose& x, const Motion& u) {
    double midHeading = x.theta + u.dTheta / 2.0;
    Matrix3 F = {{{1, 0, -u.d * sine(midHeading)},
                  {0, 1,  u.d * cosine(midHeading)},
                  {0, 0, 1}}};
    return F;
}

// Q: the uncertainty the move itself adds. More driving means more slip, so more noise.
// (Simplified: the noise is the same in x and y, with no correlation between them.)
Matrix3 processNoise(const Motion& u) {
    double posSigma = kPosSigmaPerInch * std::abs(u.d) + kPosSigmaFloor;
    double headingSigma = kHeadingSigmaPerRad * std::abs(u.dTheta) + kHeadingSigmaFloor;
    return diagonalMatrix3(posSigma*posSigma, posSigma*posSigma, headingSigma*headingSigma);
}


// ===========================================================================
// 8. [CLAUDE] H, THE SLOPE OF h(x)
//    The update needs to know how the expected reading changes if x, y, or theta is
//    off a little. We find each slope by nudging one thing up and down and re-measuring.
// ===========================================================================

// expectedDistance after nudging one of (x, y, theta). axis: 0 = x, 1 = y, 2 = theta.
static std::optional<double> expectedDistanceNudged(const Pose& x, const SensorModel& sensor, int axis, double amount) {
    Pose nudged = x;
    if (axis == 0) {
        nudged.x += amount;
    } else if (axis == 1) {
        nudged.y += amount;
    } else {
        nudged.theta += amount;
    }
    return expectedDistance(nudged, sensor);
}

bool measurementJacobian(const Pose& x, const SensorModel& sensor, double H[3]) {
    for (int axis = 0; axis < 3; axis++) {
        std::optional<double> plus = expectedDistanceNudged(x, sensor, axis, kNudge);
        std::optional<double> minus = expectedDistanceNudged(x, sensor, axis, -kNudge);
        if (!plus || !minus) {
            return false;   // a nudge made the reading disappear, so we can't trust this slope
        }
        H[axis] = (*plus - *minus) / (2.0 * kNudge);
    }
    return true;
}


// ===========================================================================
// 9. [CLAUDE] THE FILTER
// ===========================================================================

// tiny 3x3 matrix helpers, written out with plain loops
static Matrix3 multiply(const Matrix3& a, const Matrix3& b) {
    Matrix3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double sum = 0.0;
            for (int k = 0; k < 3; k++) {
                sum += a.m[i][k] * b.m[k][j];
            }
            result.m[i][j] = sum;
        }
    }
    return result;
}

static Matrix3 transpose(const Matrix3& a) {
    Matrix3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = a.m[j][i];
        }
    }
    return result;
}

static Matrix3 add(const Matrix3& a, const Matrix3& b) {
    Matrix3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = a.m[i][j] + b.m[i][j];
        }
    }
    return result;
}

Ekf::Ekf(const Pose& startPose, const Matrix3& startCovariance, const std::vector<SensorModel>& sensors)
    : x_(startPose), P_(startCovariance), sensors_(sensors) {
    // (the members are listed in the same order they are declared in the class)
}

// The motion step. The belief moves with the robot, and gets less certain.
void Ekf::predict(const Motion& u) {
    // F has to be found at the OLD pose, so build it before moving x_
    Matrix3 F = motionJacobian(x_, u);
    Matrix3 Q = processNoise(u);

    x_ = predictPose(x_, u);

    // new P = F * P * F^T + Q    (the old uncertainty, carried through the move, plus new noise)
    P_ = add(multiply(multiply(F, P_), transpose(F)), Q);
}

// The measurement step for one sensor. This is Bayes' rule for a bell-curve belief:
// prior belief (from predict) combined with p(z|x) (from the reading).
bool Ekf::update(int sensorIndex, double readingInches) {
    const SensorModel& sensor = sensors_[sensorIndex];

    // A reading outside the sensor's range doesn't mean anything.
    if (readingInches < kMinRange || readingInches > kMaxRange) {
        return false;
    }

    // h(x): what we EXPECT this sensor to read from where we think we are.
    std::optional<double> expected = expectedDistance(x_, sensor);
    if (!expected) {
        return false;
    }

    // H: how h(x) changes if x, y, or theta is a little off.
    double H[3];
    if (!measurementJacobian(x_, sensor, H)) {
        return false;
    }

    // R: the variance of p(z|x), how noisy the reading is.
    double R = measurementNoise(*expected);

    // the innovation: how far the real reading is from what we expected
    double innovation = readingInches - *expected;

    // P * H^T  (a list of 3 numbers)
    double PHt[3];
    for (int i = 0; i < 3; i++) {
        PHt[i] = 0.0;
        for (int j = 0; j < 3; j++) {
            PHt[i] += P_.m[i][j] * H[j];
        }
    }

    // S = H * P * H^T + R : the total uncertainty of the innovation
    // (uncertainty from where we think we are, plus the sensor's own noise)
    double S = R;
    for (int i = 0; i < 3; i++) {
        S += H[i] * PHt[i];
    }

    // Gating: if the reading is way off from what we expected, something is in the way
    // (another robot, a game piece). Throw it out instead of letting it pull us off.
    if (innovation * innovation / S > kGateThreshold) {
        return false;
    }

    // K: the gain. How much of the innovation we move by, for each of x, y, theta.
    // Sure of our position (small P) and noisy sensor (big R) means K is small.
    double K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = PHt[i] / S;
    }

    // update the best guess
    x_.x += K[0] * innovation;
    x_.y += K[1] * innovation;
    x_.theta = wrapAngle(x_.theta + K[2] * innovation);

    // update the uncertainty (the "Joseph form", which keeps P well-behaved):
    //     P = (I - K*H) * P * (I - K*H)^T + R * K * K^T
    Matrix3 A;   // A = I - K*H
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double identity = (i == j) ? 1.0 : 0.0;
            A.m[i][j] = identity - K[i] * H[j];
        }
    }
    Matrix3 sensorPart;   // R * K * K^T
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            sensorPart.m[i][j] = R * K[i] * K[j];
        }
    }
    P_ = add(multiply(multiply(A, P_), transpose(A)), sensorPart);

    return true;
}

Pose Ekf::pose() const {
    return x_;
}

Matrix3 Ekf::covariance() const {
    return P_;
}


// ===========================================================================
// 10. [CLAUDE] HARDWARE CONVERSIONS
// ===========================================================================

double mmToInches(double millimeters) {
    return millimeters / 25.4;
}

double wheelRotationToInches(double rotationDegrees, double wheelDiameterInches) {
    // one full turn (360 degrees) rolls the wheel one circumference
    return rotationDegrees / 360.0 * kPi * wheelDiameterInches;
}

double forwardDistanceFromWheel(double wheelInches, double wheelOffsetLeft, double dTheta) {
    // A wheel off to the LEFT of the center moves backward when the robot turns left (dTheta > 0),
    // so its reading is short by wheelOffsetLeft * dTheta. Add that back.
    // (A wheel on the right has a negative offset, and the same formula works.)
    return wheelInches + wheelOffsetLeft * dTheta;
}

double headingToTheta(double headingDegrees) {
    // heading 0 faces the 0 degree wall (+y), and heading grows clockwise.
    // theta 0 faces +x, and theta grows counter-clockwise. So: theta = 90 degrees - heading.
    double thetaDegrees = 90.0 - headingDegrees;
    return wrapAngle(thetaDegrees * kPi / 180.0);
}

double thetaToHeading(double thetaRadians) {
    double headingDegrees = 90.0 - thetaRadians * 180.0 / kPi;
    while (headingDegrees < 0.0) {
        headingDegrees += 360.0;
    }
    while (headingDegrees >= 360.0) {
        headingDegrees -= 360.0;
    }
    return headingDegrees;
}

double imuTurnToDTheta(double turnDegreesClockwise) {
    // clockwise is the opposite direction from our theta, so the sign flips
    return -turnDegreesClockwise * kPi / 180.0;
}



