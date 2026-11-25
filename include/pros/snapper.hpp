#pragma once

#include <vector>

//stores the robot's x position, y position, and heading
struct FieldPoint {
    double x;
    double y;
    double heading;
};

//stores all recorded robot positions
extern std::vector<FieldPoint> savedWaypoints;

//retrieves the robot's current position
FieldPoint getCurrentPosition();

//handles waypoint recording and display through controller inputs
void snapperControl();
