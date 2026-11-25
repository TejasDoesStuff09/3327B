#include "snapper.hpp"
#include "vex.h"

//stores all recorded robot positions
std::vector<FieldPoint> savedWaypoints;

//retrieves the robot's current LemLib odometry position
FieldPoint getCurrentPosition() {

    FieldPoint currentPos;

    currentPos.x = chassis.getPose().x;
    currentPos.y = chassis.getPose().y;
    currentPos.heading = chassis.getPose().theta;

    return currentPos;
}

//handles waypoint recording and display through controller inputs
void snapperControl() {

    bool lastButtonState = false;
    bool lastPrintState = false;

    int displayIndex = 0;

    while (true) {

        bool currentButtonState = Controller1.ButtonA.pressing();

        if (currentButtonState && !lastButtonState) {

            //retrieves and saves the current robot position
            FieldPoint currentPos = getCurrentPosition();
            savedWaypoints.push_back(currentPos);

            //displays confirmation on the controller screen
            Controller1.Screen.clearLine(1);
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("Saved Pt #%d", savedWaypoints.size());

            //vibrates controller to confirm waypoint was saved
            Controller1.rumble(".");
        }

        lastButtonState = currentButtonState;

        bool currentPrintState = Controller1.ButtonX.pressing();

        if (currentPrintState && !lastPrintState) {

            //checks whether any waypoints have been saved
            if (savedWaypoints.size() > 0) {

                //retrieves the currently selected waypoint
                FieldPoint point = savedWaypoints[displayIndex];

                //displays waypoint number and coordinates
                Controller1.Screen.clearScreen();

                Controller1.Screen.setCursor(1, 1);
                Controller1.Screen.print("Point #%d", displayIndex + 1);

                Controller1.Screen.setCursor(2, 1);
                Controller1.Screen.print("X: %.1f", point.x);

                Controller1.Screen.setCursor(3, 1);
                Controller1.Screen.print("Y: %.1f", point.y);

                Controller1.Screen.setCursor(4, 1);
                Controller1.Screen.print("H: %.1f", point.heading);

                //advances to the next saved waypoint
                displayIndex++;

                //returns to the first waypoint after the last one
                if (displayIndex >= savedWaypoints.size()) {
                    displayIndex = 0;
                }
            }

            else {
                //notifies driver if no waypoints have been saved
                Controller1.Screen.clearScreen();
                Controller1.Screen.print("No Points Saved");
            }
        }

        lastPrintState = currentPrintState;

        // Standard 20 ms loop delay.
        wait(20, msec);
    }
}
