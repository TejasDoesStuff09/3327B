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
std::optional<double> expectedDistance(const Pose& p, const SensorModel& s) {
    //should return the expected reading h(x) given the position of the bot, x, for a certain sensor.
    Pose sensor = {s.pos.a,s.pos.b,s.pos.alpha};
    Pose sensorPose = p + projectToField(sensor,p.theta); //position of sensor w.r.t the field
    //raycasting
    
}


