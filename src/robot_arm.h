#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "enviro.h"
#include <sstream>
#include <cmath>

using namespace enviro;

class RobotArm : public Agent {
public:
    RobotArm(json spec, World& world) 
        : Agent(spec, world), joint1_angle(0), joint2_angle(0), joint3_angle(0) {
    }

    void init() {
        watch("rotate_joint1", [this](Event &e) {
            double theta = e.value()["theta"];
            joint1_angle = theta;
        });
        watch("rotate_joint2", [this](Event &e) {
            double theta = e.value()["theta"];
            joint2_angle = theta;
        });
        watch("rotate_joint3", [this](Event &e) {
            double theta = e.value()["theta"];
            joint3_angle = theta;
        });
    }

    void update() {
        double L1 = 50.0; 
        double L2 = 50.0; 
        double L3 = 50.0;

        // First link end point.
        double x1 = L1 * cos(joint1_angle);
        double y1 = L1 * sin(joint1_angle);

        // Second link end point.
        double total_angle1 = joint1_angle + joint2_angle;
        double x2 = x1 + L2 * cos(total_angle1);
        double y2 = y1 + L2 * sin(total_angle1);

        // Third link (end-effector) position.
        double total_angle2 = total_angle1 + joint3_angle;
        double x3 = x2 + L3 * cos(total_angle2);
        double y3 = y2 + L3 * sin(total_angle2);

        std::ostringstream oss;
        // Draw first link.
        oss << "<line x1='0' y1='0' x2='" << x1 << "' y2='" << y1 
            << "' style='stroke: black; stroke-width: 5' />";
        // Draw second link.
        oss << "<line x1='" << x1 << "' y1='" << y1 << "' x2='" << x2 
            << "' y2='" << y2 << "' style='stroke: black; stroke-width: 5' />";
        // Draw third link.
        oss << "<line x1='" << x2 << "' y1='" << y2 << "' x2='" << x3 
            << "' y2='" << y3 << "' style='stroke: black; stroke-width: 5' />";
        
        // Draw joints (base, first joint, second joint, and end-effector).
        oss << "<circle cx='0' cy='0' r='5' style='fill: blue' />";
        oss << "<circle cx='" << x1 << "' cy='" << y1 << "' r='5' style='fill: blue' />";
        oss << "<circle cx='" << x2 << "' cy='" << y2 << "' r='5' style='fill: blue' />";
        oss << "<circle cx='" << x3 << "' cy='" << y3 << "' r='5' style='fill: blue' />";

        decorate(oss.str());
        
        emit(Event("head_pos", json{{"x", x3}, {"y", y3}}));
    }

    void stop() {}

private:
    double joint1_angle;
    double joint2_angle;
    double joint3_angle;
};

DECLARE_INTERFACE(RobotArm)

#endif
