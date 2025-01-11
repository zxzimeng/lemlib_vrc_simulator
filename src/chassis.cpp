#include <math.h>
#include <cmath>
#include "util.hpp"
#include "chassis.hpp"
#include <future>
#include "main.h"
#include <stdexcept>

void lemlib::Chassis::setPose(lemlib::Pose pose, bool radians) {
    robot_pose=pose;
    robot_pose.theta=sanitizeAngle(robot_pose.theta, false);
    movements.emplace_back(robot_pose);
    printf("Set Point: x: %.3f y: %.3f theta: %.3f\n", pose.x, pose.y, sanitizeAngle(robot_pose.theta, false));
}

lemlib::Pose lemlib::Chassis::getPose(bool radians, bool standardPos) {
    robot_pose.theta=sanitizeAngle(robot_pose.theta, false);
    return robot_pose;
}

void lemlib::Chassis::turnToPoint(Pose pose, int timeout, TurnToPointParams params, bool async) {
    // Calculate the difference in x and y coordinates
    float deltaX = pose.x - robot_pose.x;
    float deltaY = pose.y - robot_pose.y;

    // Calculate the arrival heading in radians using atan2
    float gamma = std::atan2(deltaY, deltaX); // Gamma is the angle relative to the vertical axis

    // Convert the angle from radians to degrees
    float arrivalTheta = gamma * 180.0 / M_PI;

    // Adjust the angle so that 0 degrees is along the positive x-axis (standard Cartesian system)
    arrivalTheta = 90.0 - arrivalTheta;

    // Normalize the angle to the range [-180, 180] degrees (optional)
    if (arrivalTheta > 180.0) {
        arrivalTheta -= 360.0;
    } else if (arrivalTheta < -180.0) {
        arrivalTheta += 360.0;
    }
    robot_pose.theta = arrivalTheta;
    robot_pose.theta=sanitizeAngle(robot_pose.theta, false);
    movements.emplace_back(robot_pose);
    printf("Turned to Point: (x: %.3f y: %.3f) | Now at theta: %.3f\n", pose.x, pose.y, sanitizeAngle(arrivalTheta, false));
}

void lemlib::Chassis::turnToHeading(float theta, int timeout, TurnToHeadingParams params, bool async) {
    robot_pose.theta = theta;
    robot_pose.theta=sanitizeAngle(robot_pose.theta, false);
    movements.emplace_back(robot_pose);
    printf("Turned to Heading: %.3f\n", sanitizeAngle(theta, false));
}

void lemlib::Chassis::moveToPose(Pose pose, int timeout, MoveToPoseParams params, bool async) {
    robot_pose=pose;
    // robot_pose.y-=2;
    robot_pose.theta=sanitizeAngle(robot_pose.theta, false);
    movements.emplace_back(robot_pose);
    printf("Moved to Pose: x: %.3f y: %.3f theta: %.3f\n", pose.x, pose.y, sanitizeAngle(pose.theta, false));
}

void lemlib::moveToPoint(Pose pose, int timeout, MoveToPointParams params, bool async) {
    // robot_pose.theta = degToRad(robot_pose.theta);
    // pose.theta = degToRad(pose.theta);
    // float arrivalTheta = robot_pose.angle(pose);
    // arrivalTheta = -(90+radToDeg(arrivalTheta));
    // printf("%f", arrivalTheta);
    // // Simulate updating the robot's pose (directly set the target pose)
    // float dx = pose.x - robot_pose.x;
    // float dy = pose.y - robot_pose.y;
    //
    // // Calculate the angle to the target (relative to the positive x-axis)
    // float arrival_heading = std::atan2(dy, dx);  // Angle in radians
    //
    // // Update robot's position and orientation (theta)
    // robot_pose.x = pose.x;
    // robot_pose.y = pose.y;
    // robot_pose.theta = radToDeg(arrival_heading);
    //
    //

    robot_pose=pose;
    robot_pose.theta = sanitizeAngle(robot_pose.theta, false);

    movements.emplace_back(robot_pose);
    printf("Moved to Point: x: %.3f y: %.3f theta: %.3f\n", pose.x, pose.y, sanitizeAngle(robot_pose.theta, false));
}