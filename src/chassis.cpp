#include <math.h>
#include <cmath>
#include "util.hpp"
#include "chassis.hpp"
#include <future>
#include "main.h"
#include <stdexcept>

void lemlib::Chassis::setPose(lemlib::Pose pose, bool radians) {
    robot_pose=pose;
}

lemlib::Pose lemlib::Chassis::getPose(bool radians, bool standardPos) {
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
}

void lemlib::Chassis::turnToHeading(float theta, int timeout, TurnToHeadingParams params, bool async) {
    robot_pose.theta = theta;
}

void lemlib::Chassis::moveToPose(Pose pose, int timeout, MoveToPoseParams params, bool async) {
    robot_pose=pose;
}

void lemlib::moveToPoint(Pose pose, int timeout, MoveToPointParams params, bool async) {
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

    // Simulate updating the robot's pose (directly set the target pose)
    robot_pose.x = pose.x;
    robot_pose.y = pose.y;
    robot_pose.theta = arrivalTheta;
}