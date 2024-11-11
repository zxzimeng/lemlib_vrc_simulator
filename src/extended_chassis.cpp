#include "main.h"
#include "extended_chassis.hpp"

#include <stdexcept>

namespace lemlib {
void ExtendedChassis::moveToPoseWithEarlyExit(Pose pose, float timeout, MoveToPoseParams params, float exit_distance,
                                              bool async , bool degrees ) {
    moveToPose(pose, timeout, params, false);
    return;
}

void ExtendedChassis::moveToPointWithEarlyExit(Pose pose, float timeout, MoveToPointParams params, float exit_distance,
                                               bool async ) {
    if (exit_distance < 0) {
        throw std::out_of_range("Exit distance must be non-negative");
    }
    // float expected_distance = getPose(false).distance(pose);
    Pose pose_n = calculatePoseWithOffsetInDirection(pose, -exit_distance, false);
    moveToPoint(pose_n, timeout, params, false);
    // waitUntil(expected_distance);
    // cancelMotion();
    return;
}

void ExtendedChassis::processMovement(movement movement_s,
                                      transform_across_field transformation ) {
    movement transformed_movement = transformMovement(calculateOffset(movement_s), transformation);
    if (std::holds_alternative<MoveToPoseParams>(transformed_movement.moveParams)) {
        MoveToPoseParams params = std::get<MoveToPoseParams>(movement_s.moveParams);
        moveToPoseWithEarlyExit(movement_s.pose, movement_s.timeout, params, movement_s.exitDistance, movement_s.async, movement_s.degrees);
    } else if (std::holds_alternative<MoveToPointParams>(transformed_movement.moveParams)) {
        MoveToPointParams params = std::get<MoveToPointParams>(movement_s.moveParams);
        moveToPointWithEarlyExit(movement_s.pose, movement_s.timeout, params, movement_s.exitDistance, movement_s.async);
    }
}
}
