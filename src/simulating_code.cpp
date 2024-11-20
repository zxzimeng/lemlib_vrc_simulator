//
// Created by fox on 11/20/24.
//
#include "main.h"
using namespace lemlib;

void red_right_awp_base_movements(bool ladder, transform_across_field transformation) {
    Pose red_right_starting_pose = {-55.724, -31.446, -90};
    Pose starting_pose = transformOnlyPose(red_right_starting_pose, transformation);
    chassis.setPose(starting_pose);

    // MOVE TO MOGO AND LOCK WITH ONE WAYPOINT
    chassis.processMovement(movement{
                                .pose = {-41.652, -36.117, 270}, .offset_distance = 0, .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 70}, .exitDistance = 0,
                                .timeout = 4000
                            },
                            transformation);
    chassis.processMovement(movement{
                                .pose = {-31.655, -28.682, 241},
                                .offset_distance = -6,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{
                                    .forwards = false,
                                    .maxSpeed = 50,
                                    .minSpeed = 40,
                                },
                                .exitDistance = 0,
                                .timeout = 4000,
                            },
                            transformation);
    delay(500); // wait for objects to settle
    aux.mogo_state = Aux::MOGO_LOCKED;
    // END MOVE AND LOCK

    // START CONVEYOR
    aux.conveyor_state = Aux::CONVEYOR_FORWARD;
    // END START

    // AWAIT PRELOAD
    delay(1250);
    // END AWAIT

    chassis.turnToHeading(180, 1000, {}, false);

    // MOVE TO SECOND RING WITH ONE WAYPOINT
    chassis.processMovement(movement{
                                .pose = {-26.407, -25.773, 180},
                                .offset_distance = 24,
                                .perp_offset_distance = 2,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 30},
                                .exitDistance = 0,
                                .timeout = 4000,
                            },
                            transformation);
    // END MOVE
    delay(800); // wait for objects to settle
    aux.conveyor_state = Aux::CONVEYOR_STOP;

    chassis.processMovement(movement{
                                .pose = {-26.407, -25.773, 180},
                                .offset_distance = 2,
                                .perp_offset_distance = -3,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 30},
                                .exitDistance = 0,
                                .timeout = 4000,
                            },
                            transformation);

    if (transformation == transform_across_field{false, true}) {
        chassis.turnToHeading(48.6, 1000, {}, false);
    } else {
        chassis.turnToHeading(311.4, 1000, {}, false);
    }

    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -20,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 30},
                                .exitDistance = 0,
                                .timeout = 4000,
                            },
                            {false, false});


    // MOVE TO LADDER
    if (ladder) {
        chassis.processMovement(movement{
                                    .pose = {-17, -35, 0},
                                    .offset_distance = 0,
                                    .perp_offset_distance = 0,
                                    .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 30},
                                    .exitDistance = 0,
                                    .timeout = 4000,
                                },
                                transformation);
        chassis.processMovement(movement{
                                    .pose = {-17, -10, 0},
                                    .offset_distance = 0,
                                    .perp_offset_distance = 0,
                                    .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 30},
                                    .exitDistance = 0,
                                    .timeout = 4000,
                                },
                                transformation);
    }
}

void red_right_base_movements(bool ladder, transform_across_field transformation) {
    Pose red_right_starting_pose = {-55.724, -31.446, -90};
    Pose starting_pose = transformOnlyPose(red_right_starting_pose, transformation);
    chassis.setPose(starting_pose);

    // MOVE TO MOGO AND LOCK WITH ONE WAYPOINT
    chassis.processMovement(movement{
                                .pose = {-41.652, -36.117, 270}, .offset_distance = 0, .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 70}, .exitDistance = 0,
                                .timeout = 4000
                            },
                            transformation);
    chassis.processMovement(movement{
                                .pose = {-31.655, -28.682, 241},
                                .offset_distance = -6,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{
                                    .forwards = false,
                                    .maxSpeed = 50,
                                    .minSpeed = 40,
                                },
                                .exitDistance = 0,
                                .timeout = 4000,
                            },
                            transformation);
    delay(500); // wait for objects to settle
    aux.mogo_state = Aux::MOGO_LOCKED;
    // END MOVE AND LOCK

    // START CONVEYOR
    aux.conveyor_state = Aux::CONVEYOR_FORWARD;
    // END START

    // AWAIT PRELOAD
    delay(1250);
    // END AWAIT

    chassis.turnToHeading(180, 1000, {}, false);

    // MOVE TO SECOND RING WITH ONE WAYPOINT
    chassis.processMovement(movement{
                                .pose = {-23.407, -25.773, 180},
                                .offset_distance = 24,
                                .perp_offset_distance = -0.5,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 30},
                                .exitDistance = 0,
                                .timeout = 4000,
                            },
                            transformation);
    // END MOVE
    delay(2000);
    aux.conveyor_state = Aux::CONVEYOR_STOP;
    aux.mogo_state = Aux::MOGO_UNLOCKED;
    chassis.processMovement(movement{
                                .pose = {-15, -44, -90},
                                .offset_distance = 2,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 30},
                                .exitDistance = 0,
                                .timeout = 4000,
                            },
                            transformation);
    // aux.flap_state = Aux::FLAP_EXTENDED;
    // chassis.turnToHeading(chassis.getPose().theta + 180, 1000, {.direction = AngularDirection::CW_CLOCKWISE}, false);
}

void red_right(bool ladder) {
    const transform_across_field transformation{false, false};
    red_right_base_movements(ladder, transformation);
}

void red_left(bool ladder) {
    const transform_across_field transformation{true, false};
    red_right_base_movements(ladder, transformation);
}

void blue_left(bool ladder) {
    const transform_across_field transformation{false, true};
    red_right_base_movements(ladder, transformation);
}

void blue_right(bool ladder) {
    const transform_across_field transformation{true, true};
    red_right_base_movements(ladder, transformation);
}

void main_code() {
    blue_right(true);
}