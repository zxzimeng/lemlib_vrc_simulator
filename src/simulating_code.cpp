#include "main.h"
using namespace lemlib;
using namespace pros;

int intaking_delay = 400;
int mogo_settle_delay = 800;

void prepare_first_quarter() {
    // Assumes mogo is locked, goes to first ring, ends with mogo in corner
    // Chassis should be at {-40.622, -29.967, 90 (lock facing first mogo)}
    transform_across_field transformation = {false, false};

    // Back into Mogo and Lock
    chassis.processMovement(movement{
                                .pose = {-40.622, -29.967, 305},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    aux.enactMogoState(Aux::MOGO_LOCKED);

    // Wait for Lock
    delay(mogo_settle_delay);

    // Turn to face first Ring
    chassis.turnToHeading(90, 1000, {}, false);

    // Score Preload
    aux.enactConveyorState(Aux::CONVEYOR_FORWARD);
}

void first_quarter() {
    transform_across_field transformation = {false, false};
    // Collects rings and puts mogo in corner
    // Assumes facing first ring
    // Go to first ring
    chassis.processMovement(movement{
                                .pose = {-14, -30, 90},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 50},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    delay(intaking_delay);

    // Turn to face second ring
    chassis.turnToHeading(180, 1000, {}, false);

    // Move to second ring
    chassis.processMovement(movement{
                                .pose = {-18, -48, 180},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    delay(intaking_delay);

    chassis.processMovement(movement{
                                .pose = {-18, -56, 180},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = true, .minSpeed = 80},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    // Turn to face third ring
    chassis.turnToHeading(270, 1000, {}, false);

    // Move to third ring
    chassis.processMovement(movement{
                                .pose = {-36, -56, 270},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    delay(intaking_delay);

    // Go forward and get fourth ring
    chassis.processMovement(movement{
                                .pose = {-53, -56, 270}, .offset_distance = 0, .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true}, .exitDistance = 14, .timeout = 2500
                            },
                            transformation);

    delay(intaking_delay);

    // Backs out, turns to get fifth ring
    // Backs out
    chassis.processMovement(movement{
                                .pose = {-24, -56, 270},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    // Turns to face fifth ring
    chassis.turnToHeading(-130, 1000, {}, false);

    // Move to intake fifth ring
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = 17.5,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    delay(intaking_delay * 3);

    // Continue spinning conveyor for 3 more seconds,
    // Then reverse conveyor to avoid conveyor getting stuck on mogo,
    // Then stop conveyor.
    // if (in_prod) {
    // pros::Task([]() {
    //     delay(3000);
    //     aux.enactConveyorState(Aux::CONVEYOR_STOP);
    //     aux.enactConveyorState(Aux::CONVEYOR_REVERSE);
    //     delay(500);
    //     aux.enactConveyorState(Aux::CONVEYOR_STOP);
    // });
    // }
    // Face corner
    chassis.processMovement(movement{
                                .pose = {-42, -56, 60},
                                .offset_distance = 0,
                                .perp_offset_distance = 3,
                                .moveParams = MoveToPoseParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    // Back into corner
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -12,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    delay(mogo_settle_delay);

    aux.enactMogoState(Aux::MOGO_UNLOCKED);

    delay(mogo_settle_delay);

    // Move forward, out of corner
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = 10,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = true},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);
}

void lock_second_mogo() {
    float offset_perpendicular_mogo = -9.45;
    transform_across_field transformation = {false, false};
    chassis.processMovement(movement{
                                .pose = {-48, 22, 180},
                                .offset_distance = 0,
                                .perp_offset_distance = offset_perpendicular_mogo,
                                .moveParams = MoveToPoseParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 7000,
                            },
                            transformation);

    delay(mogo_settle_delay);
    aux.enactMogoState(Aux::MOGO_LOCKED);
}

void second_quarter() {
    transform_across_field transformation = {true, false};

    aux.enactConveyorState(Aux::CONVEYOR_FORWARD);

    // Collects rings and puts mogo in corner
    // Assumes facing first ring
    // Go to first ring

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.processMovement(movement{
                                .pose = {-14, -30, 90},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 50},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);
    delay(intaking_delay);

    // Turn to face second ring
    chassis.turnToHeading(0, 1000, {}, false);

    // Move to second ring
    chassis.processMovement(movement{
                                .pose = {-17, -48, 180},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    delay(intaking_delay);

    chassis.processMovement(movement{
                                .pose = {-17, -56, 180},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = true, .minSpeed = 80},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    // Turn to face third ring
    chassis.turnToHeading(270, 1000, {}, false);

    // Move to third ring
    chassis.processMovement(movement{
                                .pose = {-36, -56, 270},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    delay(intaking_delay);

    // Go forward and get fourth ring
    chassis.processMovement(movement{
                                .pose = {-53, -56, 270}, .offset_distance = 0, .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true}, .exitDistance = 14, .timeout = 2500
                            },
                            transformation);

    delay(intaking_delay);

    // Backs out, turns to get fifth ring
    // Backs out
    chassis.processMovement(movement{
                                .pose = {-24, -56, 270},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);
    // Turns to face fifth ring
    chassis.turnToHeading(-50, 1000, {}, false);

    // Move to intake fifth ring
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = 15,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            {false, false});

    delay(intaking_delay * 2);

    // Continue spinning conveyor for 3 more seconds,
    // Then reverse conveyor to avoid conveyor getting stuck on mogo,
    // Then stop conveyor.
    // if (in_prod)
    // {
    // pros::Task([]() {
    //     delay(3000);
    //     aux.enactConveyorState(Aux::CONVEYOR_STOP);
    //     aux.enactConveyorState(Aux::CONVEYOR_REVERSE);
    //     delay(500);
    //     aux.enactConveyorState(Aux::CONVEYOR_STOP);
    // });
    // }
    // Face corner
    chassis.processMovement(movement{
                                .pose = {-42, -56, 50},
                                .offset_distance = 0,
                                .perp_offset_distance = -2,
                                .moveParams = MoveToPoseParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    // Back into corner
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -14.5,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            {false, false});

    // delay(mogo_settle_delay);

    aux.enactMogoState(Aux::MOGO_UNLOCKED);

    delay(mogo_settle_delay * 2);

    // Move forward, out of corner
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = 10,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = true},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            {false, false});
}

void ram_first_mogo() {
    transform_across_field transformation = {false, false};

    // Lock mogo lock to prevent jammping
    aux.enactMogoState(Aux::MOGO_LOCKED);

    // Go behind the first mogo
    chassis.processMovement(movement{
                                .pose = {50, -5, -60},
                                .offset_distance = 0,
                                .perp_offset_distance = 11,
                                .moveParams = MoveToPoseParams{.forwards = false},
                                .exitDistance = 0,
                                .timeout = 3500,
                            },
                            transformation);

    // Turn such that it is facing the corner
    chassis.turnToHeading(-170, 1000, {}, false);

    // Back into mogo until it goes in corner
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -65,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = false, .minSpeed = 127},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    // Move forward after backing in
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = 20,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = true, .minSpeed = 127},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);
}

void ram_second_mogo() {
    transform_across_field transformation = {false, false};

    // Lock mogo lock to prevent jammping
    aux.enactMogoState(Aux::MOGO_LOCKED);

    // Go behind the first mogo
    chassis.processMovement(movement{
                                .pose = {53, -14, 0},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false, .lead = 0.1, .minSpeed = 127},
                                .exitDistance = 10,
                                .timeout = 2500,
                            },
                            transformation);

    // Turn such that it is facing the corner
    chassis.turnToHeading(-8, 1000, {}, false);

    // Back into mogo until it goes in corner
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -60,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = false, .minSpeed = 127},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);

    // Move forward after backing in
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = 20,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = true, .minSpeed = 127},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            transformation);
}

void red_60s() {
    // printf("60");
    // return;
    lemlib::Pose red_right_skill_starting_pose = {(-24 - 28.5), (-24 + 2.35), 305};
    chassis.setPose(red_right_skill_starting_pose);

    // lock onto mogo -> face first ring
    prepare_first_quarter();

    // fill first mogo -> put in corner, move forward
    first_quarter();

    lock_second_mogo();

    // chassis thinks it at :{-48, 22, 180}, it should be in line with a ring at {-14, 30, 90}
    chassis.setPose({chassis.getPose().x, (chassis.getPose().y + 11), chassis.getPose().theta});

    second_quarter();

    // chassis should go to {0,48,90}
    chassis.processMovement(movement{
                                .pose = {6 + 6, 53, 90},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 80},
                                .exitDistance = 10,
                                .timeout = 7000,
                            },
                            {false, false});

    chassis.setPose(Pose{6, 53, 90}, false);

    // ram_first_mogo();

    // waypoint between two mogos

    ram_second_mogo();
    chassis.processMovement(movement{
                                .pose = {66, 62, 3},
                                .offset_distance = 0,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true, .lead = 0.3, .minSpeed = 127},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            {false, false});
    delay(mogo_settle_delay);
    chassis.processMovement(movement{
                                .pose = chassis.getPose(),
                                .offset_distance = -20,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = false, .lead = 0.3, .minSpeed = 127},
                                .exitDistance = 0,
                                .timeout = 2500,
                            },
                            {false, false});
}
