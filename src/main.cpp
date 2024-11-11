#include <variant>
#include "main.h"
#include <thread>

bool jerryio = true;
std::atomic<bool> stopflag(false);
lemlib::ExtendedChassis chassis;
lemlib::Pose robot_pose={0,0,0};
Aux aux;
// std::thread t(handle_controller_inputs);

using namespace lemlib;
int main() {
    aux.conveyor_state=Aux::CONVEYOR_FORWARD;
    aux.enact_conveyor_state();
    jerryio=true;
    lemlib::Pose red_right_skill_starting_pose = {(-24 - 28.5), (-24 + 2.35), 305};
    chassis.setPose(red_right_skill_starting_pose);

    // 3RD QUADRANT
    transform_across_field transformation = {false, false};

    // BACK INTO MOGO AND LOCK
    chassis.processMovement(movement{
                                .pose = red_right_skill_starting_pose, .offset_distance = -14.5,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPointParams{.forwards = false, .minSpeed = 30}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    aux.mogo_state = Aux::MOGO_LOCKED;
    aux.enact_mogo_state();
    delay(500);
    // END BACK AND LOCK

    // SCORE PRELOAD
    // aux.conveyor_state = Aux::CONVEYOR_FORWARD;

    // END SCOREEE

    // GET FIRST RING->MOGO IN CORNER
    chassis.turnToHeading(90, 1000, {}, false);
    chassis.processMovement(movement{
                                .pose = {-24, -28, 90}, .offset_distance = 7,
                                .perp_offset_distance = 0,
                                .moveParams = MoveToPoseParams{.forwards = true, .minSpeed = 50}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    // base_quarter_field(transformation);
    // END FIRST RING->MOGO IN CORNER
    //
    // // MOVE TO FIFTH RING IN TWO MOVES: BACKS OUT AND AWAY (X++ Y++) AND THEN GOES IN
    chassis.processMovement(movement{
                                .pose = {
                                    chassis.getPose().x, chassis.getPose().y,
                                    transformPose({0, 0, -90}, transformation).theta
                                },
                                .offset_distance = -25,
                                .perp_offset_distance = 2,
                                .moveParams = MoveToPoseParams{.forwards = false, .minSpeed = 60}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    if (transformation == transform_across_field{true, false}
    ) {
        chassis.turnToHeading(-40, 1000,
                              {AngularDirection::CW_CLOCKWISE},
                              false);
    } else {
        chassis.turnToHeading(-130, 1000,
                              {AngularDirection::CCW_COUNTERCLOCKWISE},
                              false);
    }
    delay(800);
    chassis.processMovement(movement{
                                .pose = chassis.getPose(), .offset_distance = 10, .perp_offset_distance = -2,
                                .moveParams = MoveToPointParams{.forwards = true, .minSpeed = 40}, .exitDistance = 0,
                                .timeout = 4000,
                            }, {false, false});
    delay(800);
    aux.mogo_state=Aux::MOGO_UNLOCKED;
    aux.enact_mogo_state();
    // aux.flap_state=Aux::FLAP_EXTENDED;
    // sleep(1);
    stopflag=true;
    // t.join();
}
