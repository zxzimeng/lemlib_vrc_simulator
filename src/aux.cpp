#include "aux.h"

#include <cstdio>
using namespace std;

Aux::Aux() {
    return;
};

Aux::FlapState Aux::get_toggled_flap_state() {
    return flap_state == FLAP_EXTENDED ? FLAP_RETRACTED : FLAP_EXTENDED;
}

Aux::IntakeClawSystemState Aux::get_toggled_intake_claw_system_state() {
    return intake_claw_system_state == MANUAL_CONVEYOR ? AUTO_CONVEYOR : MANUAL_CONVEYOR;
}

void Aux::update_screen_intake_claw_system_state() {
    switch (intake_claw_system_state) {
        case MANUAL_CONVEYOR:
            printf("Claw Mode\n");
        // master.clear();
        // pros::delay(200);
        // master.set_text(0, 0, "Claw Mode\n");
            break;
        case AUTO_CONVEYOR:
            printf("Conveyor Mode\n");
        // master.clear();
        // pros::delay(200);
        // master.set_text(0, 0, "Conveyor Mode\n");
            break;
    }
}

void Aux::enact_conveyor_state() {
    // if (intake_claw_system_state == AUTO_CONVEYOR && ring_is_at_position) {
    //     printf("Conveyor Stop\n");
    //     // conveyor_motor.move_velocity(0);
    //     conveyor_state = CONVEYOR_STOP;
    // } else {
    switch (conveyor_state) {
        case CONVEYOR_FORWARD:
            // conveyor_motor.move_velocity(-12000);
            printf("Conveyor Forward\n");
            break;
        case CONVEYOR_REVERSE:
            // conveyor_motor.move_velocity(12000);
            printf("Conveyor Reverse\n");
            break;
        case CONVEYOR_STOP:
            printf("Conveyor Stop\n");
        // conveyor_motor.move_velocity(0);
            break;
    }
    // }
}

// void Aux::watch_and_stop_when_ring_is_detected() {
//     if (abs(ring_line.get_value_calibrated_HR()) > 25000) {
//         conveyor_motor.move_voltage(0);
//         conveyor_state = CONVEYOR_STOP;
//         ring_is_at_position = true;
//     } else if (ring_line.get_value_calibrated_HR() < 25000) {
//         ring_is_at_position = false;
//     }
// }

void Aux::enact_mogo_state() {
    switch (mogo_state) {
        case MOGO_LOCKED:
            // mogo_lock_solenoid.extend();
            printf("Mogo Locked\n");
            break;
        case MOGO_UNLOCKED:
            printf("Mogo Unlocked\n");
        // mogo_lock_solenoid.retract();
            break;
    };
}

void Aux::enact_flap_state() {
    switch (flap_state) {
        case FLAP_EXTENDED:
            // flap_solenoid.extend();
            printf("Flap Extended\n");
            break;
        case FLAP_RETRACTED:
            // flap_solenoid.retract();
            printf("Flap Extended\n");
            break;
    };
}

void Aux::spin_claw(ClawDirection direction, int voltage) {
    switch (direction) {
        case CW_TOWARDS_INSIDE:
            // claw_motor.move_voltage(-1 * (abs(voltage)));
            printf("Claw CW TOWARDS INSIDE\n");
            break;
        case CCW_TOWARDS_OUTSIDE:
            // claw_motor.move_voltage((abs(voltage)));
            printf("Claw CCW TOWARDS OUTSIDE\n");
            break;
        case CLAW_STOP:
            // claw_motor.move_voltage(0);
            printf("Claw STOP\n");
            break;
    }
}

void Aux::tare_claw(int timeout) {
    printf("Claw Tare with timeout:%fms", timeout);
    // int difference_from_target;
    // int error_range = 200;
    // pros::Clock::time_point start_claw = pros::Clock::now();
    // pros::Clock::duration duration = pros::Clock::now() - start_claw;
    // while (duration.count() < timeout) {
    //     int claw_position = claw_pos.get_value_calibrated_HR();
    //     difference_from_target = claw_position - POTENTIOMETER_BASE;
    //     if (difference_from_target > 0) {
    //         if (difference_from_target > 1000) {
    //             spin_claw(CW_TOWARDS_INSIDE, 10000);
    //         } else {
    //             spin_claw(CW_TOWARDS_INSIDE, 5000);
    //         }
    //     } else {
    //         if (difference_from_target < -1000) {
    //             spin_claw(CCW_TOWARDS_OUTSIDE, -10000);
    //         } else {
    //             spin_claw(CCW_TOWARDS_OUTSIDE, -5000);
    //         }
    //     }
    //     if ((claw_position < (POTENTIOMETER_BASE + error_range) && claw_position > (
    //              POTENTIOMETER_BASE - error_range)
    //         )
    //         ||
    //         (difference_from_target < error_range && difference_from_target > -1 * error_range)
    //     ) {
    //         break;
    //     }
    //     duration = pros::Clock::now() - start_claw;
    //     pros::delay(20);
    // }
    // spin_claw(CLAW_STOP);
}
