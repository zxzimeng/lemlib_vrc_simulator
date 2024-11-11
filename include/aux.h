#pragma once
class Aux {
public:
    // Enums for different states
    enum ConveyorState { CONVEYOR_FORWARD, CONVEYOR_REVERSE, CONVEYOR_STOP };

    enum MogoState { MOGO_LOCKED, MOGO_UNLOCKED };

    enum FlapState { FLAP_EXTENDED, FLAP_RETRACTED };

    enum IntakeClawSystemState { MANUAL_CONVEYOR, AUTO_CONVEYOR };

    enum ClawDirection { CW_TOWARDS_INSIDE, CCW_TOWARDS_OUTSIDE, CLAW_STOP };

    // Member variables
    ConveyorState conveyor_state = CONVEYOR_STOP;
    MogoState mogo_state = MOGO_UNLOCKED;
    FlapState flap_state = FLAP_RETRACTED;
    IntakeClawSystemState intake_claw_system_state = AUTO_CONVEYOR;

    // bool ring_is_at_position = false;
    // bool initial_calibration_line = false;
    // int POTENTIOMETER_BASE = 12500; // percent

    // Constructor
    Aux();

    FlapState get_toggled_flap_state();

    IntakeClawSystemState get_toggled_intake_claw_system_state();

    void update_screen_intake_claw_system_state();

    void enact_conveyor_state();

    // void watch_and_stop_when_ring_is_detected();

    void enact_mogo_state();

    void enact_flap_state();

    void spin_claw(ClawDirection direction, int voltage = 12000);

    void tare_claw(int timeout = 5000);
};
