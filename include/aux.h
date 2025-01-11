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
    ConveyorState conveyorState = CONVEYOR_STOP;
    MogoState mogoState = MOGO_UNLOCKED;
    FlapState flapState = FLAP_RETRACTED;
    IntakeClawSystemState intakeClawSystemState = MANUAL_CONVEYOR;

    bool rightIsAtPosition = false;
    bool initialCalibrationLine = false;
    int POTENTIOMETER_BASE = 0; //


    // Constructor
    Aux();

    FlapState getToggledFlapState();

    IntakeClawSystemState getToggledIntakeClawSystemState();

    void updateScreenIntakeClawSystemState();

    void enactConveyorState();

    void watchStopWhenRingDetected();

    void enactMogoState();

    void enactFlapState();

    void spinClaw(ClawDirection direction, int voltage = 12000);

    void tareClaw(int timeout = 5000);

    void enactMogoState(Aux::MogoState target_state);

    void enactConveyorState(Aux::ConveyorState target_state);

    void enactFlapState(Aux::FlapState target_state);
};
