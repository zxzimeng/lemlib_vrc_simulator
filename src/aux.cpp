#include "main.h"


Aux::Aux(){
}

Aux::FlapState Aux::getToggledFlapState() {
    return flapState == FLAP_EXTENDED ? FLAP_RETRACTED : FLAP_EXTENDED;
}

Aux::IntakeClawSystemState Aux::getToggledIntakeClawSystemState() {
    return intakeClawSystemState == MANUAL_CONVEYOR ? AUTO_CONVEYOR : MANUAL_CONVEYOR;
}

void Aux::enactMogoState(MogoState target_state) {
    mogoState = target_state;
    printf("Mogo is at: %s", target_state == MOGO_LOCKED ? "Mogo Locked" : "Mogo Unlcoked");
}

void Aux::enactConveyorState(ConveyorState target_state) {
    conveyorState = target_state;
    printf("Conveyor is at: %s", target_state == CONVEYOR_STOP ? "Conveyor Stopped" : target_state==CONVEYOR_FORWARD ? "Conveyor Forward" : "Conveyor Back");
}

void Aux::enactFlapState(FlapState target_state) {
    flapState = target_state;
    printf("Flap is at: %s", target_state == FLAP_EXTENDED ? "Flap Extended" : "Flap Retracted");
}

