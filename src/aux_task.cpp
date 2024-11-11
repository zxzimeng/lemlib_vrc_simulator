#include "aux_task.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include "main.h"

std::mutex aux_mutex;

void handle_controller_inputs() {
    Aux lastaux = aux; // Initialize with the current aux state

    while (!stopflag) {


        // Lock the mutex to safely access the shared aux variable
        std::lock_guard<std::mutex> lock(aux_mutex);
        std::cout << "Checking state changes... Conveyor: " << aux.conveyor_state
                  << ", Flap: " << aux.flap_state
                  << ", Mogo: " << aux.mogo_state << std::endl;

        // Check if the conveyor state has changed
        if (lastaux.conveyor_state != aux.conveyor_state) {
            std::cout << "Conveyor state changed!" << std::endl;
            aux.enact_conveyor_state(); // Directly enact the state change
        }
        // Check if the flap state has changed
        else if (lastaux.flap_state != aux.flap_state) {
            std::cout << "Flap state changed!" << std::endl;
            aux.enact_flap_state(); // Directly enact the state change
        }
        // Check if the mogo state has changed
        else if (lastaux.mogo_state != aux.mogo_state) {
            std::cout << "Mogo state changed!" << std::endl;
            aux.enact_mogo_state(); // Directly enact the state change
        }

        // Update the last state to the current state for next iteration comparison
        lastaux = aux;
    }
}