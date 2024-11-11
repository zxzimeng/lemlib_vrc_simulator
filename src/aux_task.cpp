#include "aux_task.h"

#include <iostream>
#include <thread>
#include <threads.h>

#include "main.h"

void handle_controller_inputs() {
    Aux lastaux;
    lastaux=aux;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
        if (lastaux.conveyor_state!=aux.conveyor_state) {
            std::thread c([]->void {aux.enact_conveyor_state();});
            c.join();
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
        }
        else if (lastaux.flap_state!=aux.flap_state) {
            std::thread f([]->void{aux.enact_flap_state();});
            f.join();
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
        }
        else if (lastaux.mogo_state!=aux.mogo_state) {
            std::thread m([]->void{aux.enact_mogo_state();});
            m.join();
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
        }
        lastaux=aux;
        if (stopflag) {
            break;
        }
    }
}
