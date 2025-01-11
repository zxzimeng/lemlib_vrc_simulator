#include <variant>
#include "main.h"

#include <generate_jerryio.h>
#include <thread>
bool jerryio = true;
std::atomic<bool> stopflag(false);
bool skills_field=true;
lemlib::ExtendedChassis chassis;
lemlib::Pose robot_pose={0,0,0};
std::vector<lemlib::Pose> movements;
Aux aux;
// std::thread t(handle_controller_inputs);
void main_code() {
    // red_60s();
    red_60s();

    // red_right(true, true);
    // red_left(false);
    // blue_right(true);
    // blue_left(true, true);
}
int main() {
    main_code();
    // red_60s();
    stopflag=true;
    // t.join();
    generatePathFile("output.txt");
}