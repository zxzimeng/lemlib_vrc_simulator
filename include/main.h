#pragma once
#include <atomic>

#include "chassis.hpp"
#include "extended_chassis.hpp"
#include "movement_utils.h"
#include "pose.hpp"
#include "util.hpp"
#include "simulate_pros_utils.h"
#include "aux.h"
#include "aux_task.h"

extern std::atomic<bool> stopflag;
extern Aux aux;
extern lemlib::Pose robot_pose;