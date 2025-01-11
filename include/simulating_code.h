//
// Created by fox on 11/20/24.
//

#ifndef SIMULATING_CODE_H
#define SIMULATING_CODE_H
void red_right(bool ladder, bool awp);

void blue_left(bool ladder, bool awp);

void red_left(bool ladder, bool awp);

void blue_right(bool ladder);

void red_60s();

using namespace lemlib;
void base_red_right(bool ladder, bool awp, lemlib::transform_across_field transformation);

void base_blue_right(bool ladder, bool awp, lemlib::transform_across_field transformation);

void base_blue_left(bool ladder, bool awp, lemlib::transform_across_field transformation);

void base_red_left(bool ladder, bool awp, lemlib::transform_across_field transformation);

#endif //SIMULATING_CODE_H
