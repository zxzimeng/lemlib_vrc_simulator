#include "main.h"

//  > HORIZONTAL MIRROR
//  ^   ┌───────────────────────┐
//  RED │ -                   - │BLUE
//  LEFT│                       │RIGHT > VERTICAL & HORIZONTAL MIRROR
//      │                       │
//      │                       │
//      │                       │
//      │                       │
//   RED│                       │BLUE
//      │                       │
//      │                       │
//      │                       │
//      │          XXX          │
// RED  │         RISKG         │BLUE
// RIGHT│ +                   + │LEFT > VERTICAL MIRROR
//      └───────────────────────┘

using namespace lemlib;
using namespace pros;

void red_right(bool ladder, bool awp) {
    const transform_across_field transformation{false, false};
    base_red_right(ladder, awp, transformation);
}

void red_left(bool ladder, bool awp) {
    const transform_across_field transformation{true, false};
    base_red_left(ladder, awp, transformation);
}

void blue_left(bool ladder, bool awp) {
    const transform_across_field transformation{false, true};
    base_red_left(ladder, awp, transformation);
}

void blue_right(bool ladder) {
    const transform_across_field transformation{true, true};
    base_red_right(ladder, false, transformation);
}
