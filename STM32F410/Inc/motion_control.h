#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "main.h"
#include <stdio.h>


// Set pid loop etc whatever needed
void init_motion();

// Update pid with new values
// Probably stepper speed target, robot angle etc.. as a parameter
void update_motion_loop(uint16_t lm_speed);



#endif