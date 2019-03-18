#include "localization.h"
#include <Arduino.h>
#include "relocalize.h"

static bool relocalized;

static float x,y,r;

void init_localization() {
    relocalized = false;
    start_relocalization();
    t_localization.setCallback(&localize);
}

void localize() {
    if(!relocalized) {
        if(relocalize()) {
            relocalized = true;
        }
    } else {
        //DEBUG_PRINT("FINISHED RELOCALIZATION");
        //PLOTTER_SERIAL.print("1,1");
    }
}

bool getXY(bool right_dead, bool front_dead, bool left_dead, bool back_dead) {
    if (front_dead && back_dead || right_dead && left_dead)
        return false;
    
}
