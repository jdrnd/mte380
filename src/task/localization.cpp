#include "localization.h"
#include <Arduino.h>
#include "relocalize.h"

static bool relocalized;

void init_localization() {
    relocalized = false;
    start_relocalization();
    t_localization.setCallback(&localize);
}

void localize() {
    if(!relocalized)
        if(relocalize())
            relocalized = true;
    else {
        DEBUG_PRINT("FINISHED RELOCALIZATION");
        //PLOTTER_SERIAL.print("0,0\n");
    }
}


