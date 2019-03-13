#ifndef IR_H
#define IR_H

#include <Arduino.h>

#define IR_PIN 35

class IR {
    public:
        static void init();
        static void read();
        static bool detected;
};
#endif