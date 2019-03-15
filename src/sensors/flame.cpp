#include <Arduino.h>

#include "flame.h"

bool flameDetected = false;

uint8_t FLAME_PINS[4] = {
    22,
    33,
    48,
    49
};

void detectFlame() {
    for (int i=0; i<4; i++) {
        if (digitalRead(FLAME_PINS[i]) == LOW) {
            flameDetected = true;
        }
    }
}

void resetFlameDetection() {
    flameDetected = false;
}