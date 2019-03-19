#include <Arduino.h>

#include "flame.h"

bool flameDetected = false;
bool flameDetectedRight = false;
bool flameDetectedLeft = false;

const uint8_t BOTTOM_LEFT = 22;
const uint8_t TOP_LEFT = 33;
const uint8_t BOTTOM_RIGHT = 49;
const uint8_t TOP_RIGHT = 48;

const uint8_t FLAME_PINS[4] = {
    BOTTOM_LEFT, 
    TOP_LEFT, 
    BOTTOM_RIGHT, 
    TOP_RIGHT
};

void detectFlame() {
    for (int i=0; i<4; i++) {
        if (digitalRead(FLAME_PINS[i]) == LOW) {
            flameDetected = true;
        }
    }
}

void detectFlameRight() {
    flameDetectedRight = (digitalRead(BOTTOM_RIGHT) == LOW || digitalRead(TOP_RIGHT) == LOW);
}

void detectFlameLeft() {
    flameDetectedLeft = (digitalRead(BOTTOM_LEFT) == LOW || digitalRead(TOP_LEFT) == LOW);
}

void resetFlameDetection() {
    flameDetected = false;
}