#include <Arduino.h>

#include "flame.h"

bool flameDetected = false;
bool flameDetectedRight = false;
bool flameDetectedLeft = false;
bool flameDetectedBottomRight = false;
bool flameDetectedBottomLeft = false;
bool flameDetectedTopRight = false;
bool flameDetectedTopLeft = false;

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
    flameDetectedBottomRight = digitalRead(BOTTOM_RIGHT) == LOW;
    flameDetectedBottomLeft = digitalRead(BOTTOM_LEFT) == LOW;
    flameDetectedTopRight = digitalRead(TOP_RIGHT) == LOW;
    flameDetectedTopLeft = digitalRead(TOP_LEFT) == LOW;
    flameDetectedRight = flameDetectedBottomRight || flameDetectedTopRight;
    flameDetectedLeft = flameDetectedBottomLeft || flameDetectedTopLeft;
    flameDetected = flameDetectedRight || flameDetectedLeft;
}
