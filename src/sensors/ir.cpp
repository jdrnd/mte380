#include <Arduino.h>

#include "ir.h"

bool IR::detected = false;

void IR::init() {
    pinMode(IR_PIN, INPUT);
}

void IR::read() {
    // Active low sensor
    detected = (digitalRead(IR_PIN) == LOW);
}
