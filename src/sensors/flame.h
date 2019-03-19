#ifndef FLAME_H
#define FLAME_H

void detectFlame();
void detectFlameRight();
void detectFlameLeft();
void resetFlameDetection();

extern bool flameDetected;
extern bool flameDetectedRight;
extern bool flameDetectedLeft;
#endif