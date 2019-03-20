#ifndef FLAME_H
#define FLAME_H

void detectFlame();
void resetFlameDetection();

extern bool flameDetected;
extern bool flameDetectedRight;
extern bool flameDetectedLeft;
extern bool flameDetectedBottomRight;
extern bool flameDetectedBottomLeft;
extern bool flameDetectedTopRight;
extern bool flameDetectedTopLeft;

#endif