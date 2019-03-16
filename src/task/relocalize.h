#ifndef RELOCALIZE
#define RELOCALIZE

#include "sensors/rangefinders.h"
#include "actuators/drive_motors.h"

extern Rangefinders rangefinders;

extern Motors motors;

void start_relocalization();

bool relocalize();

#endif
