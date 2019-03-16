#ifndef OBJECT_DETECTION_H_
#define OBJECT_DETECTION_H_

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>
#include <math.h>

#include "sensors/rangefinders.h"

//threshold for what counts as a significant 'edge'
#define THRESHOLD 50

//the distance in the units of the LIDAR for 12 inches = 305 mm
#define DIS_PER_BLOCK 305

//this is dependant on how often the read_sensors task is run
//we only care about the last second worth of data for edge detection
#define MEASUREMENTS_PER_SECOND 20

//used to calculate direction of object
enum LidarSensor : uint8_t
    {
        LIDAR_FRONT                               = 0,
        LIDAR_LEFT                                = 1,
        LIDAR_BACK                                = 2,
        LIDAR_RIGHT                               = 3,
    };

extern Task t_objDetLin;
extern Task t_objDetRot;

extern Rangefinders rangefinders;

// linear 2D array, hense (0,1) = index 6, true if object detected
extern bool objects[36];

// measures the confidence of the presence of an object in a given coord, like # of detections.
extern int16_t confidence[36];

//looks for differences in the L and R buffers to see if there are objects to the left or right of the robot
//Run this at the same rate the lidars are updated at so process data continuously
void scan_objects_lin(uint16_t heading, uint16_t X, uint16_t Y);

//looks for differences in all buffers while the robot is turning
//Run this at the same rate the lidars are updated at so process data continuously
void scan_objects_rot(uint16_t heading, uint16_t X, uint16_t Y);
		
// takes the coordinate and rounds it to the nearest square
void round_object_coord(uint16_t X_obj, uint16_t Y_obj);

// translates the measurement to a coordinate in X, Y
// diff - the distance to the object detected
void locate_coord_lin(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y);

//translates the rotational measurment to a coordinate in X, Y
//diff - the distance to the object detected
//(X,Y) - the robot's current position
void locate_coord_rot(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y);
	
#endif
