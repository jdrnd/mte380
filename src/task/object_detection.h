#ifndef OBJECT_DETECTION_H_
#define OBJECT_DETECTION_H_

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

#include "sensors/rangefinders.h"
#include "task/mission_control.h"
#include "task/motor_control.h"

//threshold for what counts as a significant 'edge'
#define THRESHOLD 25

//the distance in the units of the LIDAR for 12 inches = 305 mm
#define DIS_PER_BLOCK 305
#define MAX_LENGTH_COURSE 1830

#define RIGHT_LIDAR_MAX 1100
#define LEFT_LIDAR_MAX 1100
#define BACK_LIDAR_MAX 1100
#define FRONT_LIDAR_MAX 1100

//effects how often object detection flags are reset

// number of cycles we use recent data
#define USE_LATEST_DATA_RESET 6 //3
// number of cycles before we reset the obejct flag if there was noise
#define OBJECT_DET_HARD_RESET 40 //30
// number of cycles behind the most recent stream of data that the left and right coordinate is updated with
// this allows an object to be detected while preventing the LR coord from shifting
#define LR_DELAY 6  
// number of cycles the algorithm waits after the falling edge of an object for the data to be reliable
#define WAIT_AFTER_OBJECT 5 //5

#define OBJ_CALC_DELAY 10 //2

//used in cpp
#define min(a,b) ((a)<(b)?(a):(b))

//USED IN CPP
#define min(a,b) ((a)<(b)?(a):(b))

//used to calculate direction of object
enum LidarSensor : uint8_t
{
    LIDAR_FRONT                               = 0,
    LIDAR_LEFT                                = 1,
    LIDAR_BACK                                = 2,
    LIDAR_RIGHT                               = 3,
};

struct Point {
    uint8_t x;
    uint8_t y;
    uint16_t confidence;
};

extern Task t_localize;

extern Rangefinders rangefinders;

namespace ObjectDetection {
//Global variables for object deteciton and localization

//position of the robot
extern uint16_t X;
extern uint16_t Y;

// linear 2D array, hense (0,1) = index 6, true if object detected
extern bool objects[6][6];
// measures the confidence of the presence of an object in a given coord, like # of detections.
extern int16_t confidence[6][6];
extern Point points[2];

//predicted position of the robot
extern uint16_t X;
extern uint16_t Y;

extern int16_t der_r;
extern int16_t der_l;

//flags to check if an object has been detected
extern bool obj_r;
extern bool obj_l;

//flags for assessing the reliability of each rangefinder
extern bool rel_r;
extern bool rel_l;
extern bool rel_f;
extern bool rel_b;

//holds the last few values of predicited X and Y so we can 
//go back in time for detecting objects.
extern CircularBuffer<uint16_t, 20, uint8_t> Xreadings;
extern CircularBuffer<uint16_t, 20, uint8_t> Yreadings;

//scans for objects on either side while updating the global X,Y variables
void localize();

void lidar_derivative();

//looks for differences in all buffers while the robot is turning
//Run this at the same rate the lidars are updated at so process data continuously
void scan_objects_rot();
		
// takes the coordinate and rounds it to the nearest square
void round_object_coord(uint16_t X_obj, uint16_t Y_obj);

// translates the measurement to a coordinate in X, Y
// diff - the distance to the object detected
void locate_coord_lin(LidarSensor sensor, uint16_t diff, uint16_t x, uint16_t y);

//translates the rotational measurment to a coordinate in X, Y
//diff - the distance to the object detected
//(X,Y) - the robot's current position
void locate_coord_rot(LidarSensor sensor, uint16_t diff);

uint8_t find_lowest_reading_index(LidarSensor sensor, uint8_t max);
void find_best_points();
bool are_we_blocked(LidarSensor sensor);

void print_object_data();
}
#endif
