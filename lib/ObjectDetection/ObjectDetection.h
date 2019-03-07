#ifndef OBJECTDETECTION_H_
#define OBJECTDETECTION_H_

#include <Arduino.h>

//size of the buffer for the lidars, so if we are running at 20Hz, 1 second of data
//hopefully this will not cause too many errors
#define BUFF_SIZE 20

//threshold for what counts as a significant 'edge'
//starting value for testing will be 5 cm
#define THRESHOLD 50

//the distance in the units of the LIDAR for 12 inches = 305 mm
#define DIS_PER_BLOCK 305

//offsets to the lidars from the IMU
//these will have to be calcualted, in mm
#define RIGHT_LIDAR_OFFSET 50
#define LEFT_LIDAR_OFFSET 50
#define FRONT_LIDAR_OFFSET 100
#define BACK_LIDAR_OFFSET 100

enum LidarSensor : uint8_t
    {
        LIDAR_FRONT                               = 0,
        LIDAR_LEFT                                = 1,
        LIDAR_BACK                                = 2,
        LIDAR_RIGHT                               = 3,
    };

/*

heading = pi/2 radians

Y
|
5

4

3

2               starting position?
               /
1             /
             /
0	1	2	3	4	5 - X  -------- heading = 0 radians

*/

class ObjectDetection{

	public:
		// assume there are some form of global object holding a list of recent readings of the distance sensors
		// most recent is at index 0
		uint32_t R[BUFF_SIZE];
		uint32_t L[BUFF_SIZE];
		uint32_t F[BUFF_SIZE];
		uint32_t B[BUFF_SIZE];
		
		ObjectDetection();
		
		/* looks for differences in the L and R buffers to see if there are objects to the left or right of the robot
		   will then attempt to place that object in the closest square, so high precision accuracy shouldn't be an issue for 6x6in precision.
        */
		void scan_objects_lin(uint16_t heading, uint16_t X, uint16_t Y);
		
		/* uses a combination of the current heading and position to find where the object is
        ISSUES:
        -relies on lidar being very close to 90 deg from eachother
        -error increases the farther the object is
        -assumes we can somewhat accurately point turn, or always have our centre position during a turn
        */
		void scan_objects_rot(uint16_t heading, uint16_t X, uint16_t Y);
	
	private:
		// linear 2D array, hense (0,1) = index 6,
        // true if object has been detected
		bool objects[36];
		
		// measures the confidence of the presence of an object in a given coord, like # of detections.
		// not yet sure how we could incorporate the 'lack' of a detection.
		// could do some kind of rotational 'sweep' looking for/verifying known objects, to reduce errors?
		int16_t confidence[36];
		
		// takes the coordinate and rounds it to the nearest square
        void round_object_coord(uint16_t X_obj, uint16_t Y_obj);
		
		// translates the measurement to a coordinate in X, Y
		// sensor = 0 -> left sensor, sensor = 1 -> right sensor
        // diff - the distance to the object detected
		void locate_coord_lin(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y);
		
        /*
        translates the rotational measurment to a coordinate in X, Y
        Param:
        sensor - 0 = front, 1 = left, 2 = back, 3 = right
        diff - the distance to the object detected
        (X,Y) - the robot's current position */
		void locate_coord_rot(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y);
		
};

#endif
