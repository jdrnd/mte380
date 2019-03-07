
#include "objectdetection.h"
#include <Arduino.h>


<<<<<<< HEAD
void ObjectDetection::scan_objects_lin(uint16_t heading, uint16_t X, uint16_t Y){
	
    // scan with this width [0][1][2][i][4][5]
    // if difference between i and i-1 then check prev 2 value and next 2 for consistency
	for(uint16_t i = 3; i<BUFF_SIZE-3; i++)
	{
		uint32_t diff_R = abs(R[i] - R[i-1]);
		uint32_t diff_L = abs(L[i] - L[i-1]);
       
		if(diff_R > THRESHOLD)
		{
            //check that previous values are consistent
            if(abs(R[i-2]-R[i]) > THRESHOLD && abs(R[i-3]-R[i]) > THRESHOLD)
            {
                //check that the next two values are also a significant change to avoid noise
                if(abs(R[i+1]-R[i-1]) > THRESHOLD && abs(R[i+2]-R[i-1]) > THRESHOLD)
                {
                    locate_coord_lin(heading, LIDAR_RIGHT, min(R[i], R[i-1]) + RIGHT_LIDAR_OFFSET, X, Y);
                }
            }
        }
		if(diff_L > THRESHOLD)
		{
            //check that previous values are consistent
            if(abs(L[i-2]-L[i]) > THRESHOLD && abs(L[i-3]-L[i]) > THRESHOLD)
            {
                //check that the next two values are also a significant change to avoid noise
                if(abs(L[i+1]-L[i-1]) > THRESHOLD && abs(L[i+2]-L[i-1]) > THRESHOLD)
                {
			        locate_coord_lin(heading, LIDAR_LEFT, min(L[i], L[i-1]) + LEFT_LIDAR_OFFSET, X, Y);
                }
            }
        }
	}	
}

// heavily relies on us being able to maintain a cardinal direction
void ObjectDetection::locate_coord_lin(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y)
{
	if(heading == 0)
	{	
		if (sensor == LIDAR_LEFT)
			round_object_coord(X, Y + diff + LEFT_LIDAR_OFFSET);
		else 
			round_object_coord(X, Y - diff - RIGHT_LIDAR_OFFSET);
	}		
	else if(heading == 90)
	{
		if(sensor == LIDAR_LEFT)
			round_object_coord(X - diff - LEFT_LIDAR_OFFSET, Y);
		else 
			round_object_coord(X + diff + RIGHT_LIDAR_OFFSET, Y);
	}		
	else if(heading == 180)
	{
		if(sensor == LIDAR_LEFT)
			round_object_coord(X, Y - diff - LEFT_LIDAR_OFFSET);
		else 
			round_object_coord(X, Y + diff + RIGHT_LIDAR_OFFSET);
	}
	else //(heading == 270)
	{
		if(sensor == LIDAR_LEFT)
			round_object_coord(X + diff + LEFT_LIDAR_OFFSET, Y);
		else 
			round_object_coord(X - diff - RIGHT_LIDAR_OFFSET, Y);
	}
}

void ObjectDetection::scan_objects_rot(uint16_t heading, uint16_t X, uint16_t Y)
{
	for(uint16_t i = 1; i<BUFF_SIZE; i++)
	{
		uint32_t diff_R = abs(R[i] - R[i-1]);
		uint32_t diff_L = abs(L[i] - L[i-1]);
		uint32_t diff_F = abs(F[i] - F[i-1]);
		uint32_t diff_B = abs(B[i] - B[i-1]);
		if(diff_R > THRESHOLD)
		{
            //check that
            if(abs(R[i-2]-R[i]) > THRESHOLD && abs(R[i-3]-R[i]) > THRESHOLD)
            {
                //check that the next two values are also a significant change to avoid noise
                if(abs(R[i+1]-R[i-1]) > THRESHOLD && abs(R[i+2]-R[i-1]) > THRESHOLD)
                {
                    locate_coord_rot(heading, LIDAR_RIGHT, min(R[i], R[i-1]) + RIGHT_LIDAR_OFFSET, X, Y);
                }
            }
        }
		if(diff_L > THRESHOLD)
		{
            //check that previous values are consistent
            if(abs(L[i-2]-L[i]) > THRESHOLD && abs(L[i-3]-L[i]) > THRESHOLD)
            {
                //check that the next two values are also a significant change to avoid noise
                if(abs(L[i+1]-L[i-1]) > THRESHOLD && abs(L[i+2]-L[i-1]) > THRESHOLD)
                {
			        locate_coord_lin(heading, LIDAR_LEFT, min(L[i], L[i-1]) + LEFT_LIDAR_OFFSET, X, Y);
                }
            }
        }
		if(diff_F > THRESHOLD)
		{
            //check that previous values are consistent
            if(abs(F[i-2]-F[i]) > THRESHOLD && abs(F[i-3]-F[i]) > THRESHOLD)
            {
                //check that the next two values are also a significant change to avoid noise
                if(abs(F[i+1]-F[i-1]) > THRESHOLD && abs(F[i+2]-F[i-1]) > THRESHOLD)
                {
			        locate_coord_rot(heading, LIDAR_FRONT, min(F[i], F[i-1]) + FRONT_LIDAR_OFFSET, X, Y);
                }
            }
        }
		if(diff_B > THRESHOLD)
		{
            //check that previous values are consistent
            if(abs(B[i-2]-B[i]) > THRESHOLD && abs(B[i-3]-B[i]) > THRESHOLD)
            {
                //check that the next two values are also a significant change to avoid noise
                if(abs(B[i+1]-B[i-1]) > THRESHOLD && abs(B[i+2]-B[i-1]) > THRESHOLD)
                {
			locate_coord_rot(heading, LIDAR_BACK, min(B[i], B[i-1]) + BACK_LIDAR_OFFSET, X, Y);
                }
            }
        }
	}		
}

void ObjectDetection::locate_coord_rot(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y)
{
	double X_obj = X + diff*cos(heading+0.5*PI*sensor);
	double Y_obj = Y + diff*sin(heading+0.5*PI*sensor);
=======
void ObjectDetection::scan_objects_lin(uint16_t heading, uint32_t X, uint32_t Y){
	
	for(uint16_t i = 1; i<BUFF_SIZE; i++)
	{
		uint32_t diff_R = abs(R[i] - R[i-1]);
		uint32_t diff_L = abs(L[i] - L[i-1]);
		if(diff_R > THRESHOLD)
		{
			locate_coord_lin(heading, 1, min(R[i], R[i-1]));
		}
		if(diff_L > THRESHOLD)
		{
			locate_coord_lin(heading, 0, min(L[i], L[i-1]));
		}
	}	
}

void ObjectDetection::locate_coord_lin(uint16_t heading, uint8_t sensor, uint32_t diff)
{
	if(heading == 0)
	{	
		if (sensor == 0)
			round_object_coord(X, Y + diff);
		else 
			round_object_coord(X, Y - diff);
	}		
	else if(heading == 90)
	{
		if(sensor == 0)
			round_object_coord(X - diff, Y);
		else 
			round_object_coord(X + diff, Y);
	}		
	else if(heading == 180)
	{
		if(sensor == 0)
			round_object_coord(X, Y - diff);
		else 
			round_object_coord(X, Y + diff);
	}
	else(heading == 270)
	{
		if(sensor == 0)
			round_object_coord(X + diff, Y);
		else 
			round_object_coord(X - diff, Y);
	}
}

void ObjectDetection::scan_objects_rot(double heading)
{
	for(uint16_t i = 1; i<BUFF_SIZE; i++)
	{
		uint32_t diff_R = abs(R[i] - R[i-1])
		uint32_t diff_L = abs(L[i] - L[i-1])
		uint32_t diff_F = abs(F[i] - F[i-1])
		uint32_t diff_B = abs(B[i] - B[i-1])
		if(diff_R > THRESHOLD)
		{
			locate_coord_rot(heading, 3, min(R[i], R[i-1]));
		}
		if(diff_L > THRESHOLD)
		{
			locate_coord_rot(heading, 1, min(L[i], L[i-1]));
		}
		if(diff_F > THRESHOLD)
		{
			locate_coord_rot(heading, 0, min(F[i], F[i-1]));
		}
		if(diff_B > THRESHOLD)
		{
			locate_coord_rot(heading, 2, min(B[i], B[i-1]));
		}
	}		
}

void ObjectDetection::locate_coord_rot(float heading, uint8_t sensor, uint32_t diff)
{
	double X_obj = X + diff*cos(heading+0.5*pi*sensor);
	double Y_obj = Y + diff*sin(heading+0.5*pi*sensor);
>>>>>>> 9072e5f9650a148f0d181926445ebff8b43c9e2e
	round_object_coord(uint32_t(X_obj), uint32_t(Y_obj));
}


<<<<<<< HEAD
void ObjectDetection::round_object_coord(uint16_t X_obj,uint16_t Y_obj)
{	
	if(X_obj < 6*DIS_PER_BLOCK && Y_obj < 6*DIS_PER_BLOCK)
	{
		uint16_t x_coord = X_obj / DIS_PER_BLOCK;
		uint16_t y_coord = Y_obj / DIS_PER_BLOCK;
=======
void ObjectDetection::round_object_coord(uint32_t X_obj,uint32_t Y_obj)
{
	distance_per_block = ??? 				// this is going to be in whatever the lidar sensors are in? or in mm?
	MAX_X = Y_MAX = 6*distance_per_block; 	//or we get this experimentally
	
	if( X < MAX_X && Y < MAX_Y)
	{
		x_coord = X / distance_per_block;
		y_coord = Y / distance_per_block;
>>>>>>> 9072e5f9650a148f0d181926445ebff8b43c9e2e
		objects[y_coord*6 + x_coord] = true;
		confidence[y_coord*6 + x_coord] +=1; //increase the confidence of an object in that block
	}
}