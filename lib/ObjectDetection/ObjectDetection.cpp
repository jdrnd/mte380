
#include "objectdetection.h"
#include <Arduino.h>


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
	round_object_coord(uint32_t(X_obj), uint32_t(Y_obj));
}


void ObjectDetection::round_object_coord(uint32_t X_obj,uint32_t Y_obj)
{
	distance_per_block = ??? 				// this is going to be in whatever the lidar sensors are in? or in mm?
	MAX_X = Y_MAX = 6*distance_per_block; 	//or we get this experimentally
	
	if( X < MAX_X && Y < MAX_Y)
	{
		x_coord = X / distance_per_block;
		y_coord = Y / distance_per_block;
		objects[y_coord*6 + x_coord] = true;
		confidence[y_coord*6 + x_coord] +=1; //increase the confidence of an object in that block
	}
}