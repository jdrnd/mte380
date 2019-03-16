
#include "object_detection.h"
#include <Arduino.h>

uint16_t* get_values(uint8_t amount, uint16_t* tail, uint16_t* buffer)
{
	uint16_t* values = new uint16_t[amount];
	//if the tail has enough data points behind it
	if(tail > buffer+amount-1)
	{
		for(uint8_t i = 0; i < amount; i++)
		{
			values[i] = *(tail-i);
		}
	}
	else
	{
		uint8_t data_at_end = amount-1-(tail-buffer);
		for(uint8_t i = 0; i < amount-data_at_end; i++)
		{
			values[i] = *(tail-i);
		}
		for(uint8_t i = amount-data_at_end; i<amount; i++)
		{
			//get the data at the end of the circular buffer
			values[i] = *(buffer+BUFF_SIZE-1-(amount-data_at_end-i));
		}
	}
	return &values[0];
}

void scan_objects_lin(uint16_t heading, uint16_t X, uint16_t Y){
	
	// scan with tail-->[0][1][POI]EDGE[3][4][5]<--old data
	// if there is enough data
    if(rangefinders.front.readings_.size() > 6)
	{
		uint16_t* R = get_values(6, rangefinders.right.readings_.get_tail(), rangefinders.right.readings_.get_buffer());
		uint16_t* L = get_values(6, rangefinders.left.readings_.get_tail(), rangefinders.left.readings_.get_buffer());

		// if difference between i and i-1 then check prev 2 value and next 2 for consistency
		uint32_t diff_R = abs(R[2] - R[3]);
		uint32_t diff_L = abs(L[2] - L[3]);
	
		if(diff_R > THRESHOLD)
		{
			//check that previous values are consistent
			if(abs(R[2]-R[4]) > THRESHOLD && abs(R[2]-R[5]) > THRESHOLD)
			{
				//check that the next two values are also a significant change to avoid noise
				if(abs(R[3]-R[1]) > THRESHOLD && abs(R[3]-R[0]) > THRESHOLD)
				{
					//locate_coord_lin(heading, LIDAR_RIGHT, min(R[3], R[2]) + RIGHT_LIDAR_OFFSET, X, Y);
				}
			}
		}
		if(diff_L > THRESHOLD)
		{
			if(abs(L[2]-L[4]) > THRESHOLD && abs(L[2]-L[5]) > THRESHOLD)
			{
				//check that the next two values are also a significant change to avoid noise
				if(abs(L[3]-L[1]) > THRESHOLD && abs(L[3]-L[0]) > THRESHOLD)
				{
					//locate_coord_lin(heading, LIDAR_RIGHT, min(L[3], L[2]) + LEFT_LIDAR_OFFSET, X, Y);
				}
			}
		}
		// delete the arrays of lidar data
		delete [] R;
		delete [] L;
	}		
}

void scan_objects_rot(uint16_t heading, uint16_t X, uint16_t Y)
{
	//if there is enough data
	if(rangefinders.front.readings_.size() > 6)
	{
		uint16_t* R = get_values(6, rangefinders.right.readings_.get_tail(), rangefinders.right.readings_.get_buffer());
		uint16_t* L = get_values(6, rangefinders.left.readings_.get_tail(), rangefinders.left.readings_.get_buffer());
		uint16_t* F = get_values(6, rangefinders.front.readings_.get_tail(), rangefinders.front.readings_.get_buffer());
		uint16_t* B = get_values(6, rangefinders.back.readings_.get_tail(), rangefinders.back.readings_.get_buffer());

		uint16_t diff_R = abs(R[3] - R[2]);
		uint16_t diff_L = abs(L[3] - L[2]);
		uint16_t diff_F = abs(F[3] - F[2]);
		uint16_t diff_B = abs(B[3] - B[2]);
		if(diff_R > THRESHOLD)
		{
            //check that previous values are consistent
			if(abs(R[2]-R[4]) > THRESHOLD && abs(R[2]-R[5]) > THRESHOLD)
			{
				//check that the next two values are also a significant change to avoid noise
				if(abs(R[3]-R[1]) > THRESHOLD && abs(R[3]-R[0]) > THRESHOLD)
				{
					//locate_coord_rot(heading, LIDAR_RIGHT, min(R[3], R[2]) + RIGHT_LIDAR_OFFSET, X, Y);
				}
			}
        }
		if(diff_L > THRESHOLD)
		{
			//check that the previous 2 values are consistent
            if(abs(L[2]-L[4]) > THRESHOLD && abs(L[2]-L[5]) > THRESHOLD)
			{
				//check that the next two values are also a significant change to avoid noise
				if(abs(L[3]-L[1]) > THRESHOLD && abs(L[3]-L[0]) > THRESHOLD)
				{
					//locate_coord_rot(heading, LIDAR_RIGHT, min(L[3], L[2]) + LEFT_LIDAR_OFFSET, X, Y);
				}
			}
        }
		if(diff_F > THRESHOLD)
		{
            //check that the previous 2 values are consistent
            if(abs(F[2]-F[4]) > THRESHOLD && abs(F[2]-F[5]) > THRESHOLD)
			{
				//check that the next two values are also a significant change to avoid noise
				if(abs(F[3]-F[1]) > THRESHOLD && abs(F[3]-F[0]) > THRESHOLD)
				{
					//locate_coord_rot(heading, LIDAR_RIGHT, min(F[3], F[2]) + FRONT_LIDAR_OFFSET, X, Y);
				}
			}
        }
		if(diff_B > THRESHOLD)
		{
            //check that the previous 2 values are consistent
            if(abs(B[2]-B[4]) > THRESHOLD && abs(B[2]-B[5]) > THRESHOLD)
			{
				//check that the next two values are also a significant change to avoid noise
				if(abs(B[3]-B[1]) > THRESHOLD && abs(B[3]-B[0]) > THRESHOLD)
				{
					//locate_coord_rot(heading, LIDAR_RIGHT, min(B[3], B[2]) + BACK_LIDAR_OFFSET, X, Y);
				}
			}
        }
		// delete the newly allocated data
		delete [] R;
		delete [] L;
		delete [] F;
		delete [] B;
	}		
}

// heavily relies on us being able to maintain a cardinal direction
void locate_coord_lin(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y)
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



void locate_coord_rot(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y)
{
	double X_obj = X + diff*cos(heading*2*PI/180+0.5*PI*sensor);
	double Y_obj = Y + diff*sin(heading*2*PI/180+0.5*PI*sensor);

	round_object_coord(uint32_t(X_obj), uint32_t(Y_obj));
}

void round_object_coord(uint16_t X_obj,uint16_t Y_obj)
{	
	if(X_obj < 6*DIS_PER_BLOCK && Y_obj < 6*DIS_PER_BLOCK)
	{
		uint16_t x_coord = X_obj / DIS_PER_BLOCK;
		uint16_t y_coord = Y_obj / DIS_PER_BLOCK;
		objects[y_coord*6 + x_coord] = true;
		confidence[y_coord*6 + x_coord] +=1; //increase the confidence of an object in that block
	}
}
