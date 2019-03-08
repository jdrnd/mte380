
#include "object_detection.h"
#include <Arduino.h>

uint16_t* get_last_values(uint8_t amount, uint16_t* tail, uint16_t* buffer)
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
	// scan with this width [0][1][i][3][4][5]
    if(rangefinders.front.readings_.size() > 6)
	{
		uint16_t* R = get_last_values(6, rangefinders.right.readings_.last(),)
		// if difference between i and i-1 then check prev 2 value and next 2 for consistency
		for(index_t i = 3; i<BUFF_SIZE-3; i++)
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
}

void scan_objects_rot(uint16_t heading, uint16_t X, uint16_t Y)
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



void ObjectDetection::locate_coord_rot(uint16_t heading, LidarSensor sensor, uint16_t diff, uint16_t X, uint16_t Y)
{
	double X_obj = X + diff*cos(heading+0.5*PI*sensor);
	double Y_obj = Y + diff*sin(heading+0.5*PI*sensor);

	round_object_coord(uint32_t(X_obj), uint32_t(Y_obj));
}

void ObjectDetection::round_object_coord(uint16_t X_obj,uint16_t Y_obj)
{	
	if(X_obj < 6*DIS_PER_BLOCK && Y_obj < 6*DIS_PER_BLOCK)
	{
		uint16_t x_coord = X_obj / DIS_PER_BLOCK;
		uint16_t y_coord = Y_obj / DIS_PER_BLOCK;
		objects[y_coord*6 + x_coord] = true;
		confidence[y_coord*6 + x_coord] +=1; //increase the confidence of an object in that block
	}
}
