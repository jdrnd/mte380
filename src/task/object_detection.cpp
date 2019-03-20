#include "object_detection.h"
#include "CircularBuffer.h"
#include <Arduino.h>

//STUFF for localizaiton
bool objects[6][6] = {0};
int16_t confidence[6][6] = {0};
uint16_t X = 0;
uint16_t Y = 0;

int16_t der_r = 0;
int16_t der_l = 0;

bool obj_r = false;
bool obj_l = false;

bool rel_r = true;
bool rel_l = true;
bool rel_f = true;
bool rel_b = true;

//hold the last few predicted values of x and y so we can go
//back in time to predict object location
CircularBuffer<uint16_t, 20, uint8_t> Xreadings;
CircularBuffer<uint16_t, 20, uint8_t> Yreadings;
/////////////////////////////// local


void localize(){
	static int8_t cnt_l = 0;
	static int8_t cnt_r = 0;
	static int8_t cnt_latest_data_use_r = 0;
	static int8_t cnt_latest_data_use_l = 0;
	static int8_t cnt_after_object_l = 0;
	static int8_t cnt_after_object_r = 0;
	static int8_t cnt_calc_delay_l = 0;
	static int8_t cnt_calc_delay_r = 0;

	static bool use_prev_data_r = true;
	static bool use_prev_data_l = true;
	static bool wait_after_object_l = false;
	static bool wait_after_object_r = false;

	uint8_t r_size = rangefinders.right.readings_.size();
	uint8_t l_size = rangefinders.left.readings_.size();
	uint8_t f_size = rangefinders.front.readings_.size();
	uint8_t b_size = rangefinders.back.readings_.size();
	
	rel_f = false;
	rel_b = false;
	rel_l = false;
	rel_r = false;

	//only update the position
	if(MotorControl::current_command.type == Command_t::DRIVE && MotorControl::current_command.status == CommandStatus::RUNNING)
	{

		//**OBJECT DETECTION**	
		//this scans for objects using an averaged derivative of the lidar data.
		if(rangefinders.front.readings_.size() > 5)
		{
			lidar_derivative();
			if(abs(der_r) > THRESHOLD)
			{
				cnt_r = 0;	
				if(der_r < -THRESHOLD && !obj_r){ 
					obj_r = true;
					cnt_calc_delay_l = 0;
				}
				else if(der_r > THRESHOLD && obj_r)
				{
					uint8_t i = find_lowest_reading_index(LidarSensor::LIDAR_RIGHT, 10);
					locate_coord_lin(LIDAR_RIGHT, rangefinders.left.readings_[l_size-1-i], Xreadings[Xreadings.size()-2-i], Yreadings[Yreadings.size()-2-i]);
					wait_after_object_r = true;
				}				
			}
			if(abs(der_l) > THRESHOLD)
			{
				cnt_l = 0;
				if(der_l < -THRESHOLD && !obj_l){ 
					obj_l = true;
					cnt_calc_delay_r = 0;
				}
				else if(der_l > THRESHOLD && obj_l)
				{
					uint8_t i = find_lowest_reading_index(LidarSensor::LIDAR_LEFT, 10);
					locate_coord_lin(LIDAR_LEFT, rangefinders.left.readings_[l_size-1-i], Xreadings[Xreadings.size()-2-i], Yreadings[Yreadings.size()-2-i]);
					wait_after_object_l = true;
				}				
			}

			//Below is a whole bunch of if statements that cause behaviour based on delays per cycle.
			
			//reset the flag if there was noise
			if(cnt_l == OBJECT_DET_HARD_RESET)
			{
				obj_l = false;
				cnt_l = 0;
			}
			if(cnt_r == OBJECT_DET_HARD_RESET) 
			{
				obj_r = false;
				cnt_r = 0;
			}
			//wait for WAIT_AFTER_OBJECT cycles after an object to update the left right coord
			if(wait_after_object_l)
			{
				cnt_after_object_l++;
				if(cnt_after_object_l == WAIT_AFTER_OBJECT)
				{
					//setting obj_l allows the coord to be updated again
					obj_l = false;
					use_prev_data_r = false;
					cnt_latest_data_use_r = 0;
					cnt_after_object_l = 0;
					wait_after_object_l = false;
				}
			}
			if(wait_after_object_r)
			{
				cnt_after_object_r++;
				if(cnt_after_object_r == WAIT_AFTER_OBJECT)
				{
					//setting obj_l allows the coord to be updated again
					obj_r = false;
					use_prev_data_r = false;
					cnt_latest_data_use_r = 0;
					cnt_after_object_r = 0;
					wait_after_object_r = false;
				}
			}
		}

	}

	//**LOCALIZE the current X,Y value**
	//check the reliability of each sensor

	if(rangefinders.front.last_reading < FRONT_LIDAR_MAX)
	{
		rel_f = true;
	}
	if(rangefinders.back.last_reading < BACK_LIDAR_MAX)
	{
		rel_b = true;
	}
	if(rangefinders.left.readings_[l_size-1-LR_DELAY*use_prev_data_l] < LEFT_LIDAR_MAX && !obj_l)
	{
		rel_l = true;
	}
	if(rangefinders.right.readings_[r_size-1-LR_DELAY*use_prev_data_r] < RIGHT_LIDAR_MAX && !obj_r)
	{
		rel_r = true;
	}

	Xreadings.push(X);
	Yreadings.push(Y);

	//calculate the coord to the left or right
	if(rel_r || rel_l)
	{
		// if both are reliable but right is closer to the wall, or if right is the only option
		// the second case essentailly prevent the coord from being updated while an object is being passed on the left side
		if((rel_r && rel_l && rangefinders.right.readings_[r_size-1-LR_DELAY*use_prev_data_r] <= rangefinders.left.readings_[l_size-1-LR_DELAY*use_prev_data_l]) ||
			(rel_r && !rel_l && !obj_l))
		{
			if(MissionControl::orientation == 0) Y = rangefinders.right.readings_[r_size-1-LR_DELAY*use_prev_data_r] + RIGHT_LIDAR_OFFSET;
			if(MissionControl::orientation == 1) X = MAX_LENGTH_COURSE - (rangefinders.right.readings_[r_size-1-LR_DELAY*use_prev_data_r] + RIGHT_LIDAR_OFFSET);
			if(MissionControl::orientation == 2) Y = MAX_LENGTH_COURSE - (rangefinders.right.readings_[r_size-1-LR_DELAY*use_prev_data_r] + RIGHT_LIDAR_OFFSET);
			if(MissionControl::orientation == 3) X = rangefinders.right.readings_[r_size-1-LR_DELAY*use_prev_data_r] + RIGHT_LIDAR_OFFSET;	 
		}
		// if both are reliable and left is closer to the wall, or if left is the only option
		// the second case essentailly prevent the coord from being updated while an object is being passed on the right side
		else if((rel_r && rel_l && rangefinders.right.readings_[r_size-1-LR_DELAY*use_prev_data_r] > rangefinders.left.readings_[l_size-1-LR_DELAY*use_prev_data_l]) ||
				(rel_l && !rel_r && !obj_r))
		{
			if(MissionControl::orientation == 0) Y = MAX_LENGTH_COURSE - rangefinders.left.readings_[l_size-1-LR_DELAY*use_prev_data_l] - LEFT_LIDAR_OFFSET;
			if(MissionControl::orientation == 1) X = rangefinders.left.readings_[l_size-1-LR_DELAY*use_prev_data_l] + LEFT_LIDAR_OFFSET;
			if(MissionControl::orientation == 2) Y = rangefinders.left.readings_[l_size-1-LR_DELAY*use_prev_data_l] + LEFT_LIDAR_OFFSET;
			if(MissionControl::orientation == 3) X = MAX_LENGTH_COURSE - rangefinders.left.readings_[l_size-1-LR_DELAY*use_prev_data_l] - LEFT_LIDAR_OFFSET;	
		}
	}
	// Calculate the coord to the front or back
	if(rel_b || rel_f)	
	{
		if((rel_f && rel_b && rangefinders.front.last_reading <= rangefinders.back.last_reading) ||
			(rel_f && !rel_b))
		{
			if(MissionControl::orientation == 0) X = MAX_LENGTH_COURSE - rangefinders.front.last_reading - FRONT_LIDAR_OFFSET;
			if(MissionControl::orientation == 1) Y = MAX_LENGTH_COURSE - rangefinders.front.last_reading - FRONT_LIDAR_OFFSET;
			if(MissionControl::orientation == 2) X = rangefinders.front.last_reading + FRONT_LIDAR_OFFSET;
			if(MissionControl::orientation == 3) Y = rangefinders.front.last_reading + FRONT_LIDAR_OFFSET;	 
		}
		else if((rel_f && rel_b && rangefinders.front.last_reading > rangefinders.back.last_reading) ||
				(rel_b && !rel_f))
		{
			if(MissionControl::orientation == 0) X = rangefinders.back.last_reading + BACK_LIDAR_OFFSET;
			if(MissionControl::orientation == 1) Y = rangefinders.back.last_reading + BACK_LIDAR_OFFSET;
			if(MissionControl::orientation == 2) X = MAX_LENGTH_COURSE - rangefinders.back.last_reading - BACK_LIDAR_OFFSET;
			if(MissionControl::orientation == 3) Y = MAX_LENGTH_COURSE - rangefinders.back.last_reading - BACK_LIDAR_OFFSET;	
		}
	}
	cnt_r++;
	cnt_l++;
	cnt_latest_data_use_r++;
	cnt_latest_data_use_l++;	
}

void lidar_derivative()
{
    uint8_t size = rangefinders.right.readings_.size();
    if(size > 5){
            der_r =( (int16_t(rangefinders.right.readings_[size-1])-int16_t(rangefinders.right.readings_[size-2]) +
                int16_t(rangefinders.right.readings_[size-2])-int16_t(rangefinders.right.readings_[size-3]) +
                int16_t(rangefinders.right.readings_[size-3])-int16_t(rangefinders.right.readings_[size-4]))/3 );
	}
    size = rangefinders.left.readings_.size();
    if(size > 5){
            der_l =( (int16_t(rangefinders.left.readings_[size-1])-int16_t(rangefinders.left.readings_[size-2]) +
                int16_t(rangefinders.left.readings_[size-2])-int16_t(rangefinders.left.readings_[size-3]) +
                int16_t(rangefinders.left.readings_[size-3])-int16_t(rangefinders.left.readings_[size-4]))/3 );
	}	
}

void locate_coord_lin(LidarSensor sensor, uint16_t diff, uint16_t x, uint16_t y)
{
	if(MissionControl::orientation == 0)
	{	
		if (sensor == LIDAR_LEFT)
			round_object_coord(x, y + diff + LEFT_LIDAR_OFFSET);
		else 
			round_object_coord(x, y - diff - RIGHT_LIDAR_OFFSET);
	}		
	else if(MissionControl::orientation == 1)
	{
		if(sensor == LIDAR_LEFT)
			round_object_coord(x - diff - LEFT_LIDAR_OFFSET, y);
		else 
			round_object_coord(x + diff + RIGHT_LIDAR_OFFSET, y);
	}		
	else if(MissionControl::orientation == 2)
	{
		if(sensor == LIDAR_LEFT)
			round_object_coord(x, y - diff - LEFT_LIDAR_OFFSET);
		else 
			round_object_coord(x, y + diff + RIGHT_LIDAR_OFFSET);
	}
	else if(MissionControl::orientation == 3)
	{
		if(sensor == LIDAR_LEFT)
			round_object_coord(x + diff + LEFT_LIDAR_OFFSET, y);
		else 
			round_object_coord(x - diff - RIGHT_LIDAR_OFFSET, y);
	}
}

void round_object_coord(uint16_t X_obj,uint16_t Y_obj)
{	
	if(X_obj < 6*DIS_PER_BLOCK && Y_obj < 6*DIS_PER_BLOCK)
	{
		uint16_t x_coord = X_obj / DIS_PER_BLOCK;
		uint16_t y_coord = Y_obj / DIS_PER_BLOCK;
		objects[y_coord][x_coord] = true;
		confidence[y_coord][x_coord] +=1; //increase the confidence of an object in that block
	}
}

uint8_t find_lowest_reading_index(LidarSensor sensor, uint8_t max)
{
	uint8_t min_dex = 0;
	if(sensor == LidarSensor::LIDAR_LEFT)
	{
		uint8_t size = rangefinders.left.readings_.size();
		uint16_t min_reading = 2000;
		for(uint8_t i = 0; i < max; i++)
		{
			if(rangefinders.left.readings_[size-1-i] < min_reading)
			{
				min_reading = rangefinders.left.readings_[size-1-i];
				min_dex = i;
			}
		}		
	}
	else if(sensor == LidarSensor::LIDAR_RIGHT)
	{
		uint8_t size = rangefinders.right.readings_.size();
		uint16_t min_reading = 2000;
		for(uint8_t i = 0; i < max; i++)
		{
			if(rangefinders.right.readings_[size-1-i] < min_reading)
			{
				min_reading = rangefinders.left.readings_[size-1-i];
				min_dex = i;
			}
		}
		
	}
	return min_dex;
}
