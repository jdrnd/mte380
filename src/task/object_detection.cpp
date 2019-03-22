#include "object_detection.h"


//STUFF for localizaiton
bool objects[6][6] = {0};
int16_t confidence[6][6] = {0};
//three most likely points for objects
Point points[3] = {{-1,-1,0},{-1,-1,0},{-1,-1,0}};
uint16_t X = 0;
uint16_t Y = 0;

uint16_t exp_range[6][2] = {{1830,1325},    //0
							{1375,1025},    //1
							{1100,725},    //2
							{775,425},    //3
							{475,125},    //4
							{0,175}};   //5

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

	if((MotorControl::current_command.type == Command_t::DRIVE && MotorControl::current_command.status == CommandStatus::RUNNING)
		|| MotorControl::current_command.type == Command_t::STOP)
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
					uint8_t i = find_lowest_reading_index(LidarSensor::LIDAR_RIGHT, 20);
						if(rangefinders.right.readings_[r_size-1-i] < RIGHT_LIDAR_MAX){
							locate_coord_lin(LIDAR_RIGHT, rangefinders.left.readings_[l_size-1-i], Xreadings[Xreadings.size()-1-i], Yreadings[Yreadings.size()-1-i]);
						}
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
					uint8_t i = find_lowest_reading_index(LidarSensor::LIDAR_LEFT, 20);
					if(rangefinders.left.readings_[l_size-1-i] < LEFT_LIDAR_MAX){
						locate_coord_lin(LIDAR_LEFT, rangefinders.left.readings_[l_size-1-i], Xreadings[Xreadings.size()-1-i], Yreadings[Yreadings.size()-1-i]);
					}
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
			// if we detect an object wait OBJ_CALC_DELAY cycles before locating the object
			// waits for the lidars to settle
			if(obj_l)
			{
				cnt_calc_delay_l++;
				if(cnt_calc_delay_l == OBJ_CALC_DELAY && rangefinders.left.last_reading < LEFT_LIDAR_MAX)
				{
					locate_coord_lin(LidarSensor::LIDAR_LEFT, rangefinders.left.last_reading, X, Y);
				}
			}
			if(obj_r)
			{
				cnt_calc_delay_r++;
				if(cnt_calc_delay_r == OBJ_CALC_DELAY && rangefinders.right.last_reading < RIGHT_LIDAR_MAX)
				{
					locate_coord_lin(LidarSensor::LIDAR_RIGHT, rangefinders.right.last_reading, X, Y);
				}

			}

			//object detetion for the front and back sensor


	

			//**LOCALIZE the current X,Y value**
			//check the reliability of each sensor
			if(rangefinders.back.last_reading < BACK_LIDAR_MAX)
			{
				if( (MissionControl::orientation == 0 &&
					rangefinders.back.last_reading < exp_range[5-MissionControl::x_pos][0] && 
					rangefinders.back.last_reading > exp_range[5-MissionControl::x_pos][1]) ||
					(MissionControl::orientation == 2 &&
					rangefinders.back.last_reading < exp_range[MissionControl::x_pos][0] && 
					rangefinders.back.last_reading > exp_range[MissionControl::x_pos][1]) ||
					(MissionControl::orientation == 1 &&
					rangefinders.back.last_reading < exp_range[5-MissionControl::y_pos][0] && 
					rangefinders.back.last_reading > exp_range[5-MissionControl::y_pos][1]) ||
					(MissionControl::orientation == 3 &&
					rangefinders.back.last_reading < exp_range[MissionControl::y_pos][0] && 
					rangefinders.back.last_reading > exp_range[MissionControl::y_pos][1]) )
				{
					rel_b = true;
				}
				else
				{
					//place the object front or behind.
				}
				
			}
			if(rangefinders.front.last_reading < FRONT_LIDAR_MAX)
			{
				if( (MissionControl::orientation == 0 &&
					rangefinders.front.last_reading < exp_range[MissionControl::x_pos][0] && 
					rangefinders.front.last_reading > exp_range[MissionControl::x_pos][1]) ||
					(MissionControl::orientation == 2 &&
					rangefinders.front.last_reading < exp_range[5-MissionControl::x_pos][0] && 
					rangefinders.front.last_reading > exp_range[5-MissionControl::x_pos][1]) ||
					(MissionControl::orientation == 1 &&
					rangefinders.front.last_reading < exp_range[MissionControl::y_pos][0] && 
					rangefinders.front.last_reading > exp_range[MissionControl::y_pos][1]) ||
					(MissionControl::orientation == 3 &&
					rangefinders.front.last_reading < exp_range[5-MissionControl::y_pos][0] && 
					rangefinders.front.last_reading > exp_range[5-MissionControl::y_pos][1]) )
				{
					rel_f = true;
				}
				else if(rel_b) // if the back lidar is reliable then front back coord is reliable, and we can place object
				{
					locate_coord_lin(LidarSensor::LIDAR_FRONT,rangefinders.front.last_reading,X,Y);
				}
				
			}
			if(rangefinders.left.readings_[l_size-1-LR_DELAY*use_prev_data_l] < LEFT_LIDAR_MAX && !obj_l)
			{
				rel_l = true;
			}
			if(rangefinders.right.readings_[r_size-1-LR_DELAY*use_prev_data_r] < RIGHT_LIDAR_MAX && !obj_r)
			{
				rel_r = true;
			}

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
			else //there is an object in the way
			{
				if(MissionControl::orientation == 0 || MissionControl::orientation == 2)
				{
					X = MissionControl::x_pos*305+152;
				}
				else // you are facing the other way
				{
					Y = MissionControl::y_pos*305+152;
				}
			}
			
		}

	}
	#ifdef LOG_OBJECT_DETECTION
	PLOTTER_SERIAL.println(String(X) + "," + String(Y) + "," + String(100*obj_l) + "," + String(150*obj_r));
	#endif

	Xreadings.push(X);
	Yreadings.push(Y);

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
		if (sensor == LIDAR_LEFT) round_object_coord(x, y + diff + LEFT_LIDAR_OFFSET);
		else if (sensor == LIDAR_RIGHT) round_object_coord(x, y - diff - RIGHT_LIDAR_OFFSET);
		else if (sensor == LIDAR_FRONT) round_object_coord(x + diff + FRONT_LIDAR_OFFSET, y);
		//too lazy to detect objects behind us, also this seems like a lot of detections if we move straight for a long time
		//else if (sensor == LIDAR_BACK) round_object_coord()
	}		
	else if(MissionControl::orientation == 1)
	{
		if(sensor == LIDAR_LEFT) round_object_coord(x - diff - LEFT_LIDAR_OFFSET, y);
		else if (sensor == LIDAR_RIGHT) round_object_coord(x + diff + RIGHT_LIDAR_OFFSET, y);
		else if (sensor == LIDAR_FRONT) round_object_coord(x, y + diff + FRONT_LIDAR_OFFSET);
	}		
	else if(MissionControl::orientation == 2)
	{
		if(sensor == LIDAR_LEFT) round_object_coord(x, y - diff - LEFT_LIDAR_OFFSET);
		else if(sensor == LIDAR_RIGHT) round_object_coord(x, y + diff + RIGHT_LIDAR_OFFSET);
		else if(sensor == LIDAR_FRONT) round_object_coord(x - diff - FRONT_LIDAR_OFFSET, y);
	}
	else if(MissionControl::orientation == 3)
	{
		if(sensor == LIDAR_LEFT) round_object_coord(x + diff + LEFT_LIDAR_OFFSET, y);
		else if (sensor == LIDAR_RIGHT) round_object_coord(x - diff - RIGHT_LIDAR_OFFSET, y);
		else if (sensor == LIDAR_FRONT) round_object_coord(x, y - diff - FRONT_LIDAR_OFFSET);
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

void find_best_points()
{
	for(int8_t i = 0; i < 6; i++)
	{
		for(int8_t j = 0; j < 6; j++)
		{
			//if we think there is an object
			if(objects[j][i])
			{
				uint16_t curr_conf = confidence[j][i];
				if(curr_conf >= points[2].confidence)
				{
					if(curr_conf >= points[1].confidence)
					{
						if(curr_conf >= points[0].confidence)
						{
							points[2] = points[1];
							points[1] = points[0];
							points[0] = {i,j,curr_conf};

						}
						else
						{
							points[2] = points[1];
							points[1] = {i,j,curr_conf};
						}
					}
					else
					{
						points[2] = {i,j,curr_conf};
					}
					
				}
			}
			
		}
	}
}