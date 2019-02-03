//This is the class wrapper for the TCS3200 color sensor

#include "TCS3200.h"

//CONSTRUCTORS_____________________________________________________________________________________________
TCS3200::TCS3200()
{

}

//constructor with configurations outside of default
//@param freq_gain - 1 MAX freq, 2 - MED freq, 3 - LOW freq
TCS3200::TCS3200(uint8_t freq_gain)
{

}

//PUBLIC_________________________________________________________________________________________________________

void TCS3200::initialize()
{
    //initialize the shit
}

// returns the raw sensor data for the red filter
uint16_t TCS3200::read_red()
{

}

// returns the raw sensor data for the green filter
uint16_t TCS3200::read_green()
{

}

// returns the raw sensor data for the blue filter
uint16_t TCS3200::read_blue()
{

}

// returns the current terrain based on the sensor data, and the experimental averages and STDEVs
uint8_t TCS3200::curr_terrain()
{

}



//PRIVATE