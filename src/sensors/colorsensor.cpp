//This is the class wrapper for the color sensor

#include "colorsensor.h"
#include <Arduino.h>

//CONSTRUCTOR_____________________________________________________________________________________________
//@param freq_gain - 1 MAX freq (500->600kHz), 2 - MED freq (100->120kHz), 3 - LOW freq (10->12kHz)
ColorSensor::ColorSensor(freqGainLevel freq_gain = FREQ_GAIN_LOW, uint16_t delay_time = 100)
{
    freq_gain_ = freq_gain;
    delay_time_ = delay_time;
}

//PUBLIC_________________________________________________________________________________________________________

void ColorSensor::initialize()
{
    r=0,g=0,b=0;
     // Setting the outputs
    pinMode(COLOR_SENSOR_PIN0, OUTPUT);
    pinMode(COLOR_SENSOR_PIN1, OUTPUT);
    pinMode(COLOR_SENSOR_PIN2, OUTPUT);
    pinMode(COLOR_SENSOR_PIN3, OUTPUT);
    
    // Setting the COLOR_SENSOR_OUT as an input
    pinMode(COLOR_SENSOR_OUT, INPUT);
    
    // Setting frequency scaling to 20% by default
    digitalWrite(COLOR_SENSOR_PIN0, LOW);
    digitalWrite(COLOR_SENSOR_PIN1, HIGH);

     // Set the frequency scaling for the sensor
    if(freq_gain_ == 1)
    {
        digitalWrite(COLOR_SENSOR_PIN0, HIGH);
        digitalWrite(COLOR_SENSOR_PIN1, HIGH);
    }
    else if(freq_gain_ == 2)
    {
        digitalWrite(COLOR_SENSOR_PIN0, HIGH);
        digitalWrite(COLOR_SENSOR_PIN1, LOW);
    }
    else if(freq_gain_ == 3)
    {
        digitalWrite(COLOR_SENSOR_PIN0, LOW);
        digitalWrite(COLOR_SENSOR_PIN1, HIGH);
    }
}

// returns the raw sensor data for the red filter
uint16_t ColorSensor::read_red()
{
// Setting RED (R) filtered photodiodes to be read
  digitalWrite(COLOR_SENSOR_PIN2, LOW);
  digitalWrite(COLOR_SENSOR_PIN3, LOW);
  // Delay to prevent the sensor from being read too often
  //delay(delay_time_);
  // Reading the output frequency
  // unsure how accurate this function is....
  return pulseIn(COLOR_SENSOR_OUT, LOW);
}

// returns the raw sensor data for the green filter
uint16_t ColorSensor::read_green()
{
// Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(COLOR_SENSOR_PIN2, HIGH);
  digitalWrite(COLOR_SENSOR_PIN3, HIGH);
  // Delay to prevent the sensor from being read too often
  //delay(delay_time_);
  // Reading the output frequency
  return pulseIn(COLOR_SENSOR_OUT, LOW);
}

// returns the raw sensor data for the blue filter
uint16_t ColorSensor::read_blue()
{
// Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(COLOR_SENSOR_PIN2, LOW);
  digitalWrite(COLOR_SENSOR_PIN3, HIGH);
  // Delay to prevent the sensor from being read too often
  //delay(delay_time_);
  // Reading the output frequency
  return pulseIn(COLOR_SENSOR_OUT, LOW);
}

// returns the current terrain based on the sensor data, and the experimental averages and STDEVs
// 0 -> grav, 1 -> water, 2 -> wood, 3-> sand
void ColorSensor::read_terrain(uint16_t distance, bool debug = false)
{
    r = read_red();
    b = read_blue();
    g = read_green(); 
    uint8_t min_index = 0;

    int32_t error[3] = {};
    
    for(int i = 0; i < 3; i++)
    {
        //calculate the Mag^2 of the 3D vector error of the RGB value
        error[i] = (r - AVG_R[i])*(r - AVG_R[i]) + (g - AVG_G[i])*(g - AVG_G[i]) + (b - AVG_B[i])*(b - AVG_B[i]);
        if(error[min_index] >= error[i])
        {
            min_index = i;
        }
    }
    if(debug)
    {
        DEBUG_PRINT(r);
        DEBUG_PRINT(",");
        DEBUG_PRINT(g);
        DEBUG_PRINT(",");
        DEBUG_PRINT(b);
        
        // the print order is gravel, water, wood, sand
        for( int j = 0; j<3; j++)
        {
            //DEBUG_PRINT(error[j]);
            //DEBUG_PRINT(",");
        }
        DEBUG_PRINT();
    }
    if(error[min_index] < dev[min_index])
    {
        curr_terrain = (Terrain)min_index;
    }
    else
    {
        curr_terrain = Terrain::ERROR;
    }

    //Update the terrain based on the distance reading

    //edge of a tile if the distance is too large for wood
    if(distance > WATER_MIN_HEIGHT)
    {
        curr_terrain = Terrain::WATER;
    }
    DEBUG_PRINT((uint8_t)curr_terrain);

}

//PRIVATE__________________________________________________________________________________________