//This is the class wrapper for the COLOR_SENSOR color sensor

#include "color_sensor.h"

//CONSTRUCTORS_____________________________________________________________________________________________
COLOR_SENSOR::COLOR_SENSOR()
{
   _freq_gain = FREQ_GAIN_LOW;    //LOWEST SETTING BY DEFAULT
   _delay_time = 100;             //in ms
}

//constructor with configurations outside of default
//@param freq_gain - 1 MAX freq (500->600kHz), 2 - MED freq (100->120kHz), 3 - LOW freq (10->12kHz)
COLOR_SENSOR::COLOR_SENSOR(freqGain freq_gain_, uint16_t delay_time_)
{
    _freq_gain = freq_gain;
    _delay_time = delay_time;
}

//PUBLIC_________________________________________________________________________________________________________

void COLOR_SENSOR::initialize()
{
     // Setting the outputs
    pinMode(COLOR_SENSOR_PIN0, OUTPUT);
    pinMode(COLOR_SENSOR_PIN1, OUTPUT);
    pinMode(COLOR_SENSOR_PIN2, OUTPUT);
    pinMode(COLOR_SENSOR_PIN3, OUTPUT);
    
    // Setting the COLOR_SENSOR_OUT as an input
    pinMode(COLOR_SENSOR_OUT, INPUT);
    
    // Setting frequency scaling to 20% by default
    digitalWrite(COLOR_SENSOR_PIN0,LOW);
    digitalWrite(COLOR_SENSOR_PIN1,HIGH);

     // Set the frequency scaling for the sensor
    if(freq_gain == 1)
    {
        digitalWrite(COLOR_SENSOR_PIN0,HIGH);
        digitalWrite(COLOR_SENSOR_PIN1,HIGH);
    }
    else if(freq_gain == 2)
    {
        digitalWrite(COLOR_SENSOR_PIN0,HIGH);
        digitalWrite(COLOR_SENSOR_PIN1,LOW);
    }
    else if(freq_gain == 3)
    {
        digitalWrite(COLOR_SENSOR_PIN0,LOW);
        digitalWrite(COLOR_SENSOR_PIN1,HIGH);
    }
}

// returns the raw sensor data for the red filter
uint16_t COLOR_SENSOR::read_red()
{
// Setting RED (R) filtered photodiodes to be read
  digitalWrite(COLOR_SENSOR_PIN2,LOW);
  digitalWrite(COLOR_SENSOR_PIN3,LOW);
  // Delay to prevent the sensor from being read too often
  delay(_delay_time);
  // Reading the output frequency
  // unsure how accurate this function is....
  return pulseIn(COLOR_SENSOR_OUT, LOW);
}

// returns the raw sensor data for the green filter
uint16_t COLOR_SENSOR::read_green()
{
// Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(COLOR_SENSOR_PIN2,HIGH);
  digitalWrite(COLOR_SENSOR_PIN3,HIGH);
  // Delay to prevent the sensor from being read too often
  delay(_delay_time);
  // Reading the output frequency
  return pulseIn(COLOR_SENSOR_OUT, LOW);
}

// returns the raw sensor data for the blue filter
uint16_t COLOR_SENSOR::read_blue()
{
// Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(COLOR_SENSOR_PIN2,LOW);
  digitalWrite(COLOR_SENSOR_PIN3,HIGH);
  // Delay to prevent the sensor from being read too often
  delay(_delay_time);
  // Reading the output frequency
  return pulseIn(COLOR_SENSOR_OUT, LOW);

}

// returns the current terrain based on the sensor data, and the experimental averages and STDEVs
uint8_t COLOR_SENSOR::curr_terrain()
{
    uint16_t r = read_red();
    uint16_t b = read_blue();
    uint16_t g = read_green(); 
    uint8_t min_index = 0;
    
    for(int i = 0; i < 4; i++)
    {
        //calculate the Mag^2 of the 3D vector error of the RGB value
        error[i] = (r - AVG_R[i])*(r - AVG_R[i]) + (g - AVG_G[i])*(g - AVG_G[i]) + (b - AVG_B[i])*(b - AVG_B[i]);
        if(error[min_index] >= error[i])
        {
        min_index = i;
        }
    }

    if(error[min_index] < dev[min_index])
    {
        return min_index;
    }
    else
    {
        return 5;
    }

}

//PRIVATE__________________________________________________________________________________________