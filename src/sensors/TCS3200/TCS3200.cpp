//This is the class wrapper for the TCS3200 color sensor

#include "TCS3200.h"

//CONSTRUCTORS_____________________________________________________________________________________________
TCS3200::TCS3200()
{
   freq_gain = 3;
   delay_time = 100;
}

//constructor with configurations outside of default
//@param freq_gain - 1 MAX freq (500->600kHz), 2 - MED freq (100->120kHz), 3 - LOW freq (10->12kHz)
TCS3200::TCS3200(uint8_t _freq_gain, uint16_t _delay_time)
{
    freq_gain = _freq_gain;
    delay_time = _delay_time;
}

//PUBLIC_________________________________________________________________________________________________________

void TCS3200::initialize()
{
     // Setting the outputs
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    
    // Setting the sensorOut as an input
    pinMode(sensorOut, INPUT);
    
    // Setting frequency scaling to 20% by default
    digitalWrite(S0,LOW);
    digitalWrite(S1,HIGH);

     // Set the frquency scaling for the sensor
    if(freq_gain == 1)
    {
        digitalWrite(S0,HIGH);
        digitalWrite(S1,HIGH);
    }
    else if(freq_gain == 2)
    {
        digitalWrite(S0,HIGH);
        digitalWrite(S1,LOW);
    }
    else if(freq_gain == 3)
    {
        digitalWrite(S0,LOW);
        digitalWrite(S1,HIGH);
    }
}

// returns the raw sensor data for the red filter
uint16_t TCS3200::read_red()
{
// Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Delay to prevent the sensor from being read too often
  delay(delay_time);
  // Reading the output frequency
  // unsure how accurate this function is....
  return pulseIn(sensorOut, LOW);
}

// returns the raw sensor data for the green filter
uint16_t TCS3200::read_green()
{
// Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Delay to prevent the sensor from being read too often
  delay(delay_time);
  // Reading the output frequency
  return pulseIn(sensorOut, LOW);
}

// returns the raw sensor data for the blue filter
uint16_t TCS3200::read_blue()
{
// Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Delay to prevent the sensor from being read too often
  delay(delay_time);
  // Reading the output frequency
  return pulseIn(sensorOut, LOW);

}

// returns the current terrain based on the sensor data, and the experimental averages and STDEVs
uint8_t TCS3200::curr_terrain()
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



//PRIVATE