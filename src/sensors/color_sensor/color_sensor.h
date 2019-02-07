//This is the class wrapper for the COLOR_SENSOR color sensor


#ifndef _COLOR_SENSOR_H_
#define _COLOR_SENSOR_H_
#endif

#include <Arduino.h>

// These are the configuration pins on the Arduino Mega
/* 
   Pins 0-1 control the scaling of the frequency output, input, connect to digital
   Pins 2-3 control the filter the sensor uses, input, connect to digital

   The outputn pin outputs a variable frequency, and is measured using
   a given arduino function that measures the period of a signal on a 
   digital pin. 

   Ensure the sensor is connected to 5v and power
*/
#define COLOR_SENSOR_PIN0 = 3 
#define COLOR_SENSOR_PIN1 = 4
#define COLOR_SENSOR_PIN2 = 5
#define COLOR_SENSOR_PIN3 = 6
#define COLOR_SENSOR_OUT = 8

// Define the experimental constants for the color sensor
// 02/06/19 Currently the devations are not being explicitly used due to not
// having the robot as a test rig, and the potential inconsistency with the data
#define COLOR_GRAV_R                    =1119
#define COLOR_GRAV_RDEV                 =137
#define COLOR_GRAV_G                    =1744
#define COLOR_GRAV_GDEV                 =185
#define COLOR_GRAV_B                    =1805
#define COLOR_GRAV_BDEV                 =157 

#define COLOR_WAT_R                     =727
#define COLOR_WAT_RDEV                  =69
#define COLOR_WAT_G                     =1286
#define COLOR_WAT_GDEV                  =97
#define COLOR_WAT_B                     =1398
#define COLOR_WAT_BDEV                  =73

#define COLOR_WOOD_R                    =465
#define COLOR_WOOD_RDEV                 =14
#define COLOR_WOOD_G                    =785
#define COLOR_WOOD_GDEV                 =16
#define COLOR_WOOD_B                    =903
#define COLOR_WOOD_BDEV                 =12

#define COLOR_SAND_R                    =1691
#define COLOR_SAND_RDEV                 =209
#define COLOR_SAND_G                    =2581
#define COLOR_SAND_GDEV                 =189
#define COLOR_SAND_B                    =2536
#define COLOR_SAND_BDEV                 =121
 
enum freqGain : uint8_t
    {
        FREQ_GAIN_LOW                               = 3,
        FREQ_GAIN_MED                               = 2,
        FREQ_GAIN_HIGH                              = 1,
    };

class COLOR_SENSOR(){
    public:
        COLOR_SENSOR();

        COLOR_SENSOR(freqGain freq_gain, uint16_t delay_time);

        void initialize();

        uint16_t read_red();
        uint16_t read_green();
        uint16_t read_blue();
        // Returns a value 0-4 based on the terrain type, 5 if undefined
        uint8_t curr_terrain();

    private:

        freqGain _freq_gain;
        uint16_t _delay_time;

        const int32_t AVG_R[4] = {COLOR_GRAV_R, COLOR_WAT_R, COLOR_WOOD_R, COLOR_SAND_R};
        const int32_t AVG_G[4] = {COLOR_GRAV_G, COLOR_WAT_G, COLOR_WOOD_G, COLOR_SAND_G};
        const int32_t AVG_B[4] = {COLOR_GRAV_B, COLOR_WAT_B, COLOR_WOOD_B, COLOR_SAND_B};
        // the hand wavey absolute error allowed for the innitial algorithm
        // will need more data to tune
        const int32_t dev[4] = {150000, 50000, 150000, 300000};
        

};
