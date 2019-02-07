//This is the class wrapper for the TCS3200 color sensor


#ifndef _TCS3200_H_
#define _TCS3200_H_
#endif

#include <Arduino.h>

// These are the configuration pins on the Arduino Mega
// Configure these to your desires
#define S0 = 3 
#define S1 = 4
#define S2 = 5
#define S3 = 6
#define SOUT = 8

// Define the experimental constants for the color sensor
// 02/06/19 Currently the devations are not being explicitly used due to not
// having the robot as a test rig, and the potential inconsistency with the data
#define GRAV_R                    =1119
#define GRAV_RDEV                 =137
#define GRAV_G                    =1744
#define GRAV_GDEV                 =185
#define GRAV_B                    =1805
#define GRAV_BDEV                 =157 

#define WAT_R                    =727
#define WAT_RDEV                 =69
#define WAT_G                    =1286
#define WAT_GDEV                 =97
#define WAT_B                    =1398
#define WAT_BDEV                 =73

#define WOOD_R                    =465
#define WOOD_RDEV                 =14
#define WOOD_G                    =785
#define WOOD_GDEV                 =16
#define WOOD_B                    =903
#define WOOD_BDEV                 =12

#define SAND_R                    =1691
#define SAND_RDEV                 =209
#define SAND_G                    =2581
#define SAND_GDEV                 =189
#define SAND_B                    =2536
#define SAND_BDEV                 =121

class TCS3200(){
    public:
        TCS3200();

        TCS3200(uint8_t _freq_gain, uint16_t _delay_time);

        void initialize();

        uint16_t read_red();

        uint16_t read_green();

        uint16_t read_blue();

        // Returns a value 0-4 based on the terrain type, 5 if undefined
        uint8_t curr_terrain();

    private:

        uint8_t freq_gain;
        uint16_t delay_time;

        const int32_t AVG_R[4] = {GRAV_R, WAT_R, WOOD_R, SAND_R};
        const int32_t AVG_G[4] = {GRAV_G, WAT_G, WOOD_G, SAND_G};
        const int32_t AVG_B[4] = {GRAV_B, WAT_B, WOOD_B, SAND_B};
        // the hand wavey absolute error allowed for the innitial algorithm
        // will need more data to tune
        const int32_t dev[4] = {150000, 50000, 150000, 300000};
        

};
