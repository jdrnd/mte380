//This is the class wrapper for the TCS3200 color sensor


#ifndef _TCS3200_H_
#define _TCS3200_H_
#endif

#include <Arduino.h>

//define the experimental constants for the color sensor

class TCS3200(){
    public:
        TCS3200();

        TCS3200(uint8_t freq_gain);

        void initialize();

        uint16_t read_red();

        uint16_t read_green();

        uint16_t read_blue();

        uint8_t curr_terrain();

    private:
        //write in the variables that configure the color sensor....

};
