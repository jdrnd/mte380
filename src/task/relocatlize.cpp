#include <Arduino.h>
#include <relocalize.h>


static const uint8_t TURN_SPEED = 20;
static const uint8_t NOISE_EST = 5;
static const uint8_t MAX_DIFF = 30;
static const uint8_t COUNTS = 5;
//static const uint8_t DIFF_SIZE = 8;

static uint16_t prev_value;
static uint8_t count;

void start_relocalization() {
    prev_value = 60000;
    count = 0;
    
    motors.left->stop();
    motors.right->stop();

    delay(10);
    
}

bool relocalize() {
    motors.left->setSpeed(TURN_SPEED);
    motors.right->setSpeed(-TURN_SPEED);
    
    if(rangefinders.front.readings_.size() == 0)
        return false;
    
    int16_t diff = rangefinders.front.readings_.last() - prev_value;

    PLOTTER_SERIAL.print(String(rangefinders.front.readings_.last()) + "," + String(count) +"\n");
    
    if(diff < NOISE_EST && diff > - MAX_DIFF)
        count++;
    else if(count >= COUNTS && diff > 0 && diff < MAX_DIFF)
    {
        motors.left->stop();
        motors.right->stop();
        return true;
    }
    else
        count = 0;

    prev_value = rangefinders.front.readings_.last();
    return false;
}
