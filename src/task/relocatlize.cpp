#include <Arduino.h>
#include <relocalize.h>
#include <motor_control.h>

#define NUM 4

static const uint8_t TURN_SPEED = 20;
static const uint8_t DIFF_SIZE = 8;
static const float THRESHOLD = 2.0;
static uint16_t prev_value[4];
static int8_t index;
static int16_t diffs[DIFF_SIZE][4];
static int16_t sum[4];
static bool buffer_ready;

static const uint16_t FIELD_WIDTH = 1829;
static const uint16_t FIELD_TOL = 50;

void start_relocalization() {
    for(size_t i = 0; i < NUM; i++) {
        prev_value[i] = 60000;
        sum[i] = 0;
    }
    buffer_ready = false;

    MotorControl::send_command(Command_t::TURN, -9000);
}

bool relocalize() {
    if (!normalize())
        return false;
    // get x,y maybe?
    // this might be done by the localization task. 
    return true;
}

bool normalize() {
    // TODO @Jordan Slater make this use all range finders
    //motors.left->setSpeed(TURN_SPEED);
    //motors.right->setSpeed(-TURN_SPEED);
    
    if(rangefinders.right.readings_.size() == 0)
        return false;
    else if(rangefinders.right.readings_.size() == 1) {
        prev_value[0] = rangefinders.right.readings_.last();
        prev_value[1] = rangefinders.front.readings_.last();
        prev_value[2] = rangefinders.left.readings_.last();
        prev_value[3] = rangefinders.back.readings_.last();
        return false;
    }
    int16_t diff[4]; 
    diff[0] = rangefinders.right.readings_.last() - prev_value[0];
    diff[1] = rangefinders.front.readings_.last() - prev_value[1];
    diff[2] = rangefinders.left.readings_.last() - prev_value[2];
    diff[3] = rangefinders.back.readings_.last() - prev_value[3];

    // put the diff in the diffs
    for(size_t i = 0; i < NUM; i++) {
        diffs[index][i] = diff[i];
        sum[i] += diff[i];
        if (index >= 0)
            sum[i] -= diffs[index - 1][i];
        else
            sum[i] -= diffs[BUFF_SIZE - 1][i];
    }
    index = (index + 1) % BUFF_SIZE;
    if (index == 0)
        buffer_ready = true;

    PLOTTER_SERIAL.print(String(rangefinders.front.readings_.last()) + "," + String(sum[1] / BUFF_SIZE) + "," + String(diff[1]) + "\n");

    if (buffer_ready) {
        if (sum[1] > - BUFF_SIZE * THRESHOLD && sum[1] > BUFF_SIZE * THRESHOLD) {
            MotorControl::stopMotors();
            //return true;
        }
    }
    
    prev_value[0] = rangefinders.right.readings_.last();
    prev_value[1] = rangefinders.front.readings_.last();
    prev_value[2] = rangefinders.left.readings_.last();
    prev_value[3] = rangefinders.back.readings_.last();
    return false;
}
