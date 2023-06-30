#include "Arduino.h"

class MotorControl{
private:
    int dir_pin[2];
    int pwm_pin;
public:
    MotorControl(int a, int b, int pwm): dir_pin{a, b}, pwm_pin{pwm} {
        pinMode(dir_pin[0], OUTPUT);
        pinMode(dir_pin[1], OUTPUT);
        pinMode(pwm_pin, OUTPUT);
    }
    void write(int speed);
    void beep(int freq, int time);


};