#include "MotorControl.h"

void MotorControl::write(int speed){
    if (speed > 0){
        digitalWrite(dir_pin[0], HIGH);
        digitalWrite(dir_pin[1], LOW);
    }else{
        digitalWrite(dir_pin[1], HIGH);
        digitalWrite(dir_pin[0], LOW);
        speed = speed*-1;
    }
    speed = min(speed, 254);
    analogWrite(pwm_pin, speed);
}

void MotorControl::beep(int vol, int time){
    write(vol);
    delay(time);
    write(0);
}