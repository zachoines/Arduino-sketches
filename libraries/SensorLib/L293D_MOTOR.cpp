#include "Arduino.h"
#include "L293D_MOTOR.h"



L293D_MOTOR::L293D_MOTOR(int enable, int input1, int input2) {
    __enable = enable;
    __input1 = input1;
    __input2 = input2;

    pinMode(__input1, OUTPUT);
    pinMode(__input2, OUTPUT);
    pinMode(__enable, OUTPUT);

    //__pwm = new mbed::PwmOut(__enable);
};

L293D_MOTOR::~L293D_MOTOR() {
    L293D_MOTOR::stop();
};

void L293D_MOTOR::forward(int speed) {
    int reverse = 0;
    
    analogWrite(__enable, speed);
    digitalWrite(__input1, !reverse);
    digitalWrite(__input2, reverse);
};

void L293D_MOTOR::reverse(int speed) {
    int reverse = 1;

    analogWrite(__enable, speed);
    digitalWrite(__input1, !reverse);
    digitalWrite(__input2, reverse);
};

void L293D_MOTOR::stop() {
    analogWrite(__enable, 0);
    digitalWrite(__input1, 0);
    digitalWrite(__input2, 0);
};