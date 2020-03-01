#include "Arduino.h"
#include "DRV8833_MOTOR.h"



DRV8833_MOTOR::DRV8833_MOTOR(int input1, int input2) {
    __input1 = input1;
    __input2 = input2;

    pinMode(__input1, OUTPUT);
    pinMode(__input2, OUTPUT);

    digitalWrite(__input1, LOW);
    digitalWrite(__input2, LOW);

};

DRV8833_MOTOR::~DRV8833_MOTOR() {
    DRV8833_MOTOR::stop();
};

void DRV8833_MOTOR::forward(int speed) {    
    // motorSpeed = map(reading, 0, 1023, 0, 255);
    analogWrite(__input1, speed);
    digitalWrite(__input2, LOW);
};

void DRV8833_MOTOR::reverse(int speed) {
    digitalWrite(__input1, LOW);
    analogWrite(__input2, speed);
};

void DRV8833_MOTOR::stop() {
    digitalWrite(__input1, HIGH);
    digitalWrite(__input2, HIGH);
};