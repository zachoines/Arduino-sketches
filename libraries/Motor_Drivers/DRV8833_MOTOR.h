#include "Arduino.h"

#ifndef DRV8833_MOTOR_h
#define DRV8833_MOTOR_h

class DRV8833_MOTOR {
  private:
    int __input1;
    int __input2;
    
  public:
    DRV8833_MOTOR(int input1, int input2);
    ~DRV8833_MOTOR();
    
    void forward(int speed);
    void reverse(int speed);
    void stop();

};

#endif