
#include "Arduino.h"

#ifndef L293D_MOTOR_h
#define L293D_MOTOR_h

class L293D_MOTOR {
  private:
    int __enable; 
    int __input1;
    int __input2;
    // embed::PwmOut* __pwm;
    
  public:
    L293D_MOTOR(int enable, int input1, int input2);
    ~L293D_MOTOR();
    
    void forward(int speed);
    void reverse(int speed);
    void stop();

};

#endif
