
#include "Arduino.h" // For I2C
#include "Wire.h" // For I2C

#ifndef RGB_LED_h
#define RGB_LED_h 

enum Color { RED, GREEN, BLUE };

class RGB_LED {
  private:
    int __GREENPIN;
    int __REDPIN;
    int __BLUEPIN;
    int __r_brightness;
    int __g_brightness;
    int __b_brightness;

  public:
    RGB_LED(int R_PIN, int G_PIN, int B_PIN);
    ~RGB_LED();

    void turnOn(Color color, float brigtness);
    void turnOn(float brightness);
    void RGB_Cycle();
    void turnOff();

};

#endif
