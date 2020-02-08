#include "Arduino.h" 
#include "RGB_LED.h"


RGB_LED::RGB_LED(int R_PIN, int G_PIN, int B_PIN) {
    __GREENPIN = R_PIN;
    __REDPIN = G_PIN;
    __BLUEPIN = B_PIN;

    pinMode(__GREENPIN, OUTPUT);
    pinMode(__REDPIN, OUTPUT);
    pinMode(__BLUEPIN, OUTPUT);
};

RGB_LED::~RGB_LED() {
    RGB_LED::turnOff();
};

void RGB_LED::turnOn(Color color, float brightness) {

    if (brightness > 1.0) {
        brightness = 1.0;
    }

    for (int i = 0; i < (int)(256.0 * brightness); i++) {

        switch(color)
        {
            case RED: 
                analogWrite(__REDPIN, i); 
                __r_brightness = i;  
                break;
            case GREEN: 
                analogWrite(__GREENPIN, i); 
                __g_brightness = i;
                break;
            case BLUE:
                analogWrite(__BLUEPIN, i);  
                __b_brightness = i;
                break;
        }
        
        delay(10);
    }  
};

void RGB_LED::turnOn(float brightness) {

    if (brightness > 1.0) {
        brightness = 1.0;
    }

    for (int i = 0; i < (int)(256.0 * brightness); i++) {
       analogWrite(__GREENPIN, i);
       analogWrite(__REDPIN, i);
       analogWrite(__BLUEPIN, i);
 
       delay(10);
    }
       
    __r_brightness = (int)(256.0 * brightness);
    __g_brightness = (int)(256.0 * brightness);
    __b_brightness = (int)(256.0 * brightness);
};

void RGB_LED::RGB_Cycle() {
    int r, g, b;
 
    // fade from blue to violet
    for (r = 0; r < 256; r++) { 
        analogWrite(__REDPIN, r);
        delay(10);
    } 
    // fade from violet to red
    for (b = 255; b > 0; b--) { 
        analogWrite(__BLUEPIN, b);
        delay(10);
    } 
    // fade from red to yellow
    for (g = 0; g < 256; g++) { 
        analogWrite(__GREENPIN, g);
        delay(10);
    } 
    // fade from yellow to green
    for (r = 255; r > 0; r--) { 
        analogWrite(__REDPIN, r);
        delay(10);
    } 
    // fade from green to teal
    for (b = 0; b < 256; b++) { 
        analogWrite(__BLUEPIN, b);
        delay(10);
    } 
    // fade from teal to blue
    for (g = 255; g > 0; g--) { 
        analogWrite(__GREENPIN, g);
        delay(10);
    } 
};

void RGB_LED::turnOff() {
    for (int i = 0; i < 256; i++) {
        analogWrite(__GREENPIN, __r_brightness);
        analogWrite(__REDPIN, __g_brightness);
        analogWrite(__BLUEPIN, __b_brightness);
 
       
        __r_brightness -= 1;
        __g_brightness -= 1;
        __b_brightness -= 1;
        delay(10);
   }
};