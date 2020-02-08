
#include "Arduino.h"
#include "DHT.h"
#include "NewPing.h"


#ifndef Sonar_h
#define Sonar_h

class Sonar {
  private:
    NewPing * sonar;
    DHT * dht;
    
  public:
    Sonar(int TRIGGER_PIN, int ECHO_PIN, DHT * dht);
    ~Sonar();
    float detect();

};

#endif

