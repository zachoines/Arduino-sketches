#include "Arduino.h"
#include "Sonar.h"

float Sonar::detect() {
  float hum;    // Stores humidity value in percent
  float temp;   // Stores temperature value in Celcius
  float duration; // Stores HC-SR04 pulse duration value
  float distance; // Stores calculated distance in cm 
  float soundsp;  // Stores calculated speed of sound in M/S
  float soundcm;  // Stores calculated speed of sound in cm/ms
  int iterations = 5;
   
  hum = dht->readHumidity();  // Get Humidity value
  temp = dht->readTemperature();  // Get Temperature value
    
  // Calculate the Speed of Sound in M/S
  soundsp = 331.4 + (0.606 * temp) + (0.0124 * hum);
    
  // Convert to cm/ms  
  soundcm = soundsp / 10000.0;
  
  // Measure duration for first sensor
  duration = sonar->ping_median(iterations);

  
  // Calculate the distances for both sensors
  distance = (duration / 2.0) * soundcm;
 

  if (distance >= 400.0 || distance <= 0.0) {
    return -1;
  }
  else {
    return distance;
  }
  
};

Sonar::Sonar(int TRIGGER_PIN, int ECHO_PIN, DHT * dht) {
  this->sonar = new NewPing(TRIGGER_PIN, ECHO_PIN, 400);
  this->dht = dht;
};

Sonar::~Sonar() {
  delete sonar;
};
