
#include "Arduino.h"
#include "Wire.h" // For I2C

#ifndef SLF06_FLOW_SENSOR_h
#define SLF06_FLOW_SENSOR_h

typedef struct flow
{
	float flow;
	float temp;
	int aux;
  bool bubbleDetected;
} flow_data;


class SLF06_FLOW_SENSOR {
  private:
    const int DEVICE_ADDRESS = 0x08;
    const float SCALE_FACTOR_FLOW = 500.0; // Scale Factor for flow rate measurement
    const float SCALE_FACTOR_TEMP = 200.0; // Scale Factor for temperature measurement
    const uint16_t SLF3S_STOP_CONTINUOUS_READ_COMMAND = 0x3ff9;
    const uint16_t SLF3S_START_CONTINUOUS_READ_COMMAND_WATER = 0x3608;
    const uint16_t SLF3S_START_CONTINUOUS_READ_COMMAND_ALCOHAL = 0x3616;
    const uint16_t SLF3S_START_SOFT_RESET = 0x0006;
    
  public:
    SLF06_FLOW_SENSOR();
    ~SLF06_FLOW_SENSOR();

    void stopContinousRead();
    flow_data continuousRead();
    void softReset();
    bool bubbleDetected();

};

#endif
