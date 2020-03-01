#include "Arduino.h"
#include <ArduinoBLE.h>

#ifndef BLECommandParser_h
#define BLECommandParser_h

class BLECommandParser {
  private:
    
    
  public:
  
    BLECommandParser(const char * service_name, const char * service_uuid, const char * tx_uuid, const char * rx_uuid, void (* rxCallback) (BLEDevice, BLECharacteristic));
    ~BLECommandParser();
    
};

#endif