#include "Arduino.h"
#include "BLECommandParser.h"
#include <ArduinoBLE.h>


BLECommandParser::BLECommandParser(const char * service_name, const char * service_uuid, const char * tx_uuid, const char * rx_uuid, void (* rxCallback) (BLEDevice, BLECharacteristic)) {
    BLEService ble_service(service_uuid);
    BLEStringCharacteristic tx(tx_uuid, BLENotify, 20);
    BLEStringCharacteristic rx(rx_uuid, BLEWrite, 20);

    BLE.setLocalName(service_name);
    BLE.setAdvertisedService(ble_service);
    ble_service.addCharacteristic(tx);
    ble_service.addCharacteristic(rx);
    BLE.addService(ble_service);
    rx.setEventHandler(BLEWritten, rxCallback);
    BLE.advertise();
};


