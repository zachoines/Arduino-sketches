#include "Arduino.h"
#include "Wire.h" // For I2C
#include "SLF06_FLOW_SENSOR.h"


SLF06_FLOW_SENSOR::SLF06_FLOW_SENSOR() {
	
	SLF06_FLOW_SENSOR::softReset();	
};

SLF06_FLOW_SENSOR::~SLF06_FLOW_SENSOR() {

};

void SLF06_FLOW_SENSOR::stopContinousRead() {
	// To stop the continuous measurement, first send 0x3FF9.
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write(0x3F);
    Wire.write(0xF9);
    if (!Wire.endTransmission()) {
      Serial.println("Error during write measurement mode command");
    }
};

flow_data SLF06_FLOW_SENSOR::continuousRead() {
    uint16_t aux_value;
    uint16_t sensor_flow_value;
    uint16_t sensor_temp_value;
    int16_t signed_flow_value;
    int16_t signed_temp_value;
    float scaled_flow_value;
    float scaled_temp_value;
    byte aux_crc;
    byte sensor_flow_crc;
    byte sensor_temp_crc;

	Wire.beginTransmission(DEVICE_ADDRESS);

	Wire.write(0x36);
	Wire.write(0x08);
	
	flow_data data;
	
	if (!Wire.endTransmission()) {
		Serial.println("Error during write measurement mode command");
		return data;
	}

	delay(1000);
	Wire.requestFrom(DEVICE_ADDRESS, 9);
	if (Wire.available() < 9) {
		Serial.println("Error while reading flow measurement");
		return data;
	}

	sensor_flow_value  = Wire.read() << 8; 
	sensor_flow_value |= Wire.read();      
	sensor_flow_crc    = Wire.read();
	sensor_temp_value  = Wire.read() << 8; 
	sensor_temp_value |= Wire.read();      
	sensor_temp_crc    = Wire.read();
	aux_value          = Wire.read() << 8; 
	aux_value         |= Wire.read();      
	aux_crc            = Wire.read();

	signed_flow_value = (int16_t) sensor_flow_value;
	scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
	signed_temp_value = (int16_t) sensor_temp_value;
	scaled_temp_value = ((float) signed_temp_value) / SCALE_FACTOR_TEMP;
	
	data.flow = scaled_flow_value;
	data.temp = scaled_temp_value;
	data.aux = aux_value;

	return data;
};

void SLF06_FLOW_SENSOR::softReset() {

	int num_tries = 0;

	do {
		num_tries++;
		// Soft reset the sensor
		Wire.beginTransmission(0x00);
		Wire.write(0x06);

		if (num_tries == 5) {
			Serial.println("Error while sending soft reset command, retrying...");
			return;
		}

		delay(500); // Wait for reset to complete
		

	} while (!Wire.endTransmission());

  	delay(50); // Wait for flow sensor heater
};

bool SLF06_FLOW_SENSOR::bubbleDetected() {
	return false;
};