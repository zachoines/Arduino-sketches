

#include <Wire.h> // Arduino library for I2C

// -----------------------------------------------------------------------------
// Sensor specific settings, adjust if needed:
// -----------------------------------------------------------------------------

const int ADDRESS = 0x08; // Sensor I2C Address
const float SCALE_FACTOR_FLOW = 500.0; // Scale Factor for flow rate measurement
const float SCALE_FACTOR_TEMP = 200.0; // Scale Factor for temperature measurement
const char *UNIT_FLOW = " ml/min"; //physical unit of the flow rate measurement
const char *UNIT_TEMP = " deg C"; //physical unit of the temperature measurement


void scanI2CDevices() {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret;

  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)

  do {
    scanI2CDevices();
    // Soft reset the sensor
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      delay(500); // wait long enough for chip reset to complete
    }
  } while (ret != 0);

  delay(50); // wait long enough for chip reset to complete
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  int ret;
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

  // To perform a measurement, first send 0x3608 to switch to continuous
  // measurement mode (H20 calibration), then read 3x (2 bytes + 1 CRC byte) from the sensor.
  // To perform a IPA based measurement, send 0x3615 instead.
  // Check datasheet for available measurement commands.
  
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.write(0x08);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error during write measurement mode command");

  } else {
    delay(1000);

    for(int i = 0; i < 10; ++i) {
      delay(100);
      Wire.requestFrom(ADDRESS, 9);
      if (Wire.available() < 9) {
        Serial.println("Error while reading flow measurement");
        continue;
      }

      sensor_flow_value  = Wire.read() << 8; // read the MSB from the sensor
      sensor_flow_value |= Wire.read();      // read the LSB from the sensor
      sensor_flow_crc    = Wire.read();
      sensor_temp_value  = Wire.read() << 8; // read the MSB from the sensor
      sensor_temp_value |= Wire.read();      // read the LSB from the sensor
      sensor_temp_crc    = Wire.read();
      aux_value          = Wire.read() << 8; // read the MSB from the sensor
      aux_value         |= Wire.read();      // read the LSB from the sensor
      aux_crc            = Wire.read();

      Serial.print("Flow value from Sensor: ");
      Serial.print(sensor_flow_value);

      signed_flow_value = (int16_t) sensor_flow_value;
      Serial.print(", signed value: ");
      Serial.print(signed_flow_value);

      scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
      Serial.print(", scaled value: ");
      Serial.print(scaled_flow_value);
      Serial.print(UNIT_FLOW);

      Serial.print(", Temp value from Sensor: ");
      Serial.print(sensor_temp_value);

      signed_temp_value = (int16_t) sensor_temp_value;
      Serial.print(", signed value: ");
      Serial.print(signed_temp_value);

      scaled_temp_value = ((float) signed_temp_value) / SCALE_FACTOR_TEMP;
      Serial.print(", scaled value: ");
      Serial.print(scaled_temp_value);
      Serial.print(UNIT_TEMP);

      Serial.println("");
    }
    // To stop the continuous measurement, first send 0x3FF9.
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x3F);
    Wire.write(0xF9);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");
    }
  }

  delay(1000); // milliseconds delay between reads (for demo purposes)
}
