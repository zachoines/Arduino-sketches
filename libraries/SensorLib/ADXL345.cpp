#include "ADXL345.h"

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"


ADXL::ADXL(){
  pinMode(calibrateButton, INPUT_PULLUP);
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  Serial.begin(9600);
 
  pinMode(CS, OUTPUT);
  
  digitalWrite(CS, HIGH);

  //zero offset 
  ADXL::writeRegister(OFSX,0x00); 
  ADXL::writeRegister(OFSY,0x00); 
  ADXL::writeRegister(OFSZ,0x00); 
 
  // writeRegister(DATA_FORMAT,0b000010);


  // Init the device
  ADXL::writeRegister(POWER_CTL, 0);  //Wake 
  ADXL::writeRegister(POWER_CTL, 16);  //Auto_Sleep
  writeRegister(POWER_CTL, 8);  //Measurement

  // Set the measurement mode to 2g. Highly sensitive.
  setRange(RANGE_2_G);

  // Set to four wire SPI mode
  setSpiBit(0);

  // // Set movement detection. Activity detection threshhold is 0 - 255 at 62.5mg per increment  
  // setActivityXYZ(1, 1, 1);  
  // setActivityThreshold(75);
}



void ADXL::writeRegister(char registerAddress, char value){

  digitalWrite(CS, LOW);

  SPI.transfer(registerAddress);

  SPI.transfer(value);

  digitalWrite(CS, HIGH);
  delay(10);
}


void ADXL::readRegister(char registerAddress, int numBytes, byte  values[]){

  char address = 0x80 | registerAddress;
  if(numBytes > 1)address = address | 0x40;
 
  digitalWrite(CS, LOW);
  SPI.transfer(address);

  for(int i=0; i< numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  
  digitalWrite(CS, HIGH);
  delay(10);
}

void ADXL::setRange(range_t range) {
    byte values[2];
    ADXL::readRegister(DATA_FORMAT, 2, values);
    uint8_t format = (values[1]<<8) | values[0];

    // Update the data rate
    format &= ~0x0F;
    format |= range;

    // Ensure setting of the FULL-RES 
    format |= 0x08;

    /* Write the register back to the IC */
    writeRegister(DATA_FORMAT, format);

    _range = range;

    
}

void ADXL::setSpiBit(bool spiBit) {
  ADXL::setRegisterBit(DATA_FORMAT, 6, spiBit);
}

void ADXL::setRegisterBit(byte regAdress, int bitPos, bool state) {
  byte _b;
  ADXL::readRegister(regAdress, 1, &_b);
  if (state) {
    _b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.  
  } 
  else {
    _b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
  }
  ADXL::writeRegister(regAdress, _b);  
}

void ADXL::setActivityX(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state); 
}

void ADXL::setActivityY(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state); 
}

void ADXL::setActivityZ(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state); 
}

void ADXL::setActivityXYZ(bool stateX, bool stateY, bool stateZ) {
  ADXL::setActivityX(stateX);
  ADXL::setActivityY(stateY);
  ADXL::setActivityZ(stateZ);
}

void ADXL::setActivityThreshold(int activityThreshold) {
  activityThreshold = constrain(activityThreshold,0,255);
  byte _b = byte (activityThreshold);
  ADXL::writeRegister(ADXL345_THRESH_ACT, _b);  
}

struct accel_data ADXL::readAccel() {
  byte values[10];
  double x, y, z;
  int16_t x_int, y_int, z_int;
  struct accel_data p;
  
  //The results of the read operation will get stored to the values[] buffer.
  ADXL::readRegister(DATAX0, 6, values);

  
  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  p.x = 0.004 * 9.80665 * (float)(int16_t)((int)(values[1]<<8)|values[0]);
  p.y = 0.004 * 9.80665 * (float)(int16_t)((int)(values[3]<<8)|values[2]);
  p.z = 0.004 * 9.80665 * (float)(int16_t)((int)(values[5]<<8)|values[4]);

  return p;
}

struct accel_data ADXL::readAccelAdjusted() {
  struct accel_data p = ADXL::readAccel();
  
  if (!calibrated) {
     return p;
  }
  
  p.x = map(p.x, AccelMinX, AccelMaxX, 0.004 * 9.80665 * -1.0  * _range, 0.004 * 9.80665 * _range);
  p.y = map(p.y, AccelMinY, AccelMaxY, 0.004 * 9.80665 * -1.0  * _range, 0.004 * 9.80665 * _range);
  p.z = map(p.z, AccelMinZ, AccelMaxZ, 0.004 * 9.80665 * -1.0  * _range, 0.004 * 9.80665 * _range);

  return p;
}

// Perform simple two-point calibration
struct calibration_data ADXL::performCalibration()  {

  while(ADXL::calibrateRequest());

  
  // Perform x-axis calibration
  Serial.print("Please rotate to new axis. Quickly press button when ready. \n");
  while(!ADXL::calibrateRequest());
  for (int i = 0; i < 100; i++) {
     calibrate();
  }
  while(ADXL::calibrateRequest()); 


  // Perform y-axis calibration
  Serial.print("Please rotate to new axis. Quickly press button when ready. \n");
  while(!ADXL::calibrateRequest());
  for (int i = 0; i < 100; i++) {
     ADXL::calibrate();
  }
  while(ADXL::calibrateRequest()); 


  // Perform z-axis calibration
  Serial.print("Please rotate to new axis. Quickly press button when ready. \n");
  while(!ADXL::calibrateRequest());
  for (int i = 0; i < 100; i++) {
     ADXL::calibrate();
  }
  while(ADXL::calibrateRequest());  


  calibrated = true;

  
}

bool ADXL::calibrateRequest() {
  if (digitalRead(calibrateButton) == LOW) {
    return true;
  } else {
    return false;
  }
}

// Record calibration event and update global Min/Max values
struct calibration_data ADXL::calibrate() {
 
  float accX = 0;
  float accY = 0;
  float accZ = 0;

  struct accel_data p;

  struct calibration_data calibration;
                             
  p = ADXL::readAccel(); 
  accX = p.x; 
  accY = p.y; 
  accZ = p.z; 

  if(accX < AccelMinX) AccelMinX = accX;
  if(accX > AccelMaxX) AccelMaxX = accX;

  if(accY < AccelMinY) AccelMinY = accY;
  if(accY > AccelMaxY) AccelMaxY = accY;

  if(accZ < AccelMinZ) AccelMinZ = accZ;
  if(accZ > AccelMaxZ) AccelMaxZ = accZ;


  calibration.Min.x = AccelMinX;
  calibration.Min.y = AccelMinY;
  calibration.Min.z = AccelMinZ;

  calibration.Max.x = AccelMaxX;
  calibration.Max.y = AccelMaxY;
  calibration.Max.z = AccelMaxZ;


  return calibration;
  
}
