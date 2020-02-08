//Add the SPI library so we can communicate with the ADXL345 sensor
#include "SPI.h"
#include "Wire.h"

struct accel_data 
{ 
   float x;
   float y;
   float z;
};

struct calibration_data
{ 
   struct accel_data Min;
   struct accel_data Max;
};


// Calibration values
float AccelMinX = 0;
float AccelMaxX = 0;
float AccelMinY = 0;
float AccelMaxY = 0;
float AccelMinZ = 0;
int AccelMaxZ = 0;
bool calibrated = false; 
int calibrateButton = 5;


// Aceleration range
float minG = -2.0;
float maxG = 2.0;

//Assign the Chip Select signal to pin 10.
int CS=10;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char ADXL345_ACT_INACT_CTL = 0x27;
char ADXL345_THRESH_ACT = 0x24;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1


char OFSX = 0x1E; //X-Axis offset
char OFSY = 0x1F; //Y-Axis offset
char OFSZ = 0x20; //Z-Axis offset



void setup(){
  pinMode(calibrateButton, INPUT_PULLUP);
  
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);
 
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  //zero offset 
  writeRegister(OFSX,0x00); 
  writeRegister(OFSY,0x00); 
  writeRegister(OFSZ,0x00); 
 
  // writeRegister(DATA_FORMAT,0b000010);


  // Init the device
  writeRegister(POWER_CTL, 0);  //Wake 
  writeRegister(POWER_CTL, 16);  //Auto_Sleep
  writeRegister(POWER_CTL, 8);  //Measurement

  // Set the measurement mode to 2g. Highly sensitive.
  setRange(16);

  // Set to four wire SPI mode
  setSpiBit(0);

  // Set movement detection. Activity detection threshhold is 0 - 255 at 62.5mg per increment  
  setActivityXYZ(1, 1, 1);  
  setActivityThreshold(75);
}



void loop(){
  struct calibration_data c;
  if (calibrateRequest()) {
      Serial.print("Calibration request initiated. \n");
      c = performCalibration();
      Serial.print("Performed calibration. \n");
  }
  
  //These variables will be used to hold the x,y and z axis accelerometer values.
  struct accel_data a = readAccelAdjusted();
  
 
  Serial.print(a.x);
  Serial.print(", next ");
  Serial.print(a.y);
  Serial.print(", next ");
  Serial.println(a.z); 
  Serial.print("\n");  
  
  delay(250);
}

void writeRegister(char registerAddress, char value){

  digitalWrite(CS, LOW);

  SPI.transfer(registerAddress);

  SPI.transfer(value);

  digitalWrite(CS, HIGH);
  delay(10);
}


void readRegister(char registerAddress, int numBytes, byte  values[]){

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

void setRange(int val) {
  byte _s;
  byte _b;
  
  switch (val) {
    case 2:  
      _s = B00000000; 
      minG = -2.0;
      maxG = 2.0;
      break;
    case 4:  
      _s = B00000001; 
      minG = -4.0;
      maxG = 4.0;
      break;
    case 8:  
      _s = B00000010; 
      minG = -8.0;
      maxG = 8.0;
      break;
    case 16: 
      minG = -16.0;
      maxG = 16.0;
      _s = B00000011; 
      break;
    default: 
      _s = B00000000;
  }
  
  readRegister(DATA_FORMAT, 1, &_b);
  _s |= (_b & B11101100);
  writeRegister(DATA_FORMAT, _s);
}

void setSpiBit(bool spiBit) {
  setRegisterBit(DATA_FORMAT, 6, spiBit);
}

void setRegisterBit(byte regAdress, int bitPos, bool state) {
  byte _b;
  readRegister(regAdress, 1, &_b);
  if (state) {
    _b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.  
  } 
  else {
    _b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
  }
  writeRegister(regAdress, _b);  
}

void setActivityX(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state); 
}

void setActivityY(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state); 
}

void setActivityZ(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state); 
}

void setActivityXYZ(bool stateX, bool stateY, bool stateZ) {
  setActivityX(stateX);
  setActivityY(stateY);
  setActivityZ(stateZ);
}

void setActivityThreshold(int activityThreshold) {
  activityThreshold = constrain(activityThreshold,0,255);
  byte _b = byte (activityThreshold);
  writeRegister(ADXL345_THRESH_ACT, _b);  
}

struct accel_data readAccel() {
  byte values[10];
  double x, y, z;
  int16_t x_int, y_int, z_int;
  struct accel_data p;
  
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  
  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  p.x = 0.004 * 9.80665 * (float)(int16_t)((int)(values[1]<<8)|values[0]);
  p.y = 0.004 * 9.80665 * (float)(int16_t)((int)(values[3]<<8)|values[2]);
  p.z = 0.004 * 9.80665 * (float)(int16_t)((int)(values[5]<<8)|values[4]);

  return p;
}

struct accel_data readAccelAdjusted() {
  struct accel_data p = readAccel();
  
  if (!calibrated) {
     return p;
  }
  
  p.x = map(p.x, AccelMinX, AccelMaxX, 0.004 * 9.80665 * minG, 0.004 * 9.80665 * maxG);
  p.y = map(p.y, AccelMinY, AccelMaxY, 0.004 * 9.80665 * minG, 0.004 * 9.80665 * maxG);
  p.z = map(p.z, AccelMinZ, AccelMaxZ, 0.004 * 9.80665 * minG, 0.004 * 9.80665 * maxG);

  return p;
}

// Perform simple two-point calibration
struct calibration_data performCalibration()  {

  while(calibrateRequest());

  
  // Perform x-axis calibration
  Serial.print("Please rotate to new axis. Quickly press button when ready. \n");
  while(!calibrateRequest());
  for (int i = 0; i < 100; i++) {
     calibrate();
  }
  while(calibrateRequest()); 


  // Perform y-axis calibration
  Serial.print("Please rotate to new axis. Quickly press button when ready. \n");
  while(!calibrateRequest());
  for (int i = 0; i < 100; i++) {
     calibrate();
  }
  while(calibrateRequest()); 


  // Perform z-axis calibration
  Serial.print("Please rotate to new axis. Quickly press button when ready. \n");
  while(!calibrateRequest());
  for (int i = 0; i < 100; i++) {
     calibrate();
  }
  while(calibrateRequest());  


  calibrated = true;

  
}

bool calibrateRequest() {
  if (digitalRead(calibrateButton) == LOW) {
    return true;
  } else {
    return false;
  }
}

// Record calibration event and update global Min/Max values
struct calibration_data calibrate() {
 
  float accX = 0;
  float accY = 0;
  float accZ = 0;

  struct accel_data p;

  struct calibration_data calibration;
                             
  p = readAccel(); 
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
