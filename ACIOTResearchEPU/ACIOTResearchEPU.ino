#include <Arduino.h>
#include <MKRMotorCarrier.h>

#include <TFMPlus.h>  // For TF mini Plus
#include <Wire.h>     // Arduino standard I2C/Two-Wire Library
#include <ArduinoBLE.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "DHT.h";
#include "Sonar.h"
#include "wiring_private.h"

#define MAX_ATTEMPTS 5
#define SLAVE_ADDR 9

// DHT and Sonic Sensor Variables
#define DHTPIN IN1
#define DHTTYPE DHT22
#define TRIGGER_PIN_1  A6
#define ECHO_PIN_1     A5
#define TRIGGER_PIN_2  A2
#define ECHO_PIN_2     A1


// Program variables
TFMPlus tfmP;         // Create a TFMini Plus object
char buf [100];
volatile byte pos;
volatile boolean process_it;
uint16_t tfDist;       // Distance
uint16_t tfFlux;       // Luminous flux or intensity of return signal
uint16_t tfTemp;       // Temp in °C

// UART Comms related variables
const byte numBytes = 3;
byte receivedBytes[numBytes];
byte numReceived = 0;
boolean newData = false;


DHT* dht;
Sonar* sonar1;
Sonar* sonar2;

// Incoming I2C bits from receive interrupt
uint8_t last_command = 0;
uint8_t last_data = 0;

// Keep track of current actuator values
byte servoOneAngle = 0;
byte servoTwoAngle = 0;

// Keep track of motor speeds
signed char motorOneDutyCycle = 0;
signed char motorTwoDutyCycle = 0;

// Accelerometer related values
Adafruit_BNO055 bno = Adafruit_BNO055(55);
adafruit_bno055_offsets_t calibrationData;
bool previousCalibration = false;

// Datatype for other peripheral sensors
typedef struct accelSonarLidarData {
  float distance_to_obstacle[2];
  double acceleration[3];
  unsigned short distance_to_target;
  unsigned long timestamp;
} sensor_data_t;

// Stores the latest data from sensors
sensor_data_t lastReading;

// unions for complex type to byte array conversions
union intToBytes {
  char buffer[2];
  uint16_t sensorData;
} converter;

union longToBytes {
  char buffer[4];
  unsigned long sensorData;
} long_converter;

union floatToBytes {
  char buffer[4];
  float sensorData;
} float_converter;

union doubleToBytes {
  char buffer[8];
  double sensorData;
} double_converter;

union allSensorData
{
  byte sensorData[sizeof(sensor_data_t)];
  sensor_data_t data;
} msg_to_send;


// Set up com ports
Uart masterSerialPort(&sercom0, 2, 3, SERCOM_RX_PAD_3, UART_TX_PAD_2);
TwoWire myWire(&sercom3, 0, 1);   // Create the new wire instance assigning it to pin 0 and 1

// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()    
{
  masterSerialPort.IrqHandler();
}

void setup() {
  // Init standard buses
  Serial.begin(115200);
  Wire.begin();
  while (!Serial);

  // Init second serial port
  pinPeripheral(2, PIO_SERCOM);   
  pinPeripheral(3, PIO_SERCOM);
  masterSerialPort.begin(9600);
  while (!masterSerialPort);

  // Initialize new I2C Bus
  myWire.begin(SLAVE_ADDR);       // join i2c bus with address #2
  myWire.onRequest(secondI2CBusRequestEvent);
  myWire.onReceive(secondI2CBusReceiveEvent);

  // Init peripheral sensors and actuators
  initServos();
  ScanI2CBus();
  setupTFMini();
  setupMKRMotorCarrier();
  setupSonicSensors();
  setupBNO055();
  
  // const byte interruptPin = 6;
  // const byte inPin = 7;
  // attachInterrupt(digitalPinToInterrupt(interruptPin), initiateSerialTransfer, CHANGE);
  // pinMode(inPin, INPUT);

  Serial.println(2*sizeof(float) + 3*sizeof(double) + sizeof(unsigned short) + sizeof(unsigned long));
  Serial.println(sizeof(sensor_data_t));
  Serial.println(sizeof(struct accelSonarLidarData));
}

void secondI2CBusRequestEvent()
{
  // We will convert float data into a string format by writing to floatToBytes union.
  // Then send the bytes of char array over to the master.
  byte currentAngle[2];
  signed char currentMotorDutyCycles[2];
  signed char motorSpeed = (signed char) last_data;

  switch (last_command) {

    // Lidar sensor only
    case 0x01:
      converter.sensorData = tfDist;
      myWire.write(converter.buffer, 2);
      break;

    // Move Servo one
    case 0x02:
      servoOneAngle = last_data;
      servo1.setAngle(servoOneAngle);

      currentAngle[0] = servoOneAngle;
      currentAngle[1] = servoTwoAngle;
      myWire.write(currentAngle, 2);
      break;

    // Move servo two
    case 0x03:
      servoTwoAngle = last_data;
      Serial.print("Servo two angle: ");
      Serial.println(servoTwoAngle);
      servo2.setAngle(servoTwoAngle);

      currentAngle[0] = servoOneAngle;
      currentAngle[1] = servoTwoAngle;
      myWire.write(currentAngle, 2);
      break;

    // return sonic obstacle detection only
    case 0x04:

      float_converter.sensorData = lastReading.distance_to_obstacle[0];
      myWire.write(float_converter.buffer, 4);

      float_converter.sensorData = lastReading.distance_to_obstacle[1];
      myWire.write(float_converter.buffer, 4);
      break;

    // Set the duty cycle on motor one, from -100% to 100%
    case 0x05:
      if (motorSpeed <= 100 && motorSpeed >= -100) {
        motorOneDutyCycle = motorSpeed;
        currentMotorDutyCycles[0] = motorOneDutyCycle;
        currentMotorDutyCycles[1] = motorTwoDutyCycle;
        M1.setDuty(last_data);
        myWire.write((char*)currentMotorDutyCycles, 2);
      } else {
        // Return -1 for error
        currentMotorDutyCycles[0] = 0xFF;
        currentMotorDutyCycles[1] = 0xFF;
        myWire.write((char*)currentMotorDutyCycles, 2);
      }
      break;
    // Set the duty cycle on motor two, from -100% to 100%
    case 0x06:
      if (motorSpeed <= 100 && motorSpeed >= -100) {
        motorTwoDutyCycle = last_data;
        currentMotorDutyCycles[0] = motorOneDutyCycle;
        currentMotorDutyCycles[1] = motorTwoDutyCycle;
        M2.setDuty(last_data);
        myWire.write((char*)currentMotorDutyCycles, 2);
      } else {
        // Return -1 for error
        currentMotorDutyCycles[0] = 0xFF;
        currentMotorDutyCycles[1] = 0xFF;
        myWire.write((char*)currentMotorDutyCycles, 2);
      }

      break;

    case 0x07:

      double_converter.sensorData = lastReading.acceleration[0];
      myWire.write(double_converter.buffer, 8);

      double_converter.sensorData = lastReading.acceleration[1];
      myWire.write(double_converter.buffer, 8);

      double_converter.sensorData = lastReading.acceleration[2];
      myWire.write(double_converter.buffer, 8);

      break;

  }
}


void secondI2CBusReceiveEvent(int numReceived) {

  int all_bytes;

  while ( 1 < myWire.available()) // loop through all but the last
  {

    byte extra_zero = myWire.read();
    byte command = myWire.read();
    byte data = myWire.read();
    all_bytes = ((command & 0xFF) << 8) | (data & 0xFF);

    last_command = command;
    last_data = data;
  }
}

// Attach the interrupt handler to the SERCOM
extern "C" {
  void SERCOM3_Handler(void);

  void SERCOM3_Handler(void) {
    myWire.onService();
  }
}

void loop() {
  // Stay in comms with MKR Motor Carrier
  controller.ping();

  // Process data from all sensors
  getAcceleration();
  getDistanceToTarget();
  detectObsticles();

  // Communicate with master over UART
  recvBytesWithStartEndMarkers();
}

void initServos() {
  servo1.setAngle(90);
  servo2.setAngle(0);
}

void printAccel() {
  sensors_event_t linAccel;
  bno.getEvent(&linAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);
  double x = linAccel.acceleration.x;
  double y = linAccel.acceleration.y;
  double z = linAccel.acceleration.z;

  Serial.print("Linear Acceleration X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);
  Serial.print("\n");
}

void getAcceleration() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  lastReading.acceleration[0] = euler.x();
  lastReading.acceleration[1] = euler.y();
  lastReading.acceleration[2] = euler.z();
}

void setCalibrationBNO055(adafruit_bno055_offsets_t data) {
  bno.setSensorOffsets(data);
  bno.getSensorOffsets(calibrationData);
}

void setupBNO055() {

  if (!bno.begin()) {
    Serial.print("BNO055 not detected!");
    while (1);
  }

  // External crystal for better accuracy.
  bno.setExtCrystalUse(true);

  if (previousCalibration) {
    bno.setSensorOffsets(calibrationData);
  } else {
    sensors_event_t event;
    bno.getEvent(&event);

    int tries = 0;
    while (!bno.isFullyCalibrated() && ++tries <= MAX_ATTEMPTS) {
      bno.getEvent(&event);
      Serial.println("Attempting to calibrate, please rotate sensor...");
      delay(100);
    }

    bno.getSensorOffsets(calibrationData);
  }
}

void setupSonicSensors() {
  dht = new DHT(DHTPIN, DHTTYPE);
  dht->begin();

  sonar1 = new Sonar(TRIGGER_PIN_1, ECHO_PIN_1, dht);
  sonar2 = new Sonar(TRIGGER_PIN_2, ECHO_PIN_2, dht);
}

void setupMKRMotorCarrier() {


  if (controller.begin())
  {
    Serial.print("MKR Motor Shield connected, firmware version ");
    Serial.println(controller.getFWVersion());
  }
  else
  {
    Serial.println("ERROR: Couldn't connect!");
  }

  // Reboot the motor controller; brings every value back to default
  Serial.println("reboot");
  controller.reboot();
  delay(500);

  //Take the battery status
  float batteryVoltage = (float)battery.getConverted();
  Serial.print("Battery voltage: ");
  Serial.print(batteryVoltage);
  Serial.print("V, Raw ");
  Serial.println(battery.getRaw());
}

void setupTFMini() {
  // Reassign pins 5 and 6 to SERCOM alt
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);


  Serial1.begin(115200);
  delay(20);
  tfmP.begin( &Serial1);


  // Send commands to device during setup.
  tfDist = 0;
  tfFlux = 0;
  tfTemp = 0;

  delay(500);

  Serial.println( "System reset: ");
  if ( tfmP.sendCommand( SYSTEM_RESET, 0))
  {
    Serial.println( "passed.");
  }
  else tfmP.printStatus(true);

  Serial.println( "Firmware version: ");
  if ( tfmP.sendCommand( OBTAIN_FIRMWARE_VERSION, 0))
  {
    Serial.print(tfmP.version[0]);
    Serial.print(tfmP.version[1]);
    Serial.println(tfmP.version[2]);

  }
  else tfmP.printStatus(true);

  // frameRate(20);

  Serial.println( "Data-Frame rate: ");
  if ( tfmP.sendCommand( SET_FRAME_RATE, FRAME_250))
  {
    Serial.println( FRAME_250);
  }
  else tfmP.printStatus(true);

}


/* TF mini PLUS functions */
void systemReset() {
  tfmP.sendCommand( SYSTEM_RESET, 0);
}

void frameRate( uint16_t rate) {
  tfmP.sendCommand( SET_FRAME_RATE, rate);
}

uint16_t getDistance() {
  return tfDist;
}

int getTempF() {
  uint16_t tfTempC = uint16_t(( tfTemp / 8) - 256);
  uint16_t tfTempF = uint16_t( tfTempC * 9.0 / 5.0) + 32.0;
  Serial.println( tfTempF);
  return tfTempF;
}

int getTempC() {
  uint16_t tfTempC = uint16_t(( tfTemp / 8) - 256);
  Serial.println( tfTempC);
  return tfTempC;
}

void getDistanceToTarget() {
  uint8_t tries = 0;
  while (!tfmP.getData( tfDist, tfFlux, tfTemp) && tries < MAX_ATTEMPTS) {
    tries++;
  }
}

/* Sonic Sensor methods */
float FrontObjectDetection() {
  return sonar1->detect();
}

float BackObjectDetection() {
  return sonar2->detect();
}

int detectObsticles() {
  lastReading.timestamp = millis();
  lastReading.distance_to_obstacle[0] = FrontObjectDetection();
  lastReading.distance_to_obstacle[1] = BackObjectDetection();
}


/* Serial communication functions */
void recvBytesWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  byte startMarker = 0x3C;
  byte endMarker = 0x3E;
  byte rb;


  while (masterSerialPort.available() > 0 && newData == false) {
    rb = masterSerialPort.read();
    Serial.println(rb);

    if (recvInProgress == true) {
      if (rb != endMarker) {
        receivedBytes[ndx] = rb;
        ndx++;
        if (ndx >= numBytes) {
          ndx = numBytes - 1;
        }
      } else {
        // receivedBytes[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        // numReceived = ndx;  // save the number for use when printing
        ndx = 0;
        newData = true;
      }
    } else if (rb == startMarker) {
      recvInProgress = true;
    }
  }


  parseMasterCommand();
}

void parseMasterCommand() {
  char command = receivedBytes[0];
  switch (command) {
    case 0x41:
      Serial.println("here we are in parse command!");
      initiateSerialTransfer();
      break;

    case 0x42:
      break;

    default :
      return;
  }

  receivedBytes[0] = 0x0;
  newData = false;
}

void initiateSerialTransfer() {
  masterSerialPort.flush();
  masterSerialPort.write(0x3C);

 
  for (int i = 0; i < 2; i++) {
    
    float_converter.sensorData = lastReading.distance_to_obstacle[i];
    for (int j = 0; j < sizeof(float); j++) {
      masterSerialPort.write(float_converter.buffer[j]);
    }
  }
  
  for (int i = 0; i < 3; i++) {
    
    double_converter.sensorData = lastReading.acceleration[i];
    for (int j = 0; j < sizeof(double); j++) {
      masterSerialPort.write(double_converter.buffer[j]);
    }
  }

  converter.sensorData = lastReading.distance_to_target;
  for (int i = 0; i < sizeof(unsigned short); i++) {
    masterSerialPort.write(converter.buffer[i]);
  }

  long_converter.sensorData = lastReading.timestamp;
  for (int i = 0; i < sizeof(unsigned long); i++) {
    masterSerialPort.write(converter.buffer[i]);
  }
 
  masterSerialPort.write(0x3E);
}


void ScanI2CBus() {
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
