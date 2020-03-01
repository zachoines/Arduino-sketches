#include <Arduino.h>
#include <MKRMotorCarrier.h>

#include <TFMPlus.h>  // For TF mini Plus
#include <Wire.h>     // Arduino standard I2C/Two-Wire Library
#include <ArduinoBLE.h>

#include "wiring_private.h"

#define MAX_ATTEMPTS 5
#define SLAVE_ADDR 9


// Program variables
TFMPlus tfmP;         // Create a TFMini Plus object
char buf [100];
volatile byte pos;
volatile boolean process_it;
uint16_t tfDist;       // Distance
uint16_t tfFlux;       // Luminous flux or intensity of return signal
uint16_t tfTemp;       // Temp in °C
const byte numBytes = 2;
byte receivedBytes[numBytes];
byte numReceived = 0;
boolean newData = false;
const byte interruptPin = 2;

// Incoming I2C bits from receive interrupt
uint8_t last_command = 0;
uint8_t last_data = 0;

// Keep track of current actuator values
int servoOneAngle = 0;
int servoTwoAngle = 0;

int motorOneDutyCycle = 0;
int motorTwoDutyCycle = 0;

union intToBytes {

  char buffer[2];
  uint16_t sensorData;

} converter;

// Set up com ports
Uart masterSerialPort(&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
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
  masterSerialPort.begin(9600);

  // Initialize new I2C Bus
  myWire.begin(SLAVE_ADDR);       // join i2c bus with address #2
  pinPeripheral(0, PIO_SERCOM);   //Assign SDA function to pin 0
  pinPeripheral(1, PIO_SERCOM);   //Assign SCL function to pin 1
  myWire.onRequest(secondI2CBusRequestEvent);
  myWire.onReceive(secondI2CBusReceiveEvent);


  // Init peripheral sensors and actuators
  initServos();
  ScanI2CBus();
  setupTFMini();
  setupMKRMotorCarrier();


  attachInterrupt(digitalPinToInterrupt(interruptPin), initiateTransfer, FALLING);
}

void secondI2CBusRequestEvent()
{
  // We will convert float data into a string format by writing to floatToBytes union.
  // Then send the bytes of char array over to the master.
  byte currentAngle[2];

  switch (last_command) {
    case 0x01:
      converter.sensorData = tfDist;
      Serial.print("Here we are in request!: ");
      Serial.println(tfDist);
      Serial.print("Here is the data: ");
      Serial.println(converter.sensorData);
      myWire.write(converter.buffer, 2);
      break;

    case 0x02:
      servoOneAngle = last_data;
      Serial.print("Servo one angle: ");
      Serial.println(servoOneAngle);
      servo1.setAngle(servoOneAngle);

      currentAngle[0] = servoOneAngle;
      currentAngle[1] = servoTwoAngle;
      myWire.write(currentAngle, 2);
      break;

    case 0x03:
      servoTwoAngle = last_data;
      Serial.print("Servo two angle: ");
      Serial.println(servoTwoAngle);
      servo2.setAngle(servoTwoAngle);

      currentAngle[0] = servoOneAngle;
      currentAngle[1] = servoTwoAngle;
      myWire.write(currentAngle, 2);
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
    all_bytes = ((command & 0xFF) << 8) |  (data & 0xFF);

    last_command = command;
    last_data = data;
  }

  Serial.print("Num bytes sent: ");
  Serial.println(numReceived);

  Serial.print("All data received: ");
  Serial.println(all_bytes, HEX);

  Serial.print("last_command: ");
  Serial.println(last_command);

  Serial.print("last_data: ");
  Serial.println(last_data);
}

// Attach the interrupt handler to the SERCOM
extern "C" {
  void SERCOM3_Handler(void);

  void SERCOM3_Handler(void) {
    myWire.onService();
  }
}

void loop() {
  while (masterSerialPort.available() > 0) {
    Serial.println(masterSerialPort.read());
  }

  getTFMINIPlusData();
  // recvBytesWithStartEndMarkers();

  // Stay in comms with MKR Motor Carrier
  controller.ping();
}

void initServos() {
  servo1.setAngle(90);
  servo2.setAngle(0);
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
  tfDist = 0;            // Clear device data variables.
  tfFlux = 0;
  tfTemp = 0;

  delay(500);            // And wait for half a second.

  Serial.println( "System reset: ");
  if ( tfmP.sendCommand( SYSTEM_RESET, 0))
  {
    Serial.println( "passed.");
  }
  else tfmP.printStatus(true);

  Serial.println( "Firmware version: ");
  if ( tfmP.sendCommand( OBTAIN_FIRMWARE_VERSION, 0))
  {
    Serial.print(tfmP.version[0]); // print three single numbers
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

void getTFMINIPlusData() {
  uint8_t tries = 0;
  while (!tfmP.getData( tfDist, tfFlux, tfTemp) && tries < MAX_ATTEMPTS) {
    tries++;
  }
}


/* Serial communication functions */
void recvBytesWithStartEndMarkers() {

  static boolean recvInProgress = false;
  static byte ndx = 0;
  byte startMarker = 0x3C;
  byte endMarker = 0x3E;
  byte rb;

  if (newData == true) {

    parseMasterCommand();
  }

  while (masterSerialPort.available() > 0 && newData == false) {
    rb = masterSerialPort.read();


    if (recvInProgress == true) {
      if (rb != endMarker) {
        receivedBytes[ndx] = rb;
        ndx++;
        if (ndx >= numBytes) {
          ndx = numBytes - 1;
        }
      }
      else {
        // receivedBytes[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        // numReceived = ndx;  // save the number for use when printing
        ndx = 0;
        newData = true;
      }
    }

    else if (rb == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseMasterCommand() {

  char command = receivedBytes[0];
  switch (command) {
    case 'A' :
      Serial.println("We are A");
      initiateTransfer();
      break;
    case 'B' :
      Serial.println("We are B");
      break;

    default :
      return;
  }


  newData = false;

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

void initiateTransfer() {
  converter.sensorData = getDistance();
  //  char message[4];
  //  message[0] = 0x3C;
  //  message[3] = 0x3E;

  masterSerialPort.write(0x3C);

  for (int i = 0; i < 2; i++) {
    masterSerialPort.write(converter.buffer[i]);
  }

  masterSerialPort.write(0x3E);
}
