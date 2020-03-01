

#include <Arduino.h>
#include <TFMPlus.h>  // For TF mini Plus
#include <Wire.h>     // Arduino standard I2C/Two-Wire Library
// #include <ArduinoBLE.h>

// #include "wiring_private.h"

#define MAX_ATTEMPTS 5


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

union intToBytes {

  char buffer[2];
  uint16_t sensorData;

} converter;


UART masterSerialPort (digitalPinToPinName(4), digitalPinToPinName(3), NC, NC);  // create a hardware serial port named mySerial with RX: pin 13 and TX: pin 8
// Uart TFMiniPlusSerial (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);

// Attach the interrupt handler to the SERCOM
//void SERCOM1_Handler()
//{
//    TFMiniPlusSerial.IrqHandler();
//}


void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  while (!Serial);
  
  // ScanI2CBus(); 
  setupTFMini();
  // setupMKRMotorCarrier();
  

  attachInterrupt(digitalPinToInterrupt(interruptPin), initiateTransfer, FALLING);
}

void loop() {
  getTFMINIPlusData();
  // recvBytesWithStartEndMarkers();
}

void setupTFMini() {
  // Reassign pins 5 and 6 to SERCOM alt
//  pinPeripheral(13, PIO_SERCOM);
//  pinPeripheral(8, PIO_SERCOM);

  delay(100); 
  // Start my new hardware serial
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

  frameRate(20);
  
//  Serial.println( "Data-Frame rate: ");
//  if ( tfmP.sendCommand( SET_FRAME_RATE, FRAME_250))
//  {
//    Serial.println( FRAME_250);
//  }
//  else tfmP.printStatus(true);

}


// TF mini PLUS functions
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

  Serial.println(tfDist);
}

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
