#include <SerialMessage.h>
#include "RoboClaw.h"

UART Serial2(8, 9, 0, 0);

// Other program constants
#define address 0x80
#define DELAY 1000

// Shared program variables
float thresholdPressure = 100.0; 
float maxPressure = 150.0;
RoboClaw roboclaw(&Serial2,10000);

// Program state variables
bool isCompressorActivated = false;
bool isSolenoidActivated = true;

// Physical Button Definitions
int SOLENOID_VALVE_BUTTON = 16;
int COMPRESSOR_BUTTON = 17;

// Physical Pin Definitions
int SOLENOID_VALVE_PIN = 0; // Not hooked up yet
int COMPRESSOR_PIN = A2; // Not hooked up yet
int PRESSURE_TRANSDUCER_PIN = A1;

// Function hoisting here
float map(float x, float in_min, float in_max, float out_min, float out_max);
float getCurrentPressure();
void buildNominalPressure(float threshold, int delay);
void toggleCompressorState();
void toggleSolenoidState();
bool getCompressorState();
bool getSolenoidState();

// Globals Here
SerialMessage* comm;
SerialMessage::MessageConfig messages[] = {                             
    { "LeftY",  SerialMessage::TYPE::CHAR, SerialMessage::DIR::INCOMING },
    { "LeftX",  SerialMessage::TYPE::INT, SerialMessage::DIR::INCOMING },
    { "RightY", SerialMessage::TYPE::FLOAT, SerialMessage::DIR::INCOMING },
    { "RightX", SerialMessage::TYPE::DOUBLE, SerialMessage::DIR::INCOMING }
};


void setup() {
  // put your setup code here, to run once:
  roboclaw.begin(38400);
  Serial.begin(9600);
  Serial1.begin(57600);
  while (!Serial1) {;}
  delay(DELAY);
  comm = new SerialMessage(
    &Serial1,
    10000,
    messages,
    4
  );
  Serial.println("Starting...");
  toggleCompressorState();
  toggleSolenoidState();
}


void loop() {
  if (comm->sync()) {
    SerialMessage::Message m1 = comm->get("LeftY");
    SerialMessage::Message m2 = comm->get("LeftX");
    SerialMessage::Message m3 = comm->get("RightY");
    SerialMessage::Message m4 = comm->get("RightX");
  } else {
    delay(200);
  }

  




  // if (Serial1.available() >= MESSAGE_SIZE_BYTES) {
  //   int numRead = Serial1.readBytes(newData.buffer, MESSAGE_SIZE_BYTES);
  //   newData.fromBytes();
  //   if (numRead != MESSAGE_SIZE_BYTES) {
  //     Serial.println("ERROR: Issue reading commands...");
  //   }
    
  //   char buffer[120];
  //   snprintf(buffer, sizeof(buffer) - 1,
  //             "New Readings: axis L: %i, %i, axis R: %i, %i",
  //             newData.values[MESSAGE::LeftY],
  //             newData.values[MESSAGE::LeftX],
  //             newData.values[MESSAGE::RightY],
  //             newData.values[MESSAGE::RightX]
  //   );
  //   Serial.println(buffer);
  //   oldData = newData;
  // }
  
  /*
  // Condition to build up pressure until threshold
  if (getCompressorState() && getCurrentPressure() >= thresholdPressure) {
    // Serial.print("Current pressure is: ");
    // Serial.println(getCurrentPressure(), 4);
    toggleCompressorState();

    // Open solenoid
    if (!getSolenoidState()) {
      toggleSolenoidState();
    }
  } 
  
  // Condition to stop building pressure
  if (getCompressorState() && getCurrentPressure() >= maxPressure) {
    // Serial.print("Current pressure is: ");
    // Serial.println(getCurrentPressure(), 4);
    toggleCompressorState();
  }
  */
}


float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float getCurrentPressure() {
  float conversion_factor = 5.0 / 1024;
  uint16_t result = analogRead(PRESSURE_TRANSDUCER_PIN);
  float voltage = (result - 1) * conversion_factor;
  // Serial.println(voltage, 4);
  // Serial.print("Analogue voltage is: ");
  return map(voltage, 0.455, 4.5, 0.0, 150.0); // Output specs of pressure-transducer. TODO:: Export as globals
}


void toggleCompressorState() {
  isCompressorActivated = !isCompressorActivated;
  if (isCompressorActivated) {
    roboclaw.ForwardM1(address, 127);
    delay(DELAY);
    Serial.println("Compressor running...");
  } else {
    roboclaw.ForwardM1(address, 0);
    delay(DELAY);
    Serial.println("Compressor stopped...");
  }
}


void toggleSolenoidState() {
  isSolenoidActivated = !isSolenoidActivated;
  if (isSolenoidActivated) {
    roboclaw.ForwardM2(address, 127);
    delay(DELAY);
    Serial.println("Solenoid on...");
  } else {
    roboclaw.ForwardM2(address, 0);
    delay(DELAY);
    Serial.println("Solenoid off...");
  } 
}


bool getCompressorState() {
  return isCompressorActivated;
}


bool getSolenoidState() {
  return isCompressorActivated;
}