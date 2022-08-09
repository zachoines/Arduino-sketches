#include <SerialMessage.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"


// Constants
#define ADDRESS 0x80
#define DELAY 500
#define THRESH_PRESSURE  100.0 
#define MAX_PRESSURE 150.0
#define NUM_MESSAGES 6
#define TIMEOUT 10000
#define STALE_THRESH 200
#define DEBUG true
#define TRANSDUCER_MIN_VOLTAGE 0.455
#define TRANSDUCER_MAX_VOLTAGE 4.50


// Globals Here
enum STATE { /* Compressor ON/OFF; Solenoid OPEN/CLOSED */
  OFF,
  ON,
  OPEN,
  CLOSED
};

enum DEVICE {
  COMPRESSOR,
  SOLENOID,
  num_devices
};

enum PINS {
  PRESSURE_TRANSDUCER = A1,
  SERIAL2_RX = A2, 
  SERIAL2_TX = A3
};

enum MESSAGE_DEFS {
  PAN_Y,
  PAN_X,
  TILT_Y,
  TILT_X,
  TOGGLE_SOLENOID,
  TOGGLE_COMPRESSOR
};


// Function hoisting here
float map(float x, float in_min, float in_max, float out_min, float out_max);
bool is_stale(SerialMessage::Message message, unsigned long currrent_time);
bool set_state(DEVICE d, STATE s);
float get_current_pressure();


// Program objects
STATE DEVICE_STATES[DEVICE::num_devices];
SerialMessage* comm;
SerialMessage::Message messages[NUM_MESSAGES];
SerialMessage::MessageConfig message_configs[] = {                             
    { "LeftY",  SerialMessage::TYPE::SHORT, SerialMessage::DIR::INCOMING },
    { "LeftX",  SerialMessage::TYPE::SHORT, SerialMessage::DIR::INCOMING },
    { "RightY", SerialMessage::TYPE::SHORT, SerialMessage::DIR::INCOMING },
    { "RightX", SerialMessage::TYPE::SHORT, SerialMessage::DIR::INCOMING },
    { "R2", SerialMessage::TYPE::BOOL, SerialMessage::DIR::INCOMING },
    { "L2", SerialMessage::TYPE::BOOL, SerialMessage::DIR::INCOMING }
};

SoftwareSerial Serial2(PINS::SERIAL2_RX, PINS::SERIAL2_TX);	
RoboClaw roboclaw(&Serial2, TIMEOUT);
// UART Serial2(8, 9, 0, 0); // For use on RP2040 chips

// Program state variables
bool isCompressorActivated = false;
bool isSolenoidActivated = true;
unsigned long currrent_time;


void setup() {
  // put your setup code here, to run once:
  currrent_time = micros();
  roboclaw.begin(38400);
  Serial.begin(9600);
  Serial1.begin(57600);
  while (!Serial1) {;}

  comm = new SerialMessage(
    &Serial1,
    TIMEOUT,
    message_configs,
    NUM_MESSAGES,
    DEBUG
  );
  
  if (DEBUG)
    Serial.println("Starting...");
}


void loop() {

  currrent_time = micros();
  if (comm->sync()) {
    messages[MESSAGE_DEFS::PAN_Y] = comm->get("LeftY");
    messages[MESSAGE_DEFS::PAN_X] = comm->get("LeftX");
    messages[MESSAGE_DEFS::TILT_Y] = comm->get("RightY");
    messages[MESSAGE_DEFS::TILT_X] = comm->get("RightX");
    messages[MESSAGE_DEFS::TOGGLE_SOLENOID] = comm->get("R2");
    messages[MESSAGE_DEFS::TOGGLE_COMPRESSOR] = comm->get("L2");
  } else {
    delay(200);
  }
  
  float current_pressure = get_current_pressure();
  if (DEBUG) {
    Serial.print("NOTE: Current pressure is: ");
    Serial.println(current_pressure, 4);
  }
    

  // Fire the air cannon 
  if (!is_stale(messages[MESSAGE_DEFS::TOGGLE_SOLENOID], currrent_time) && *(bool*) messages[MESSAGE_DEFS::TOGGLE_SOLENOID].value && current_pressure > THRESH_PRESSURE) 
  {
    STATE previous_compressor_state = get_state(DEVICE::COMPRESSOR); 
    set_state(DEVICE::COMPRESSOR, STATE::OFF); // AVOID too much current draw, so turn off
    set_state(DEVICE::SOLENOID, STATE::OPEN);
    delay(DELAY);
    set_state(DEVICE::SOLENOID, STATE::CLOSED);
    set_state(DEVICE::COMPRESSOR, previous_compressor_state);
  }

  // Turn on the compresssor if toggled
  current_pressure = get_current_pressure();
  if (
    !is_stale(messages[MESSAGE_DEFS::TOGGLE_COMPRESSOR], currrent_time) && 
    *(bool*) messages[MESSAGE_DEFS::TOGGLE_COMPRESSOR].value &&
    get_state(DEVICE::COMPRESSOR) == STATE::OFF &&
    current_pressure < MAX_PRESSURE)
  {
    set_state(DEVICE::COMPRESSOR, STATE::ON);
  }

  
  // Turn off compressor if over MAX
  if (
    get_state(DEVICE::COMPRESSOR) == STATE::ON &&
    current_pressure >= MAX_PRESSURE
  ) 
  {
    set_state(DEVICE::COMPRESSOR, STATE::OFF);
  }
}


float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float get_current_pressure() {
  float conversion_factor = 5.0 / 1024;
  uint16_t result = analogRead(PINS::PRESSURE_TRANSDUCER);
  float voltage = (result - 1) * conversion_factor;
  // Serial.println(voltage, 4);
  // Serial.print("Analogue voltage is: ");
  return map(voltage, TRANSDUCER_MIN_VOLTAGE, TRANSDUCER_MAX_VOLTAGE, 0.0, MAX_PRESSURE); // Output specs of pressure-transducer. TODO:: Export as globals
}


bool is_stale(SerialMessage::Message message, unsigned long currrent_time) {
  return (message.timestamp - currrent_time > STALE_THRESH);
}


STATE get_state(DEVICE d) {
  return DEVICE_STATES[d];
}


bool set_state(DEVICE d, STATE s) {
  switch (d)
  {
  case DEVICE::COMPRESSOR:
    switch (s)
    {
    case STATE::OFF:
      roboclaw.ForwardM1(ADDRESS, 127);
      if (DEBUG)
        Serial.println("NOTE: Compressor off.");
      
      break;
    
    case STATE::ON:
      roboclaw.ForwardM1(ADDRESS, 127);
      if (DEBUG)
        Serial.println("NOTE: Compressor on.");
      
      break;

    default:
      if (DEBUG)
        Serial.print("WARNING: Not valid state for compressor.");
      return false;
    }

    DEVICE_STATES[d] = s;    
    return true;
  
  case DEVICE::SOLENOID:
    switch (s)
      {
      case STATE::CLOSED:
        roboclaw.ForwardM2(ADDRESS, 0);
        if (DEBUG)
          Serial.println("NOTE: Solenoid off.");

        break;
      
      case STATE::OPEN:
        roboclaw.ForwardM2(ADDRESS, 127);
        if (DEBUG)
          Serial.println("NOTE: Solenoid on.");
        
        break;

      default:
        if (DEBUG)
          Serial.print("WARNING: Not valid state for solenoid.");
        return false;
      }
      
      DEVICE_STATES[d] = s;
      return true;

  default:
    return false;
  }
}