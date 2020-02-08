

// #include "Time.h"
#include "Sonar.h"
#include "Wire.h"
#include "ADXL345.h"
#include "L293D_MOTOR.h"



#define INTERRUPT_PIN 6

//Variable to store the battery voltage
int batteryVoltage;


// Define Slave I2C Address
#define SLAVE_ADDR 9

#define DHTPIN 1
#define DHTTYPE DHT22
#define TRIGGER_PIN_1  3
#define ECHO_PIN_1     0
#define TRIGGER_PIN_2  4
#define ECHO_PIN_2     2
#define MAX_DISTANCE 400
#define MIN_DISTANCE 3


#define motor_one_enable 5
#define motor_one_input1 14
#define motor_one_input2 15

#define motor_two_enable 6
#define motor_two_input1 17
#define motor_two_input2 16




#define SOUND_PIN 6
#define LED_PIN 9

DHT* dht;
Sonar* sonar1;
Sonar* sonar2;
ADXL* adxl;
L293D_MOTOR* motor1 = nullptr;
L293D_MOTOR* motor2 = nullptr;

// Union Memory trick.
/* Write the float to floatToBytes.sensorFloatData. Its bits will automatically be
   formatted in memory and readable from floatToBytes.buffer. Send buffer over wire.write() and then
   read into a similarly declared union's floatToBytes.buffer member on the master. Then, reading the floatToBytes.sensorFloatData
   on the master will give the correct float value 
   
*/
union floatToBytes {

  char buffer[4];
  float sensorFloatData;

} converter;

typedef struct accelAndSonarData {
  float distance[2];
  float acceleration[3];
  unsigned long timestamp;
} sensor_data_t;

// Stores the latest data from sensors
sensor_data_t lastReading;

// Data buffers holding the last 10 values
sensor_data_t dataBuffer[10];
int BUFF_COUNT = -1;
int BUFF_ENTRIES = 10;


// other global variables
int last_command = -1;

void setup() {
  // Setup of pins
  pinMode(SOUND_PIN, OUTPUT);

  adxl = new ADXL();

  Serial.begin(9600);

  //Serial port initialization
  while (!Serial);

  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);

  // Function to run when data requested from master
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // Init global variables
  dht = new DHT(DHTPIN, DHTTYPE);
  dht->begin();


  sonar1 = new Sonar(TRIGGER_PIN_1, ECHO_PIN_1, dht);
  sonar2 = new Sonar(TRIGGER_PIN_2, ECHO_PIN_2, dht);

  motor1 = new L293D_MOTOR(motor_one_enable, motor_one_input1, motor_one_input2);
  motor2 = new L293D_MOTOR(motor_two_enable, motor_two_input1, motor_two_input2);

}

void loop() {

  // Loop every
  detect();

  if (motor1 != nullptr && motor2 != nullptr) {
      motor1->reverse(255);
      motor2->reverse(255); 
  }
  

  delay(100);

}

void receiveEvent(int request) {
  last_command = request;
}

void requestEvent() {

  // We will convert float data into a string format by writing to floatToBytes union.
  // Then send the bytes of char array over to the master.
  switch (last_command) {

    case -1:
      converter.sensorFloatData = 0xFFFF;
      Wire.write(converter.buffer, 4);

      break;

    // Send data from sonar sensors
    case 1:

      converter.sensorFloatData = lastReading.distance[0];
      Wire.write(converter.buffer, 4);

      converter.sensorFloatData = lastReading.distance[1];
      Wire.write(converter.buffer, 4);

      break;

    // Send data from acceleration
    case 2:

      converter.sensorFloatData = lastReading.acceleration[0];
      Wire.write(converter.buffer, 4);

      converter.sensorFloatData = lastReading.acceleration[1];
      Wire.write(converter.buffer, 4);

      converter.sensorFloatData = lastReading.acceleration[2];
      Wire.write(converter.buffer, 4);

      converter.sensorFloatData = 0xFFFF;
      Wire.write(converter.buffer, 4);

      break;

    // reading data from both
    case 3:

      converter.sensorFloatData = lastReading.distance[0];
      Wire.write(converter.buffer, 4);

      converter.sensorFloatData = lastReading.distance[1];
      Wire.write(converter.buffer, 4);

      converter.sensorFloatData = lastReading.acceleration[0];
      Wire.write(converter.buffer, 4);

      converter.sensorFloatData = lastReading.acceleration[1];
      Wire.write(converter.buffer, 4);

      converter.sensorFloatData = lastReading.acceleration[2];
      Wire.write(converter.buffer, 4);

      //      converter.sensorFloatData = 0xFFFF;
      //      Wire.write(converter.buffer, 4);
  }
}


void playSound(float freq) {

  tone(SOUND_PIN, freq);
  delay(500);
  noTone(SOUND_PIN);
  delay(100);
}

void blinkLED(int pin) {

}

float FrontObjectDetection() {
  return sonar1->detect();
}

float BackObjectDetection() {
  return sonar2->detect();
}

int updateBuffer() {
  sensor_data_t data;
  data.distance[0] = lastReading.distance[0];
  data.distance[1] = lastReading.distance[1];

  data.acceleration[0] = lastReading.acceleration[0];
  data.acceleration[1] = lastReading.acceleration[1];
  data.acceleration[2] = lastReading.acceleration[2];

  data.timestamp = lastReading.timestamp;

  dataBuffer[(BUFF_COUNT++) % BUFF_ENTRIES] = data;
}

struct accel_data readAccelData() {

//  if (adxl->calibrateRequest()) {
//    struct calibration_data c;
//    playSound(1000);
//    Serial.print("Calibration request initiated. \n");
//    c = adxl->performCalibration();
//    Serial.print("Performed calibration. \n");
//  }

  //These variables will be used to hold the x,y and z axis accelerometer values.
  return adxl->readAccelAdjusted();


}

int detect() {
  struct accel_data a = readAccelData();

  // lastReading.timestamp = millis();

  lastReading.acceleration[0] = a.x;
  lastReading.acceleration[1] = a.y;
  lastReading.acceleration[2] = a.z;

  lastReading.distance[0] = FrontObjectDetection();
  lastReading.distance[1] = BackObjectDetection();
  //
  //  updateBuffer();

//  Serial.print(a.x);
//  Serial.print(", next ");
//  Serial.print(a.y);
//  Serial.print(", next ");
//  Serial.println(a.z);
//  Serial.print("\n");
//
//
//  Serial.print("Detected object in front at ");
//  Serial.print(lastReading.distance[0]);
//  Serial.print("\n");
//
//  Serial.print("Detected object in back at ");
//  Serial.print(lastReading.distance[1]);
//  Serial.print("\n");
}
