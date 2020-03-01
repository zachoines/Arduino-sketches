#include <MKRMotorCarrier.h>


volatile unsigned int SERVO_2_ANGLE = 0;
volatile unsigned int SERVO_4_ANGLE = 0;

union floatToBytes {

  char buffer[4];
  float sensorFloatData;

} converter;

void setup() {
  Serial.begin(9600); // begin transmission
  while (!Serial);


  //Establishing the communication with the motor shield
  if (controller.begin()) {
    Serial.print("Motors, servos, and sensors Connected. Firmware version ");
    Serial.println(controller.getFWVersion());
    
    controller.reboot();
    delay(500);
  
    //Take the battery status
    float batteryVoltage = (float)battery.getConverted();
    Serial.print("Battery voltage: ");
    Serial.print(batteryVoltage);
    Serial.print("V, Raw ");
    Serial.println(battery.getRaw());
  } else {
    Serial.println("Couldn't connect motors, servos, and sensors. Check wires and start again");
    while (1);
  }


  // Setup up interupts
  attachInterrupt(digitalPinToInterrupt(7), motorRun, RISING);
  attachInterrupt(digitalPinToInterrupt(8), servoRun, RISING);
  attachInterrupt(digitalPinToInterrupt(9), readSensors, RISING);
}


void motorRun() {
  String val;
  while (Serial.available() > 0) {
    val = val + (char)Serial.read(); // read data byte by byte and store it
  }
  Serial.print("Hi from motor \n"); // send the received data back to raspberry pi
}

float checkBatteryVoltages() {

  return (float)battery.getConverted();

}

void servoRun() {
  String val;
  int angle;
  int servo;
  
//  while (Serial.available() > 0) {
//    val = val + (char)Serial.read(); // read data byte by byte and store it
//  }

  servo = Serial.parseInt();
  angle = Serial.parseInt();

  switch (servo) {
      case 1: 
          servo1.setAngle( angle );
      case 2: 
          servo2.setAngle( angle );
      case 3: 
          servo3.setAngle( angle );
      case 4: 
          servo4.setAngle( angle );
  }
}

void readSensors() {
  String val;
  while (Serial.available() > 0) {
    val = val + (char)Serial.read(); // read data byte by byte and store it
  }
  Serial.print("Hi from sensors \n"); // send the received data back to raspberry pi
}

void loop() {

}
