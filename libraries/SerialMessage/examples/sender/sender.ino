#include <SerialMessage.h>

/*

  This example compiled and ran successfully using this hardware:

  SiK Telemetry Radio V3: https://shop.holybro.com/sik-telemetry-radio-v3_p1103.html
  raspberry-pi-pico: https://www.raspberrypi.com/products/raspberry-pi-pico/

  Simply connect the radio's TX and RX ports to Pico's RX and TX ports, respectively. 

*/

// Define globals
SerialMessage* comm;
SerialMessage::MessageConfig messages[] = {                             
    { "V1",  SerialMessage::TYPE::CHAR, SerialMessage::DIR::OUTGOING },
    { "V2",  SerialMessage::TYPE::INT, SerialMessage::DIR::OUTGOING },
    { "V3", SerialMessage::TYPE::FLOAT, SerialMessage::DIR::OUTGOING },
    { "V4", SerialMessage::TYPE::DOUBLE, SerialMessage::DIR::OUTGOING }
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(57600);
  while (!Serial1) {;}
  while (!Serial) {;}

  comm = new SerialMessage(
    &Serial1, 
    10000, // Timeout
    messages, 
    4, // Number messages
    true // Enable debug messages
  );

}

void loop() {
  char v1 = 'A';
  int v2 = 1234;
  float v3 = 30.234f;
  double v4 = 112.12d;
  comm->set("V1", &v1);
  comm->set("V2", &v2);
  comm->set("V3", &v3);
  comm->set("V4", &v4);
  delay(200);
}
