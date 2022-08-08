#include <SerialMessage.h>

/*

  This example compiled and ran successfully using this hardware:

  SiK Telemetry Radio V3: https://shop.holybro.com/sik-telemetry-radio-v3_p1103.html
  raspberry-pi-pico: https://www.raspberrypi.com/products/raspberry-pi-pico/

  Simply connect the radio's TX and RX ports to Pico's RX and TX ports, respectively. 

*/

// Globals Here
SerialMessage* comm;
SerialMessage::MessageConfig messages[] = {                             
    { "V1",  SerialMessage::TYPE::CHAR, SerialMessage::DIR::INCOMING },
    { "V2",  SerialMessage::TYPE::INT, SerialMessage::DIR::INCOMING },
    { "V3", SerialMessage::TYPE::FLOAT, SerialMessage::DIR::INCOMING },
    { "V4", SerialMessage::TYPE::DOUBLE, SerialMessage::DIR::INCOMING }
};

void setup() {
  Serial.begin(9600);
  Serial1.begin(57600);
  while (!Serial1) {;}
  delay(DELAY);
  comm = new SerialMessage(
    &Serial1, 
    10000, // Timeout
    messages, 
    4, // Number messages
    true // Enable debug messages
  );

}

void loop() {

  if (comm->sync()) {
    SerialMessage::Message m1 = comm->get("V1");
    SerialMessage::Message m2 = comm->get("V2");
    SerialMessage::Message m3 = comm->get("V3");
    SerialMessage::Message m4 = comm->get("V4");
  } else {
    delay(200);
  }
}
