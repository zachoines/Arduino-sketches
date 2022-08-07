/*

  Connect's bluetooth gamepad, and sends repective signals over RF to some other device (i.e RC car, robot, another arduino....)

  Requires WIFI Nina based boards:
  Arduino Nano RP2040 Connect
  Arduino Nano 33 IoT  

  Must first flash board with Bluepad32 firmware, as shown here: https://gitlab.com/ricardoquesada/bluepad32/-/blob/main/docs/plat_nina.md
  
*/
#include <Bluepad32.h>
#include <SerialMessage.h>

// Define globals
SerialMessage* comm;
SerialMessage::MessageConfig messages[] = {                             
    { "LeftY",  SerialMessage::TYPE::CHAR, SerialMessage::DIR::OUTGOING },
    { "LeftX",  SerialMessage::TYPE::INT, SerialMessage::DIR::OUTGOING },
    { "RightY", SerialMessage::TYPE::FLOAT, SerialMessage::DIR::OUTGOING },
    { "RightX", SerialMessage::TYPE::DOUBLE, SerialMessage::DIR::OUTGOING }
};

GamepadPtr myGamepads[BP32_MAX_GAMEPADS] = {};

void setup() {
  // Initialize serial
  Serial.begin(9600);
  Serial1.begin(57600);
  while (!Serial1) {;}
  while (!Serial) {;}

  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  comm = new SerialMessage(
    &Serial1,
    10000,
    messages,
    4
  );
  

  // This call is mandatory. It setups Bluepad32 and creates the callbacks.
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  // BP32.forgetBluetoothKeys();
}

void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.print("CALLBACK: Gamepad is connected, index=");
      Serial.println(i);
      myGamepads[i] = gp;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      GamepadProperties properties = gp->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.print("CALLBACK: Gamepad is disconnected from index=");
      Serial.println(i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

void loop() {

  BP32.update();

  char v1 = 'A';
  int v2 = 1234;
  float v3 = 30.234f;
  double v4 = 112.12d;
  comm->set("LeftY", &v1);
  delay(100);
  comm->set("LeftX", &v2);
  delay(100);
  comm->set("RightY", &v3);
  delay(100);
  comm->set("RightX", &v4);
  delay(2000);

  /*
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

    if (myGamepad && myGamepad->isConnected()) {


      // comm->set("LeftY", (void*)myGamepad->axisY());
      // comm->set("LeftX", (void*)myGamepad->axisX());
      // comm->set("RightY", (void*)myGamepad->axisRY());
      // comm->set("RightX", (void*)myGamepad->axisRX());
      comm->sync();
    
      newData.values[MESSAGE::LeftY] = myGamepad->axisY();
      newData.values[MESSAGE::LeftX] = myGamepad->axisX();
      newData.values[MESSAGE::RightY] = myGamepad->axisRY();
      newData.values[MESSAGE::RightX] = myGamepad->axisRX();
      newData.toBytes();

      if (!(newData == oldData)) {
        if (Serial1.write(newData.buffer, MESSAGE_SIZE_BYTES) == MESSAGE_SIZE_BYTES) {
          unsigned long clocktime = millis();
          Serial.print(clocktime, 1);  
          Serial.println(": message sent");
          char buffer[120];
          snprintf(buffer, sizeof(buffer) - 1,
                  "New Readings: axis L: %4li, %4li, axis R: %4li, %4li",
                  newData.values[MESSAGE::LeftY],
                  newData.values[MESSAGE::LeftX],
                  newData.values[MESSAGE::RightY],
                  newData.values[MESSAGE::RightX]);
          Serial.println(buffer);
        };
      }

      oldData = newData;
    }
  }
  */
}