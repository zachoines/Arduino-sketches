/*

  Connect's bluetooth gamepad, and sends repective signals over RF to some other device (i.e RC car, robot, another arduino....)

  Requires WIFI Nina based boards:
  Arduino Nano RP2040 Connect
  Arduino Nano 33 IoT  

  Must first flash board with Bluepad32 firmware, as shown here: https://gitlab.com/ricardoquesada/bluepad32/-/blob/main/docs/plat_nina.md
  
*/
#include <Bluepad32.h>

// Data
#define MESSAGE_SIZE_BYTES 8
enum MESSAGE { LeftY, LeftX, RightY, RightX, SIZE };
union shortToBytes {
  uint8_t bytes[2];
  short value;
} converter;

struct Signal {
  uint8_t buffer[MESSAGE_SIZE_BYTES];
  short values[MESSAGE::SIZE];

  void toBytes() {
    for (int i = 0; i < MESSAGE::SIZE; i++) {
      converter.value = values[i];
      buffer[i * 2] = converter.bytes[0];
      buffer[(i * 2) + 1] = converter.bytes[1];
    }
  }

  void fromBytes() {
    for (int i = 0; i < MESSAGE::SIZE; i++) {
      converter.bytes[0] = buffer[i * 2];
      converter.bytes[1] = buffer[(i * 2) + 1];
      values[i] = converter.value;
    }
  }

  bool operator == (struct Signal& a)
  {
    for (int i = 0; i < MESSAGE::SIZE; i++) {
      if (abs(values[i] - a.values[i]) > 0) {
        return false;
      }
    }
    
    return true;
  }

  struct Signal& operator = (struct Signal& a)
  {
    for (int i = 0; i < MESSAGE::SIZE; i++) {
      this->values[i] = a.values[i];
    }
    toBytes();
    return *this;
  }
};

// Define globals
Signal newData;
Signal oldData;

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

  // BP32.pinMode(27, OUTPUT);
  // // BP32.digitalWrite(27, 0);

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

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

    if (myGamepad && myGamepad->isConnected()) {
    
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

  delay(150);
}