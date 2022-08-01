#include <Wire.h>
#include <ArduinoBLE.h>
#include "DRV8833_MOTOR.h"
#include "SLF06_FLOW_SENSOR.h"
#include "RGB_LED.h"

DRV8833_MOTOR* motor1 = nullptr;
DRV8833_MOTOR* motor2 = nullptr;
SLF06_FLOW_SENSOR* flow_meter = nullptr;
RGB_LED* led = nullptr;

// Current program values.
char last_command = 's';
long last_time_frame = 0;
float last_milliliters = 0;
int last_motor_speed = 0;

// BLE SPIDER Service
BLEService SPIDERService("c03b622f-332e-4058-8fa8-f3d3eceafd8b");

// BLE SPIDER Characteristics
BLEFloatCharacteristic currentFlowRate("da75b171-a289-4684-9413-3767fe886163",  // standard 16-bit characteristic UUID
                                       BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEFloatCharacteristic currentTemp("644b9364-7b9f-4a5b-94e7-6ec1a5b729c9",  // standard 16-bit characteristic UUID
                                   BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes


BLEIntCharacteristic currentMotorRate("86fe771d-a1ab-4455-87ec-a9a994787808",  // standard 16-bit characteristic UUID
                                      BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes


BLELongCharacteristic timeFrame("2a4effe1-0a8c-49f1-98d2-c45658cd93d6", BLEWriteWithoutResponse);

BLEFloatCharacteristic milliliters("d9d993b6-cf4a-42e1-b184-57de95962bec", BLEWriteWithoutResponse);

BLEIntCharacteristic motorSpeed("9b1d7d08-d675-437c-93b4-bc94be9bf51d", BLEWriteWithoutResponse);

BLECharCharacteristic command("1e6c421f-4a2f-42fe-8593-a6e285cf6978", BLEWriteWithoutResponse);

BLEIntCharacteristic ledBrightness("505db17a-83f0-4a06-b62f-6e38ef4cadaf", BLEWriteWithoutResponse);


/* Callbacks for writable characteristics */
void timeFrameCallback(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Here we are in the set time frame callback. Here is the read value: ");
  updateTimeFrame(timeFrame.value());
  Serial.println(timeFrame.value(), DEC);
}

void millilitersCallback(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Here we are in the set mililiters callback. Here is the read value: ");
  updateMilliliters(milliliters.value());
  Serial.println(milliliters.value(), DEC);
}

void motorSpeedCallback(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Here we are in the set motorSpeed callback. Here is the read value: ");
  updateMotorSpeed(motorSpeed.value());
  Serial.println(motorSpeed.value(), DEC);
}

void commandCallback(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Here we are in the command callback. Here is the read value: ");

  char newCommand = command.value();
  
  Serial.println(newCommand);

  updateCommand(newCommand);
}

void ledBrightnessCallback(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Here we are in the led callback. Here is the read value: ");
  
  Serial.println(ledBrightness.value(), DEC);

  updateLedBrightness((float)ledBrightness.value() / 256.0);
}


void setup()
{

  Serial.begin(9600); // init serial communication
  // while (!Serial);
  Wire.begin();       // join i2c bus

  // Configuration of connected pins
  int motor_one_input1 = 5;
  int motor_one_input2 = 3;

  int motor_two_input1 = 6;
  int motor_two_input2 = 9;

  int R_PIN = 10;
  int G_PIN = 11;
  int B_PIN = 12 ;

  motor1 = new DRV8833_MOTOR(motor_one_input1, motor_one_input2);
  motor2 = new DRV8833_MOTOR(motor_two_input1, motor_two_input2);
  flow_meter = new SLF06_FLOW_SENSOR();
  led = new RGB_LED(R_PIN, G_PIN, B_PIN);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("SPIDER");
  BLE.setAdvertisedService(SPIDERService); // add the service UUID

  // Add all supported characteristics to service
  SPIDERService.addCharacteristic(currentFlowRate);
  SPIDERService.addCharacteristic(currentTemp);
  SPIDERService.addCharacteristic(currentMotorRate);
  SPIDERService.addCharacteristic(timeFrame);
  SPIDERService.addCharacteristic(milliliters);
  SPIDERService.addCharacteristic(command);
  SPIDERService.addCharacteristic(motorSpeed);
  SPIDERService.addCharacteristic(ledBrightness);
  

  // Add the service
  BLE.addService(SPIDERService);


  // Initialize characteristic values
  currentFlowRate.writeValue(0.0);
  currentTemp.writeValue(0);
  currentMotorRate.writeValue(0);

  // Wstablish characteristic callbacks
  timeFrame.setEventHandler(BLEWritten, timeFrameCallback);
  milliliters.setEventHandler(BLEWritten, millilitersCallback);
  motorSpeed.setEventHandler(BLEWritten, motorSpeedCallback);
  command.setEventHandler(BLEWritten, commandCallback);
  ledBrightness.setEventHandler(BLEWritten, ledBrightnessCallback);

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  // ScanI2CBus();

}

void loop()
{
  // BLE.poll();

  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {

    Serial.print("Connected to central: ");

    // print the central's BT address:
    Serial.println(central.address());

    // while the central is connected:
    while (central.connected()) {
      updateCurrentCharacteristicValues();
    }

    // When central disconnects
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }

}


void updateCurrentCharacteristicValues() {

  delay(100);
  flow_data data = flow_meter->continuousRead();

  

//  Serial.println("Current flow rate: ");
//  Serial.println(data.flow, DEC);
//  Serial.println(" ml/min \n");
//
//  Serial.println("Current temp: ");
//  Serial.println(data.temp, DEC);
//  Serial.println(" deg C \n");

  currentFlowRate.writeValue(data.flow);
  currentTemp.writeValue(data.temp);
}

void updateCommand(char command) {

  if (command != last_command) {
    switch (command)
    {
      case 'l':
        led->turnOn(Color::RED, 1.0);
        break;

      case 's':
        led->turnOff();
        motor1->stop();
        motor2->stop();
        break;

      case 'm':
        if (motor1 != nullptr && motor2 != nullptr) {
          motor1->forward(last_motor_speed);
          motor2->forward(last_motor_speed);
        }
      default:
        return;
    }

    last_command = command;
  }
}

void updateMotorSpeed(int newSpeed) {

  // currentMotorRate

  if ( newSpeed != last_motor_speed) {
    last_motor_speed = newSpeed;
    currentMotorRate.writeValue(newSpeed);
    motor1->forward(newSpeed);
    motor2->forward(newSpeed);
  }
}

void updateTimeFrame(long timeFrame) {
  if (last_time_frame != timeFrame) {
    last_time_frame = timeFrame;
  }
}

void updateMilliliters(float newMilliliters) {
  if (last_milliliters != newMilliliters) {
    last_milliliters = newMilliliters;
  }
}

void updateLedBrightness(float brightness) {
  led->setBrightness(Color::GREEN, brightness);
}

void ScanI2CBus() {
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
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
