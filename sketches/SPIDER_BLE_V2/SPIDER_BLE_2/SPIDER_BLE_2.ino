#include <Wire.h>
#include <ArduinoBLE.h>
#include <Adafruit_PWMServoDriver.h>
#include "SLF06_FLOW_SENSOR.h"
#include "PCA9685_DV8833_MOTOR.h"
#include "PCA9685_RGB_LED.h"


#define MIN_PULSE_WIDTH       1500
#define MAX_PULSE_WIDTH       4095
#define FREQUENCY             120

SLF06_FLOW_SENSOR* flow_meter = nullptr;
Adafruit_PWMServoDriver* pwm = nullptr;
PCA9685_DV8833_MOTOR* motors;
PCA9685_RGB_LED* led = nullptr;

// Current program values.
char last_command = 's';
long last_time_frame = 0;
float last_milliliters = 0;
int last_motor_speed = 0;
float last_brightness = 0.0;
float last_speed = 0.0;

// Control Panel Buttons and knobs
int brightnessButton = 7;
int motorSpeedButton = 8;
int adjustmentUP = A0;
int adjustmentDown = A1;

// BLE SPIDER Service
BLEService SPIDERService("c03b622f-332e-4058-8fa8-f3d3eceafd8b");

// BLE SPIDER Characteristics
// standard 16-bit characteristic UUID 
BLEFloatCharacteristic currentFlowRate("da75b171-a289-4684-9413-3767fe886163",  BLERead | BLENotify); 
BLEFloatCharacteristic currentTemp("644b9364-7b9f-4a5b-94e7-6ec1a5b729c9", BLERead | BLENotify); 
BLEIntCharacteristic currentMotorRate("86fe771d-a1ab-4455-87ec-a9a994787808", BLERead | BLENotify);  

BLELongCharacteristic timeFrame("2a4effe1-0a8c-49f1-98d2-c45658cd93d6", BLEWriteWithoutResponse);
BLEFloatCharacteristic milliliters("d9d993b6-cf4a-42e1-b184-57de95962bec", BLEWriteWithoutResponse);
BLEIntCharacteristic motorSpeed("9b1d7d08-d675-437c-93b4-bc94be9bf51d", BLEWriteWithoutResponse);
BLECharCharacteristic command("1e6c421f-4a2f-42fe-8593-a6e285cf6978", BLEWriteWithoutResponse);
BLEIntCharacteristic ledBrightness("505db17a-83f0-4a06-b62f-6e38ef4cadaf", BLEWriteWithoutResponse);

void setup()
{

    Serial.begin(9600); // init serial communication
    Wire.begin();       // join i2c bus

    ScanI2CBus();
    
    // Configuration of connected hardware
    flow_meter = new SLF06_FLOW_SENSOR();
    pwm = new Adafruit_PWMServoDriver(0x40);
    pwm->begin();
    pwm->setPWMFreq(FREQUENCY);
    motors = new PCA9685_DV8833_MOTOR(pwm, 0, 1, 2, 3, FREQUENCY, MAX_PULSE_WIDTH, MIN_PULSE_WIDTH);
    
    led = new PCA9685_RGB_LED(pwm, 5, 4, 6);
    
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

    // Setup control panel
    pinMode(brightnessButton, INPUT_PULLUP);
    pinMode(motorSpeedButton, INPUT_PULLUP);
    pinMode(adjustmentUP, INPUT_PULLUP);
    pinMode(adjustmentDown, INPUT_PULLUP);
}

void readControlInput() {
  
  while (digitalRead(brightnessButton)==LOW){
    
    float brightness = 0.0;
    if (digitalRead(adjustmentUP)==LOW) {
      brightness = constrain(last_brightness + .01, 0.0, .99);
      last_brightness = brightness;
      updateLedBrightness(brightness);
      delay(20);
    } else if (digitalRead(adjustmentDown)==LOW) {
      brightness = constrain(last_brightness - .01, 0.0, .99);
      last_brightness = brightness;
      updateLedBrightness(brightness);
      delay(20);
    }
  }

  while (digitalRead(motorSpeedButton)==LOW) {
    float new_speed = 0.0;
    if (digitalRead(adjustmentUP)==LOW) {
      new_speed = constrain(last_speed + .01, 0.0, 1.0);
      int motor_speed = new_speed * 255.0;  
      
      if (motor_speed != last_motor_speed) {
        last_motor_speed = motor_speed;
        last_speed = new_speed;
        currentMotorRate.writeValue(motor_speed);
        motors->run(0, 1, motor_speed);
        motors->run(1, 1, motor_speed);
        delay(20);
      }
    } else if (digitalRead(adjustmentDown)==LOW) {
      new_speed = constrain(last_speed - .01, 0.0, 1.0);
      int motor_speed = new_speed * 255.0;  
      
      if (motor_speed != last_motor_speed) {
        last_motor_speed = motor_speed;
        last_speed = new_speed;
        currentMotorRate.writeValue(motor_speed);
        motors->run(0, 1, motor_speed);
        motors->run(1, 1, motor_speed);
        delay(20);
      }
    }
  } 
}

void loop()
{
  // BLE.poll();
  readControlInput();
  
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {

    Serial.print("Connected to central: ");

    // print the central's BT address:
    Serial.println(central.address());

    // while the central is connected:
    while (central.connected()) {
      readControlInput();
      updateCurrentCharacteristicValues();
    }

    // When central disconnects
    Serial.print("Disconnected from central: ");
    Serial.print(central.address());
    Serial.print('\n');
  }

}

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
        led->turnOn(Color::RED, .99);
        break;

      case 's':
        led->turnOff();
        motors->stop();
        break;

      case 'm':
        if (motors != nullptr) {
          motors->run(0, 1, last_motor_speed);
          motors->run(1, 1, last_motor_speed);
        }
      default:
        return;
    }

    last_command = command;
  }
}

void updateMotorSpeed(int newSpeed) {
    Serial.println("Here we are in update motor speed");
    if ( newSpeed != last_motor_speed) {
        last_motor_speed = newSpeed;
        currentMotorRate.writeValue(newSpeed);
        delay(50);
        motors->run(0, 1, newSpeed);
        motors->run(1, 1, newSpeed);
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
  led->setBrightness(Color::RED, brightness);
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
