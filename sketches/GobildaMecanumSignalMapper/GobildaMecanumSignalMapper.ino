#include <RoboClaw.h>
#include <SoftwareSerial.h>
#include <ArduinoSTL.h>
#include "Arduino.h"
#define num_interrupts 3
#define rc_address 0x80
volatile int pwm_value[] = { 1400, 1400, 1400 };
volatile int pwm_value_centers[] = { 1400, 1400, 1400 };
volatile int pwm_value_highs[] = { 1900, 1900, 1900 };
volatile int pwm_value_lows[] = { 1100, 1100, 1100 };
volatile int prev_time[] = { 0, 0, 0 };
int interrupt_pins[] = { 5, 4, 2 }; // RY, RX & LX, respectively

// Function hoisting
static double mapOutput(double x, double in_min, double in_max, double out_min, double out_max);
void R1(), R2(), R3(), F1(), F2(), F3();
void calibratePWMs();
int updatePWMBounds();

struct Point {
  int y;
  int x;
};

class LineVector  {
  
private:
  Point p;
  Point q;

public:
  double X; // X component
  double Y; // Y component
  double M; // Magnitude
  double A; // Angle of unit circle 
  
  LineVector::LineVector(Point p1, Point p2) {
    p = p1;
    q = p2;
    X = q.x - p.x;
    Y = q.y - p.y;
    M = sqrt(pow(X, 2) + pow(Y, 2));
    A = atan2(Y, X);
    A *= 180.0 / PI; // To degrees -180 <-> 180
    A = A < 0 ? A + 360 : A;  // To 0 <-> 360
  }
};

/* 
  Ensure motors are laid out in this fashion:

  (M1)  TL _____TR (M2)
          |     |
          |     |
          |_____|
  (M2)  BL       BR (M1)
  (Left)         (Right) 
  
  Left motor channel: TR <---> BR
  Right motor channel: TL <---> BL
  
*/
class MacanumDriveController {
private:
  RoboClaw* right = nullptr;
  RoboClaw* left = nullptr;  
  int address = -1;
  double currentDrives[4] = { 0.0 };

  void _setDrive(double TL, double TR, double BL, double BR) {
    TL = mapOutput(TL, -1.0, 1.0, -32768.0, 32767.0);
    TR = mapOutput(TR, -1.0, 1.0, -32768.0, 32767.0);
    BL = mapOutput(BL, -1.0, 1.0, -32768.0, 32767.0);
    BR = mapOutput(BR, -1.0, 1.0, -32768.0, 32767.0);

    right->DutyM1M2(address, (int)BR, (int)TR);
    left->DutyM1M2(address, (int)TL, (int)BL);
  }

  void _setRotation(double direction, double magnitude) {
    double dir = direction > 0.0 ? 1.0 : -1.0;
    double r = -dir * magnitude;
    double l = dir * magnitude;
    currentDrives[0] = constrain(currentDrives[0] + -l, -1.0, 1.0); // TL
    currentDrives[1] = constrain(currentDrives[1] + -r, -1.0, 1.0); // TR
    currentDrives[2] = constrain(currentDrives[2] + l, -1.0, 1.0); // BL
    currentDrives[3] = constrain(currentDrives[3] + r, -1.0, 1.0); // BR
  }

public:
  enum Q {
    I,
    II,
    III,
    IV          
  };

  MacanumDriveController(RoboClaw* r, RoboClaw* l, int address) {
    right = r;
    left = l;
    this->address = address;
  }

  int quadrant(double angle) {
    if (angle >= 0.0 && angle < 90.0) return Q::I;
    if (angle >= 90.0 && angle < 180.0) return Q::II;
    if (angle >= 180.0 && angle < 270.0) return Q::III;
    if (angle >= 270.0 && angle < 360.0) return Q::IV;
    return -1;
  }

  void setDrive(LineVector direction, LineVector rotation) {
    double angle = direction.A;
    double magnitude = direction.M;
    double RRatio = 1.0; // Ratio betweem Left and Right motor drivers 
    double LRatio = 1.0;
    double LDrive = 1.0; // Controls M1 <---> M4
    double RDrive = 1.0; // COntrols M2 <---> M3

    switch (quadrant(angle)) {
      case Q::I:
        // LRatio constant: 1.0
        // angle = 45 * (1.0 + RRatio)
        RRatio = (angle / 45.0) - 1.0;
        RDrive = RRatio * magnitude;
        LDrive = LRatio * magnitude;    
        currentDrives[0] = -LDrive;
        currentDrives[1] = -RDrive;
        currentDrives[2] = RDrive;
        currentDrives[3] = LDrive;
        
        // Serial.print("RDRIVE: " + String(RDrive));
        // Serial.print(", LDRIVE: " + String(LDrive));
        // Serial.print(", ANGLE: " + String(angle));
        // Serial.print(", MAG: " + String(magnitude));
        break;
      case Q::II:
        // RRatio constant: 1.0
        // angle = 45 * (1.0 - LRatio) + 90
        LRatio = -((angle - 90.0) / 45.0) + 1.0;
        RDrive = RRatio * magnitude;
        LDrive = LRatio * magnitude;
        currentDrives[0] = -LDrive;
        currentDrives[1] = -RDrive;
        currentDrives[2] = RDrive;
        currentDrives[3] = LDrive;
        break;
      case Q::III:
        // LRatio constant: -1.0
        // angle = 45 * ((-RRatio) - (- 1)) + 180
        RRatio = -((angle - 180.0) / 45.0) + 1.0;
        RDrive = RRatio * magnitude;
        LDrive = LRatio * magnitude;
        currentDrives[0] = LDrive;
        currentDrives[1] = -RDrive;
        currentDrives[2] = RDrive;
        currentDrives[3] = -LDrive;
        break;
      case Q::IV:
        // RRatio constant: -1.0
        // angle = 45 * (1 + LRatio)) + 270.0
        LRatio = ((angle - 270.0) / 45.0) - 1.0;
        RDrive = RRatio * magnitude;
        LDrive = LRatio * magnitude;
        currentDrives[0] = -LDrive;
        currentDrives[1] = RDrive;
        currentDrives[2] = -RDrive;
        currentDrives[3] = LDrive;
        break;
      default:
        currentDrives[0] = 0.0;
        currentDrives[1] = 0.0;
        currentDrives[2] = 0.0;
        currentDrives[3] = 0.0;        
    }

    _setRotation(rotation.X, rotation.M);
    _setDrive(currentDrives[0], currentDrives[1], currentDrives[2], currentDrives[3]);
  }
};

void (*FunctionPointers[])() = { R1, R2, R3, F1, F2, F3 };
SoftwareSerial serial2(7, 8); // RX, TX
SoftwareSerial serial3(9, 10); // RX, TX
RoboClaw* roboclawR = nullptr;
RoboClaw* roboclawL = nullptr;
MacanumDriveController* vehicle = nullptr;

void setup() {
  Serial.begin(115200);
  roboclawR = new RoboClaw(&serial3,10000);
  roboclawL = new RoboClaw(&serial2,10000);
  roboclawR->begin(38400);
  roboclawL->begin(38400);
  vehicle = new MacanumDriveController(roboclawR, roboclawL, rc_address);

  // Initial state of pins and interrupts
  for (int i = 0; i < num_interrupts; i++) {
    int pin = interrupt_pins[i];
    pinMode(pin, INPUT); 
    digitalWrite(pin, HIGH);
    attachInterrupt(
      digitalPinToInterrupt(pin), 
      FunctionPointers[i],
      RISING
    );
  }
  delay(100);
  calibratePWMs();
  updatePWMBounds();
}

void loop() { 
  
  Serial.print(", PWM1: " + String(pwm_value[0]));
  Serial.print(", PWM2: " + String(pwm_value[1]));
  Serial.println(", PWM3: " + String(pwm_value[2]));
          
  LineVector direction({ pwm_value_centers[0], pwm_value_centers[1] }, { pwm_value[0], pwm_value[1]});
  LineVector rotation({ 0, pwm_value_centers[2] }, { 0, pwm_value[2]});
  
  int max_magnitude = updatePWMBounds();
  rotation.M = constrain(rotation.M / max_magnitude, 0.0, 1.0);
  direction.M = constrain(direction.M / max_magnitude, 0.0, 1.0);
  vehicle->setDrive(direction, rotation);

  delay(150);
}

int updatePWMBounds() {
  int max_magnitude = 0;
  for (int i = 0; i < num_interrupts; i++) {
    int current = pwm_value[i];

    pwm_value_highs[i] = max(current, pwm_value_highs[i]);
    pwm_value_lows[i] = min(current, pwm_value_lows[i]);

    int highRange = pwm_value_highs[i] - pwm_value_centers[i];
    int lowRange =  pwm_value_centers[i] - pwm_value_lows[i];
    max_magnitude += ((highRange + lowRange) / 2);
  }

  return max_magnitude / 3;  
}

void calibratePWMs() {
  
  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < num_interrupts; j++) {
      // Simple moving average
      pwm_value_centers[j] += (pwm_value[j] - pwm_value_centers[j]) / (i + 1);
    }
    
    delay(10);
  }
}

static double mapOutput(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 
void R1 () {
  attachInterrupt(
    digitalPinToInterrupt(interrupt_pins[0]),
    F1, 
    FALLING
  );
  prev_time[0] = micros();
}

void F1 () {
  attachInterrupt(
    digitalPinToInterrupt(interrupt_pins[0]), 
    R1,
    RISING
  );
  pwm_value[0] = micros()-prev_time[0]; 
}

void R2 () {
  attachInterrupt(
    digitalPinToInterrupt(interrupt_pins[1]),
    F2, 
    FALLING
  );
  prev_time[1] = micros();
}

void F2 () {
  attachInterrupt(
    digitalPinToInterrupt(interrupt_pins[1]), 
    R2,
    RISING
  );
  pwm_value[1] = micros()-prev_time[1];
}

void R3 () {
  attachInterrupt(
    digitalPinToInterrupt(interrupt_pins[2]),
    F3, 
    FALLING
  );
  prev_time[2] = micros();
}

void F3 () {
  attachInterrupt(
    digitalPinToInterrupt(interrupt_pins[2]), 
    R3,
    RISING
  );
  pwm_value[2] = micros()-prev_time[2]; 
}