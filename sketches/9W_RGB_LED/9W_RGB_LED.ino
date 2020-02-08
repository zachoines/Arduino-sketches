// color swirl! connect an RGB LED to the PWM pins as indicated
// in the #defines
// public domain, enjoy!
 
#define REDPIN 11
#define GREENPIN 10
#define BLUEPIN 9
 
#define FADESPEED 10     // make this higher to slow down

int brightness = 255;

int gBright = 0;
int rBright = 0;
int bBright = 0;
 
void setup() {
  Serial.begin(9600);
  pinMode(GREENPIN, OUTPUT);
  pinMode(REDPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  
  TurnOn();
  delay(5000);
  // TurnOff();
}

void TurnOn() { 
   for (int i = 0; i < 256; i++) {
       analogWrite(REDPIN, rBright);
       rBright +=1;
       delay(FADESPEED);
   }
 
   for (int i = 0; i < 256; i++) {
       analogWrite(BLUEPIN, bBright);
       bBright += 1;
       delay(FADESPEED);
   } 

   for (int i = 0; i < 256; i++) {
       analogWrite(GREENPIN, gBright);
       gBright +=1;
       delay(FADESPEED);
   } 
}

void TurnOff() {
   for (int i = 0; i < 256; i++) {
       analogWrite(GREENPIN, brightness);
       analogWrite(REDPIN, brightness);
       analogWrite(BLUEPIN, brightness);
 
       brightness -= 1;
       delay(FADESPEED);
   }
}
 
 
void loop() {
   int r, g, b;
 
  // fade from blue to violet
  for (r = 0; r < 256; r++) { 
    analogWrite(REDPIN, r);
    Serial.println("here we are");
    delay(FADESPEED);
  } 
  // fade from violet to red
  for (b = 255; b > 0; b--) { 
    analogWrite(BLUEPIN, b);
    delay(FADESPEED);
  } 
  // fade from red to yellow
  for (g = 0; g < 256; g++) { 
    analogWrite(GREENPIN, g);
    delay(FADESPEED);
  } 
  // fade from yellow to green
  for (r = 255; r > 0; r--) { 
    analogWrite(REDPIN, r);
    delay(FADESPEED);
  } 
  // fade from green to teal
  for (b = 0; b < 256; b++) { 
    analogWrite(BLUEPIN, b);
    delay(FADESPEED);
  } 
  // fade from teal to blue
  for (g = 255; g > 0; g--) { 
    analogWrite(GREENPIN, g);
    delay(FADESPEED);
  } 
}
