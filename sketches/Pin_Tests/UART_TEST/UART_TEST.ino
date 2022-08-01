UART MySerial (digitalPinToPinName(4), digitalPinToPinName(3), NC, NC);

void setup() {
  Serial.begin(9600);
  MySerial.begin(9600);
  
  while (!Serial);

}

void loop() {
  if (MySerial.available()) {
    Serial.println(MySerial.read());
  }

}
