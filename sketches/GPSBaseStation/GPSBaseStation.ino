#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <Wire.h> 

SFE_UBLOX_GNSS rtk2;

typedef struct latitudeLongitude {
  double lat;
  double lon;
} GPS_t;

void getGPS(GPS_t &loc);
void printGPS(GPS_t &loc);

void setupRTK() {    
  if (!rtk2.begin()) {
    Serial.println(F("u-blox GNSS not detected at default I2C address."));
    while (1);
  }
  rtk2.factoryReset();
  delay(5000);
}

void checkSurveyStatus() {
  Serial.print(F("Time elapsed: "));
  Serial.print((String)rtk2.getSurveyInObservationTime()); // Call the helper function
  Serial.print(F(" ("));
  Serial.print((String)rtk2.packetUBXNAVSVIN->data.dur); // Read the survey-in duration directly from packetUBXNAVSVIN

  Serial.print(F(") Accuracy: "));
  Serial.print((String)rtk2.getSurveyInMeanAccuracy()); // Call the helper function
  Serial.print(F(" ("));

  float meanAcc = ((float)rtk2.packetUBXNAVSVIN->data.meanAcc) / 10000.0;
  Serial.print((String)meanAcc); 
  Serial.println(F(")"));
}

void startServeyIn() {
  rtk2.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3);
  rtk2.setUSBOutput(COM_TYPE_UBX | COM_TYPE_RTCM3);
  rtk2.setUART2Output(COM_TYPE_RTCM3);
  rtk2.setSerialRate(57600, COM_PORT_UART2, 1100);
  bool response = true;
  
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 5); 
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_USB, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_USB, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_USB, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_USB, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_USB, 1);
  // response &= rtk2.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_USB, 5); 
  response &= rtk2.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART2, 1);
  response &= rtk2.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART2, 1);
  response &= rtk2.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART2, 1);
  response &= rtk2.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART2, 1);
  response &= rtk2.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART2, 1);
  response &= rtk2.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART2, 5); 

  if (response == false) {
    Serial.println(F("Messages setup failed"));
    while(1);
  }

  // Check for fresh data
  if (!rtk2.getSurveyStatus(2000)) {
    Serial.println(F("Failed to get Survey-in status"));
    // while (1); //Freeze
  } else {
    // Start survey if able
    if (rtk2.getSurveyInActive()) {
      Serial.print(F("Survey already in progress."));
    } else {
      response = rtk2.enableSurveyMode(60, 5.0); 
      if (response == false){
        Serial.println(F("Survey start failed. Freezing..."));
        while (1);
      }
      Serial.println(F("Survey started..."));
    }
  }
  
  // Begin waiting for survey to complete
  while (!rtk2.getSurveyInValid()) {
    checkSurveyStatus();
    delay(1000);
  }
  
  Serial.println(F("Survey valid!"));
  Serial.println(F("Base survey complete! RTCM now broadcasting."));
  rtk2.saveConfiguration();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println(F("Setting up Base Station"));
  Wire.setClock(400000); 
  setupRTK();
  startServeyIn();
}

void loop()
{
  rtk2.checkUblox();
  GPS_t gps_current;
  getGPS(gps_current);
  printGPS(gps_current);
  delay(1000); 
}


void getGPS(GPS_t &loc){
  int32_t latitude = rtk2.getHighResLatitude();
  int8_t latitudeHp = rtk2.getHighResLatitudeHp();
  int32_t longitude = rtk2.getHighResLongitude();
  int8_t longitudeHp = rtk2.getHighResLongitudeHp();

  loc.lat = ((double)latitude) / 10000000.0; 
  loc.lat += ((double)latitudeHp) / 1000000000.0;
  loc.lon = ((double)longitude) / 10000000.0;
  loc.lon += ((double)longitudeHp) / 1000000000.0;
}

void printGPS(GPS_t &loc){
  
  // Print the lat and lon
  Serial.print("Lat (deg): ");
  Serial.print(loc.lat, 9);
  Serial.print(", Lon (deg): ");
  Serial.println(loc.lon, 9);

  long accuracy = rtk2.getPositionAccuracy();
  Serial.print(F(" Horizontal Positional Accuracy: "));
  Serial.print(accuracy);
  Serial.println(F(" (mm)"));
}