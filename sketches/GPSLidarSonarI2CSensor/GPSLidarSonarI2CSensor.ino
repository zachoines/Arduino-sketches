#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <Wire.h> 
#include <RoboClaw.h>
#include <TFMPI2C.h>  // TFMini-Plus I2C
#include "wiring_private.h"

#define MAX_ATTEMPTS 5
#define SLAVE_ADDR 9

SFE_UBLOX_GNSS rtk2;
TwoWire myWire(&sercom0, 6, 5);
TFMPI2C tfmP;

enum Direction {
    NORTH, 
    SOUTH, 
    EAST, 
    WEST
};

union intToBytes {
  char buffer[2];
  uint16_t sensorData;
} converter;

// Datatype for other peripheral sensors
typedef struct AllSensorData {
  float distance_to_obstacle[2];
  double acceleration[3];
  unsigned short distance_to_target;
  unsigned long timestamp;
} sensor_data_t;

typedef struct latitudeLongitude {
  double lat;
  double lon;
} GPS_t;

typedef struct tfData {
  int16_t dist;  
  int16_t flux;  
  int16_t temp;  
} TF_t;

GPS_t gps_current;
GPS_t gps_last;
sensor_data_t lastReading;
TF_t tf;
uint8_t last_command;
uint8_t last_data;
int16_t counter;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  myWire.begin(SLAVE_ADDR);       // join i2c bus with address #9

  pinPeripheral(6, PIO_SERCOM_ALT);   //Assign SDA function to pin 6
  pinPeripheral(5, PIO_SERCOM_ALT);   //Assign SCL function to pin 5

  myWire.onRequest(secondI2CBusRequestEvent);
  myWire.onReceive(secondI2CBusReceiveEvent);

  setupRTK2();
  setupTFMini();
}

void loop()
{
  getTFMiniData(tf);
  Serial.print("TfMini Distance is: "); 
  Serial.print(tf.dist); 
  Serial.println(" mm");
  rtk2.checkUblox(); //See if new data is available. Process bytes as they come in.
  getGPS(gps_current);
  printGPS(gps_current);
  double distance = Haversine(gps_current, gps_last);
  Serial.print("Lat/Long Distance diff is: ");  
  Serial.print(distance * 1000000.00);
  Serial.println(" mm");
  gps_last = gps_current;
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

void setupRTK2() {
  if (!rtk2.begin())  {
    Serial.println("Could not set up rtk2 GPS module");
    while (1);
  }

  if (!rtk2.factoryDefault()){
    Serial.println("!!! Warning: factoryDefault failed !!!");
  } else {
    Serial.println("Configuration set to factory default!");
  }
  
  delay(5000);


  if (!rtk2.setPortInput(COM_PORT_UART2, COM_TYPE_RTCM3)) {
    Serial.println(F("Ublox GPS could not set up RTCM3 on import port. Freezing"));
    while (1);
  } else {
    rtk2.setSerialRate(57600, COM_PORT_UART2, 1100);
  }

  if (rtk2.setNavigationFrequency(40) == false) {
    Serial.println(F("Ublox GPS could not set navigation frequency. Freezing"));
    while (1);
  }

  rtk2.saveConfiguration();
}

void secondI2CBusRequestEvent()
{
  // We will convert float data into a string format by writing to floatToBytes union.
  // Then send the bytes of char array over to the master.
  
  switch (last_command) {

    // Lidar sensor only
    case 0x01:
      converter.sensorData = tf.dist;
      myWire.write(converter.buffer, 2);
      break;
  }
}


void secondI2CBusReceiveEvent(int numReceived) {

  int all_bytes;

  while ( 1 < myWire.available()) // loop through all but the last
  {

    byte extra_zero = myWire.read();
    byte command = myWire.read();
    byte data = myWire.read();
    all_bytes = ((command & 0xFF) << 8) | (data & 0xFF);

    last_command = command;
    last_data = data;
  }
}

// Attach the interrupt handler to the SERCOM
extern "C" {
  void SERCOM0_Handler(void);

  void SERCOM0_Handler(void) {
    myWire.onService();
  }
}

void setupTFMini() {
  Serial.println("INFO: Setting up TFmini Plus... ");
  if (tfmP.sendCommand( SOFT_RESET, 0)) {
    Serial.print( "INFO: TF Mini Connected, ");
    Serial.print( "Firmware version: ");
      
    // if (!tfmP.sendCommand( SET_I2C_MODE, 0)) {  /* Convert between UART and I2C modes */
    //   Serial.println("Could not set i2c mode");
    //   while(true);
    // }
    if (tfmP.sendCommand( GET_FIRMWARE_VERSION, 0)) {
      Serial.print(tfmP.version[0]);
      Serial.print(tfmP.version[1]);
      Serial.println(tfmP.version[2]);

    } else {
      tfmP.printReply();
    };

    if ( !tfmP.sendCommand( SET_FRAME_RATE, FRAME_250)) { 
      tfmP.printReply(); 
    }

    Serial.println("INFO: TFmini Plus connected!");
  } else {
    Serial.println("ERROR: TF MINI Plus not connected.");
    tfmP.printReply();
  }
}

uint16_t getTempF(TF_t &tf) {
  uint16_t tfTempC = uint16_t(( tf.temp / 8) - 256);
  uint16_t tfTempF = uint16_t( tfTempC * 9.0 / 5.0) + 32.0;
  return tfTempF;
}

uint16_t getTempC(TF_t &tf) {
  uint16_t tfTempC = uint16_t(( tf.temp / 8) - 256);
  return tfTempC;
}

void getTFMiniData(TF_t &tf) {
  uint8_t tries = 0;
  while (!tfmP.getData( tf.dist, tf.flux, tf.temp) && tries < MAX_ATTEMPTS) {
    tries++;
  }
}

// This function converts decimal degrees to radians
double deg2rad(double deg) {
  return (deg * M_PI / 180.0);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
  return (rad * 180.0 / M_PI);
}

/**
 * Returns the distance between two points on the Earth.
 * Haversine Formula
 * @param lat1d Latitude of the first point in degrees
 * @param lon1d Longitude of the first point in degrees
 * @param lat2d Latitude of the second point in degrees
 * @param lon2d Longitude of the second point in degrees
 * @return The distance between the two points in kilometers
 */
double Haversine(GPS_t loc1, GPS_t loc2) {
  double earthRadiusKm = 6371.0;
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = deg2rad(loc1.lat);
  lon1r = deg2rad(loc1.lat);
  lat2r = deg2rad(loc2.lat);
  lon2r = deg2rad(loc2.lat);
  u = sin((lat2r - lat1r)/2.0);
  v = sin((lon2r - lon1r)/2.0);
  return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

/**
 * Returns new GPS location given vector (distance and direction)
 * @param loc Current GPS coord
 * @param distance distance in meters to travel
 * @param Direction the direction to move
 * @return new gps pair with updated location
 */
GPS_t move_location(GPS_t loc, double distance, Direction direction) {
    double equator_circumference = 6371000.0;
    double polar_circumference = 6356800.0;

    double m_per_deg_long =  360.0 / polar_circumference;
    double rad_lat = (loc.lat * (PI) / 180.0);
    double m_per_deg_lat = 360.0 / ( cos(loc.lat) * equator_circumference);

    double deg_diff_long = distance * m_per_deg_long;
    double deg_diff_lat = distance * m_per_deg_lat; 

    switch (direction) {
      case NORTH:
        loc.lat += deg_diff_long;
        return loc;
      case SOUTH:
        loc.lat -= deg_diff_long;
        return loc;
      case EAST:
        loc.lon += deg_diff_lat;  
        return loc;
      case WEST:
        loc.lon -= deg_diff_lat;
        return loc;
      default:
        return loc;
    }
}
