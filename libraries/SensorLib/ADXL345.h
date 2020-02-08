
#include "Arduino.h"



#ifndef ADXL_h
#define ADXL_h

struct accel_data 
{ 
   float x;
   float y;
   float z;
};

struct calibration_data
{ 
   struct accel_data Min;
   struct accel_data Max;
};

typedef enum
{
  RANGE_16_G          = 0b11,  
  RANGE_8_G           = 0b10,  
  RANGE_4_G           = 0b01,  
  RANGE_2_G           = 0b00   
} range_t;

class ADXL {
    private:
        // Calibration values
        float AccelMinX = 0;
        float AccelMaxX = 0;
        float AccelMinY = 0;
        float AccelMaxY = 0;
        float AccelMinZ = 0;
        int AccelMaxZ = 0;
        bool calibrated = false; 
        int calibrateButton = 5;

        // Aceleration range
        float _range = 2.0;

        //Assign the Chip Select signal to pin 10.
        int CS=10;

        //This is a list of some of the registers available on the ADXL345.
        //To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
        char POWER_CTL = 0x2D;  //Power Control Register
        char DATA_FORMAT = 0x31;
        char ADXL345_ACT_INACT_CTL = 0x27;
        char ADXL345_THRESH_ACT = 0x24;
        char DATAX0 = 0x32; //X-Axis Data 0
        char DATAX1 = 0x33; //X-Axis Data 1
        char DATAY0 = 0x34; //Y-Axis Data 0
        char DATAY1 = 0x35; //Y-Axis Data 1
        char DATAZ0 = 0x36; //Z-Axis Data 0
        char DATAZ1 = 0x37; //Z-Axis Data 1


        char OFSX = 0x1E; //X-Axis offset
        char OFSY = 0x1F; //Y-Axis offset
        char OFSZ = 0x20; //Z-Axis offset
    

        void writeRegister(char registerAddress, char value);
        void readRegister(char registerAddress, int numBytes, byte  values[]);
        void setSpiBit(bool spiBit);
        void setRegisterBit(byte regAdress, int bitPos, bool state);
        void setActivityX(bool state);
        void setActivityY(bool state);
        void setActivityZ(bool state);
        void setActivityXYZ(bool stateX, bool stateY, bool stateZ);
        void setActivityThreshold(int activityThreshold);
        struct calibration_data calibrate();
        

    
    public:
        ADXL();
        ~ADXL();

        struct accel_data readAccel();
        struct accel_data readAccelAdjusted(); 
        bool calibrateRequest();
        struct calibration_data performCalibration();
        void setRange(range_t range);

        
        


};

#endif