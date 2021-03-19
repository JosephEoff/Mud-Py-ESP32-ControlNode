#ifndef FloraDecoder_h
#define FloraDecoder_h

#include "Arduino.h"
#include "BLEDevice.h"

class FloraDecoder
{
  public:
    FloraDecoder();
    static void DecodeSensorData(std::string floraData);
    static void DecodeBattery(std::string floraData);
    static float sensor_Temperature;
    static int sensor_Moisture;
    static int sensor_Light;
    static int sensor_Conductivity;
    static int sensor_Battery;
    static bool decodeOK_Battery;
    static bool decodeOK_Sensor;
  private:

 

};

#endif 
