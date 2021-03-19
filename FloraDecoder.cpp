#include "Arduino.h"
#include "FloraDecoder.h" 


float FloraDecoder::sensor_Temperature;
int FloraDecoder::sensor_Moisture;
int FloraDecoder::sensor_Light;
int FloraDecoder::sensor_Conductivity;
int FloraDecoder::sensor_Battery;
bool FloraDecoder::decodeOK_Battery;
bool FloraDecoder::decodeOK_Sensor;


void FloraDecoder::DecodeSensorData(std::string floraData){
  try{
    const char *floraBytes = floraData.c_str();
  
    int16_t* temperature_integer = (int16_t*)floraBytes;
    FloraDecoder::sensor_Temperature = (*temperature_integer) / ((float)10.0);
    
    FloraDecoder::sensor_Moisture = floraBytes[7];
    
    FloraDecoder::sensor_Light = floraBytes[3] + floraBytes[4] * 256;
    FloraDecoder::sensor_Conductivity =  floraBytes[8] + floraBytes[9] * 256;
    if (FloraDecoder::sensor_Temperature>150){
      FloraDecoder::decodeOK_Sensor = false;
    }
    else{
      FloraDecoder::decodeOK_Sensor = true;
    }
  }  
  catch (...) {   
     FloraDecoder::decodeOK_Sensor = false;
  }
  
}

void FloraDecoder::DecodeBattery(std::string floraData){
  try{
    const char *floraBytes = floraData.c_str();
    FloraDecoder::sensor_Battery = floraBytes[0];
    if (FloraDecoder::sensor_Battery>=0 && FloraDecoder::sensor_Battery<=100){
      FloraDecoder::decodeOK_Battery = true;
    }
    else{
      FloraDecoder::decodeOK_Battery = false;
    }
  }
  catch (...) {
    FloraDecoder::decodeOK_Battery = false;
  }
  
}
