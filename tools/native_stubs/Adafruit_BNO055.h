#pragma once
#include "Wire.h"
constexpr int OPERATION_MODE_IMUPLUS=8;
struct Adafruit_BNO055 {
    enum { BNO055_GYRO_DATA_Z_LSB_ADDR=0x18 };
    Adafruit_BNO055(int,int,TwoWire*){}
    bool begin(int){return true;}
    void getCalibration(uint8_t*s,uint8_t*g,uint8_t*a,uint8_t*m){*s=*g=*a=*m=3;}
};
