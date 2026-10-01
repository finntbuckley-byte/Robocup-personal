#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include <cassert>
#include <iostream>
unsigned long mockNow=100;int mockGo=0,mockPWM=0;
void put(unsigned reg,int16_t v){Wire.regs[reg]=v&255;Wire.regs[reg+1]=(uint16_t(v)>>8);}
void tick(unsigned long ms=25){mockNow+=ms;imuUpdate();}
int main(){
    Wire.regs[0]=0xA0;Wire.regs[0x3D]=8;
    put(0x1A,30*16);put(0x18,-10*16);put(0x1C,20*16);put(0x1E,-15*16);
    assert(imuInit());imuZero();assert(imuOk());
    put(0x1A,120*16);tick();assert(imuOk());assert(std::abs(imuHeadingDeg()-90)<.01);
    assert(std::abs(imuRateDps()-10)<.01); // sign from physical config
    for(int pitch=-90;pitch<=90;++pitch){
        put(0x1E,pitch*16);put(0x1C,-pitch*16);tick();
        assert(imuOk());assert(std::abs(imuHeadingDeg()-90)<.01);
    }
    put(0x1A,359*16);tick();assert(std::abs(imuHeadingDeg()+31)<.01);
    Wire.fail=true;tick();assert(imuOk()); // bounded cached sample
    tick(150);assert(!imuOk());assert(imuResetCount()==0);
    Wire.fail=false;put(0x1A,1*16);tick();assert(imuOk());
    assert(std::abs(imuHeadingDeg()+29)<.01); // no re-zero on transfer failure
    Wire.shortRead=true;tick();assert(imuOk());tick(150);assert(!imuOk());
    Wire.shortRead=false;tick();assert(imuOk());
    put(0x1A,400*16);tick();assert(std::abs(imuHeadingDeg()+29)<.01);
    tick(150);assert(!imuOk());put(0x1A,1*16);tick();assert(imuOk());
    Wire.regs[0x3D]=0;tick();assert(!imuOk() && imuResetCount()==1);
    put(0x1A,0);tick(120);assert(imuOk());assert(std::abs(imuHeadingDeg()+29)<.01);
    imuZero();assert(std::abs(imuHeadingDeg())<.01);
    std::cout<<"PASS actual IMU driver: yaw/gyro register map, 181 pitch/roll cases, wrap, transfer loss, corrupt data, reset\n";
}
