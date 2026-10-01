#pragma once
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>
using byte=uint8_t;
constexpr int HIGH=1,LOW=0,INPUT=0,INPUT_PULLUP=2,OUTPUT=1,A9=23,A8=22,HEX=16;
constexpr float PI=3.14159265358979323846f;
#define F(s) s
extern unsigned long mockNow;
extern int mockGo,mockPWM;
inline unsigned long millis(){return mockNow;}
inline void delay(unsigned long n){mockNow+=n;}
inline int digitalRead(int){return mockGo;}
inline void pinMode(int,int){}
inline void analogWrite(int,int v){mockPWM=v;}
struct MockSerial {
    std::string printed;
    template<typename... T> void printf(const char* fmt,T... args){
        char line[1024];std::snprintf(line,sizeof(line),fmt,args...);printed+=line;
    }
    template<typename... T> void print(T...){}
    template<typename... T> void println(T...){}
};
inline MockSerial Serial;
