#pragma once
#include "../native_stubs/Arduino.h"
#include <vector>
#include <deque>
#include <algorithm>
template<class T> T constrain(T v,T low,T high){return std::max(low,std::min(high,v));}
struct GateSerial {
    bool responds=false,corrupt=false;
    int pos=895,closeCommands=0,openCommands=0;
    std::vector<uint8_t> tx;
    std::deque<uint8_t> rx;
    void begin(int){}
    void write(uint8_t v){tx.push_back(v);}
    int available(){if(rx.empty())++mockNow;return int(rx.size());}
    int read(){int v=rx.front();rx.pop_front();return v;}
    void flush(){
        auto p=tx;tx.clear();
        if(p.size()<7)return;
        // Real level-shifter echoes command packets. Driver must ignore them.
        for(auto b:p)rx.push_back(b);
        if(!responds)return;
        if(p[4]==6){
            pos=p[8]|(p[9]<<8);
            if(pos==895)++closeCommands;
            if(pos==613)++openCommands;
            return;
        }
        std::vector<uint8_t> data;
        if(p[4]==4){
            data={p[7],p[8]};
            for(int j=0;j<p[8];++j)data.push_back(p[7]==58?uint8_t(pos>>(8*j)):100);
        }else if(p[4]!=7)return;
        data.push_back(0);data.push_back(0); // status fields
        uint8_t size=uint8_t(7+data.size()),cmd=p[4]|0x40,cs=size^p[3]^cmd;
        for(auto b:data)cs^=b;
        cs&=0xFE;
        std::vector<uint8_t> out={255,255,size,p[3],cmd,uint8_t(cs^(corrupt?2:0)),uint8_t((~cs)&0xFE)};
        out.insert(out.end(),data.begin(),data.end());
        for(auto b:out)rx.push_back(b);
    }
};
inline GateSerial Serial2;
