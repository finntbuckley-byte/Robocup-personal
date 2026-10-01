#pragma once
#include <array>
#include <cstdint>
struct TwoWire {
    std::array<uint8_t,256> regs{};
    bool fail=false,shortRead=false;
    unsigned reg=0,pos=0,writes=0;
    void beginTransmission(uint8_t){writes=0;}
    void write(uint8_t b){if(!writes++) reg=b;else regs[reg++]=b;}
    int endTransmission(bool=true){return fail?1:0;}
    int requestFrom(uint8_t,uint8_t n){pos=reg;return fail?0:(shortRead?n-1:n);}
    int read(){return regs[pos++];}
};
inline TwoWire Wire,Wire1;
