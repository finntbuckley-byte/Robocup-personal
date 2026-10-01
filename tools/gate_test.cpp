#include <Arduino.h>
#include "gate.h"
#include <cassert>
#include <iostream>
unsigned long mockNow=100;int mockGo=0,mockPWM=0;
void poll(unsigned long ms=2100){mockNow+=ms;gate_update();}
int main(){
    gate_init();assert(!gate_found());
    Serial2.responds=true;poll();assert(gate_found());
    assert(Serial2.closeCommands==1 && Serial2.openCommands==0);
    assert(gate_readPos()==895);
    Serial2.responds=false;poll(600);assert(gate_found());
    Serial2.responds=true;poll(600);assert(gate_found());
    Serial2.corrupt=true;assert(gate_readPos()==-1);
    poll(600);poll(600);poll(600);assert(!gate_found());
    Serial2.corrupt=false;poll();assert(gate_found());
    assert(Serial2.closeCommands==2 && Serial2.openCommands==0);
    gate_open();assert(gate_isOpen() && gate_readPos()==613);
    gate_close();assert(!gate_isOpen() && gate_readPos()==895);
    std::cout<<"PASS actual gate driver: late discovery, echo/checksum, transient loss, persistent loss, recovery closes, position feedback\n";
}
