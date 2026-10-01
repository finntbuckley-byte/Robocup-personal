#include <Arduino.h>
#include "config.h"
#include "weight_detect.h"
#include <cassert>
#include <iostream>
unsigned long mockNow=100;int mockGo=0,mockPWM=0;
uint16_t tofMM[5]={0};
uint16_t &tofBL=tofMM[0],&tofBR=tofMM[1],&tofUpright=tofMM[2],&tofRear=tofMM[3],&tofTop=tofMM[4];
bool valid[5]={true,true,true,true,true},frontFresh=true;
uint16_t frontL=1200,frontR=1200;
bool tofOk(int i){return valid[i];}bool x8Fresh(){return frontFresh;}
uint16_t x8LeftMM(){return frontL;}uint16_t x8RightMM(){return frontR;}
uint16_t x8FrontMM(){return !frontL?frontR:(!frontR?frontL:(frontL<frontR?frontL:frontR));}
void tick(unsigned ms=20){mockNow+=ms;weightDetectUpdate();}
void confirm(){tick();tick(WEIGHT_STICK_MS);}
int main(){
    tofTop=400;tick();assert(!weightFound);tick(WEIGHT_STICK_MS);
    assert(weightFound && weightCentreActive && weightSide==0 && weightDistMM==400);
    valid[TOF_TOP]=false;tick();assert(!weightFound && !weightCentreActive);
    valid[TOF_TOP]=true;confirm();assert(weightFound);
    frontFresh=false;tick();assert(!weightFound);frontFresh=true;
    frontL=450;confirm();assert(!weightFound); // wall on either matrix half vetoes top
    frontL=1200;frontR=450;confirm();assert(!weightFound);
    frontR=1200;tofTop=501;confirm();assert(!weightFound);
    tofTop=59;confirm();assert(!weightFound);tofTop=180;confirm();assert(weightFound);
    suppressTargetFor(1000);tick();assert(!weightFound && !weightCentreActive);
    tick(1000);assert(!weightFound);tick(WEIGHT_STICK_MS);assert(weightCentreActive);
    tofTop=0;tofBR=250;confirm();assert(weightFound && !weightCentreActive && weightSide==-1);
    tofBR=0;tofBL=300;confirm();assert(weightSide==1);
    tofBR=250;confirm();assert(weightSide==0 && weightDistMM==250 && !weightCentreActive);
    tofTop=350;confirm();assert(weightCentreActive && weightDistMM==350);
    frontL=frontR=0;confirm();assert(weightCentreActive); // fresh clear matrix
    std::cout<<"PASS actual weight detector: top-only, stale data, walls, range, suppression, crossed lower beams and combined targets\n";
}
