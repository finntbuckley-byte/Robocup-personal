#include <Arduino.h>
#include "collection.h"
#include "config.h"
#include <cassert>
#include <iostream>
unsigned long mockNow=100;int mockGo=0,mockPWM=0;
void step(){mockNow+=10;collection_update();}
void finish(){const auto start=mockNow;while(collection_busy() && mockNow-start<8000)step();assert(!collection_busy());}
int main(){
    collection_init();assert(mockPWM==0);assert(!collection_release_held());
    collection_start(true);
    while(collection_arm_angle()!=CRANE_PICKUP_ANGLE)step();
    const auto arrived=mockNow;
    while(mockNow-arrived<CRANE_PICKUP_SEAT_MS-20){step();assert(collection_arm_angle()==CRANE_PICKUP_ANGLE);}
    finish();assert(collection_holding());assert(mockPWM==255);
    for(int k=0;k<1000;++k) step();
    assert(collection_holding() && mockPWM==255);
    assert(collection_release_held());assert(!collection_release_held());
    finish();assert(!collection_holding());assert(mockPWM==0);assert(collection_arm_angle()==CRANE_REST_ANGLE);
    collection_start(false);finish();assert(!collection_holding() && mockPWM==0);
    collection_start(true);finish();assert(collection_holding() && mockPWM==255);
    assert(collection_release_held());step();collection_stop_motion();
    const int stoppedAngle=collection_arm_angle();
    for(int k=0;k<1000;++k) step();
    assert(!collection_busy() && collection_holding() && mockPWM==255);
    assert(collection_arm_angle()==stoppedAngle);
    std::cout<<"PASS actual crane code: seated wait, sustained 100% hold, transfer, subsequent normal and held pickups\n";
}
