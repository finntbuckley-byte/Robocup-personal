#include "round.h"
#include "config.h"
#include "colour.h"
#include <cassert>
#include <iostream>
unsigned long mockNow=100;int mockGo=0,mockPWM=0;
int realWeightCount=0,dummyCount=0;
static bool fresh=true;
static ColourSurface surface=COLOUR_FLOOR;
bool colourOk(){return fresh;}
ColourSurface colourSurface(){return surface;}
void tick(unsigned long ms){mockNow+=ms;roundUpdate();}
void start(){mockGo=0;roundInit();tick(20);tick(100);mockGo=1;tick(20);tick(100);assert(roundRunning());mockGo=0;tick(20);}
int main(){
    start();assert(roundWantsWeights());
    noteCollected();noteCollected();assert(!roundWantsHome());
    noteCollected();assert(roundRunning() && roundWantsHome());assert(!roundWantsWeights());
    const auto timeBefore=roundElapsedMs();roundAcknowledgeReturn();noteDelivered(3);
    assert(targetsOnBoard()==0 && roundWantsWeights());assert(roundElapsedMs()==timeBefore);
    tick(RETURN_HOME_AT_MS-roundElapsedMs()-1);assert(!roundWantsHome());
    tick(1);assert(roundWantsHome());roundAcknowledgeReturn();noteDelivered(0);
    assert(!roundWantsHome());tick(1000);assert(roundWantsWeights());
    noteCollected();noteCollected();noteCollected();assert(roundWantsHome());
    noteDelivered(3);assert(!roundWantsHome());
    surface=COLOUR_GREEN;assert(!roundWantsWeights());surface=COLOUR_FLOOR;fresh=false;assert(!roundWantsWeights());fresh=true;
    tick(ROUND_MS-ROUND_END_MARGIN_MS-roundElapsedMs());assert(roundOver());
    mockGo=1;tick(500);assert(roundOver());
    start();noteCollected();noteCollected();noteCollected();roundAcknowledgeReturn();
    tick(RETURN_HOME_AT_MS-roundElapsedMs()+1);noteDelivered(3);
    assert(!roundWantsHome()); // capacity trip crossed late-return deadline
    std::cout<<"PASS actual round code: capacity, 88.5s trigger, repeated trips, original deadline, base pickup gating\n";
}
