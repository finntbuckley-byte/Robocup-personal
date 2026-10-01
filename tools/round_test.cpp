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
bool colourOnHomeBase(){return fresh && surface==COLOUR_GREEN;}
void tick(unsigned long ms){mockNow+=ms;roundUpdate();}
void start(){mockGo=0;roundInit();tick(20);tick(100);mockGo=1;tick(20);tick(100);assert(roundRunning());mockGo=0;tick(20);}
int main(){
    start();assert(roundWantsWeights());
    noteCollected();noteCollected();assert(!roundWantsHome());
    noteCollected();assert(roundRunning() && roundWantsHome());assert(!roundWantsWeights());
    const auto timeBefore=roundElapsedMs();roundAcknowledgeReturn();noteDelivered(3);
    assert(targetsOnBoard()==0 && roundWantsWeights());assert(roundElapsedMs()==timeBefore);
    tick(RETURN_HOME_AT_MS-roundElapsedMs()-1);assert(!roundWantsHome());
    tick(1);assert(!roundWantsHome()); // no empty timed return
    noteCollected();assert(roundWantsHome());roundAcknowledgeReturn();noteDelivered(1);
    assert(!roundWantsHome());tick(1000);assert(roundWantsWeights());
    noteCollected();noteCollected();noteCollected();assert(roundWantsHome());
    noteDelivered(3);assert(!roundWantsHome());
    surface=COLOUR_GREEN;assert(!roundWantsWeights());surface=COLOUR_FLOOR;fresh=false;assert(!roundWantsWeights());fresh=true;
    tick(ROUND_MS-ROUND_END_MARGIN_MS-roundElapsedMs());
    if(ROUND_TIME_LIMIT_ENABLED) assert(roundOver());
    else {
        assert(roundRunning() && roundWantsWeights());tick(300000);
        assert(roundRunning() && roundWantsWeights());
        mockGo=1;tick(20);tick(60);assert(roundOver()); // stop holds until reset
    }
    mockGo=0;tick(100);mockGo=1;tick(100);assert(roundOver());
    start();noteCollected();noteCollected();noteCollected();roundAcknowledgeReturn();
    tick(RETURN_HOME_AT_MS-roundElapsedMs()+1);noteDelivered(3);
    assert(!roundWantsHome()); // capacity trip crossed late-return deadline
    start();surface=COLOUR_GREEN;assert(!roundWantsHome());
    noteCollected();assert(roundWantsHome());
    surface=COLOUR_BLUE;assert(!roundWantsHome()); // opponent colour
    surface=COLOUR_GREEN;fresh=false;assert(!roundWantsHome());fresh=true;
    assert(roundWantsHome());noteDelivered(1);assert(!roundWantsHome());
    std::cout<<"PASS round: incidental loaded home, no empty return, GO latched stop, timed="<<ROUND_TIME_LIMIT_ENABLED<<"\n";
}
