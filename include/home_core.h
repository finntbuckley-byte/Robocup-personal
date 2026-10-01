#pragma once
#include <cstdint>
namespace home {
enum class State { Idle, Return, SeekColour, Confirm, Align, Reconfirm, Transfer,
                   Open, Unload, Close, ExitTurn, Exit, Failed, Arrived };
struct Policy {
    bool movementOnly=false;
    uint32_t confirmMs=600, turnTimeoutMs=12000;
    uint32_t transferTimeoutMs=8000, gateTimeoutMs=3000, unloadMs=3000;
    uint32_t exitTimeoutMs=10000;
    float arrivalMm=250, turnTolerance=10;
    int speed=35, turn=28; // delivery turn and departure only; travel uses navigation.cpp
    int frontStopMm=270, spinClearMm=150, sideClearMm=120;
};
struct Input {
    uint32_t now=0, remainingMs=0;
    bool running=false, request=false, pickupBusy=false;
    bool poseValid=false, imuFresh=false, frontFresh=false, rearFresh=false;
    bool homeKnown=false, onHome=false, onFloor=false, heldWeight=false, craneBusy=false;
    bool gateReady=false, gateOpen=false, gateClosed=false;
    uint16_t front=0, left=0, right=0, rear=0, sideL=0, sideR=0;
    float x=0,y=0,heading=0; // heading is relative to the orientation recorded at GO
};
struct Output {
    bool ownsDrive=false, releaseHeld=false, openGate=false, closeGate=false;
    bool beginReturn=false, delivered=false, resumeSearch=false;
    // Return travel is executed by the same navigation FSM used for collection.
    bool navigate=false;
    float targetHeading=0;
    int left=0,right=0;
};
float wrap(float);
class Controller {
public:
    explicit Controller(Policy p={}) : p_(p) {}
    Output tick(const Input&);
    State state() const { return state_; }
    const char* reason() const { return reason_; }
    float turnProgress() const { return spinProgress_; }
    void reset() { *this=Controller(p_); }
private:
    Policy p_; State state_=State::Idle;
    uint32_t entered_=0, confirmAt_=0;
    bool confirming_=false, craneSeen_=false, waitingGate_=false;
    uint32_t turnActiveMs_=0, turnTickAt_=0;
    bool turnCommanded_=false;
    float spinProgress_=0,lastHeading_=0;
    int seekIndex_=0;
    const char* reason_="collecting";
    void enter(State,const Input&,const char*);
    const char* pivotBlocker(const Input&) const;
    Output fail(const Input&,const char*);
};
const char* name(State);
}
