#include "home_core.h"
#include <cassert>
#include <iostream>
#include <random>
#include <string>
using namespace home;
struct Rig {
    Controller c;Input i;Output o;
    Rig() {
        i.running=i.poseValid=i.imuFresh=i.frontFresh=i.rearFresh=i.gateReady=i.homeKnown=true;
        i.front=i.left=i.right=i.rear=1000;i.sideL=i.sideR=300;
        i.remainingMs=90000;i.x=1000;i.heading=180;
    }
    void tick(uint32_t ms=20) {i.now+=ms;o=c.tick(i);}
    void start() {i.request=true;tick();assert(o.beginReturn);i.request=false;}
    void arrive() {i.x=0;i.onHome=true;tick();tick(620);assert(c.state()==State::Align);}
    void spin() {
        // Simulated arrival at this round's GO reference, regardless of approach.
        i.heading=0;tick(1000);
        assert(c.state()==State::Reconfirm);assert(!o.openGate);
    }
    void unload() {
        i.gateOpen=true;tick();assert(c.state()==State::Unload);
        tick(3020);assert(o.closeGate);assert(!o.delivered);
        i.gateOpen=false;i.gateClosed=true;tick();assert(o.delivered);
        i.heading=0;tick();assert(c.state()==State::Exit);
        i.onHome=false;i.onFloor=true;tick();tick(620);
        assert(o.resumeSearch);assert(c.state()==State::Idle);
    }
};
int main() {
    // Both recorded arrival headings, either turning direction, wrap boundary,
    // and already aligned: all must finish at this round's zero reference.
    for(float heading:{112.f,50.f,-170.f,170.f,179.f,-179.f,0.f,8.f,-8.f}) {
        Rig a;a.start();a.i.heading=heading;a.arrive();a.tick();
        if(std::fabs(heading)>10) {
            assert(a.c.state()==State::Align && !a.o.openGate);
            assert((heading>0 && a.o.left<0 && a.o.right>0) ||
                   (heading<0 && a.o.left>0 && a.o.right<0));
            a.i.heading=heading>0?11:-11;a.tick();assert(a.c.state()==State::Align);
            a.i.heading=heading>0?9:-9;a.tick();
        }
        assert(a.c.state()==State::Reconfirm && !a.o.left && !a.o.right && !a.o.openGate);
        a.tick(580);assert(!a.o.openGate);a.tick(40);assert(a.o.openGate);
    }
    Rig drift;drift.start();drift.arrive();drift.spin();drift.i.heading=18;drift.tick(620);
    assert(drift.c.state()==State::Align && !drift.o.openGate);
    drift.i.heading=0;drift.tick();drift.tick(580);assert(!drift.o.openGate);
    drift.tick(40);assert(drift.o.openGate);
    // Alignment by itself never authorizes release without the home colour.
    Rig colourLost;colourLost.start();colourLost.arrive();colourLost.spin();
    colourLost.i.onHome=false;colourLost.tick(2000);assert(!colourLost.o.openGate && !colourLost.o.releaseHeld);
    colourLost.i.onHome=true;colourLost.tick(300);assert(!colourLost.o.openGate);
    colourLost.tick(320);assert(!colourLost.o.openGate);colourLost.tick(300);assert(colourLost.o.openGate);
    Rig idle;idle.tick();assert(!idle.o.ownsDrive);
    idle.i.pickupBusy=true;idle.i.request=true;idle.tick();assert(!idle.o.beginReturn);
    idle.i.pickupBusy=false;idle.tick();assert(idle.o.beginReturn);
    Rig r;r.start();r.tick();assert(r.o.navigate && std::fabs(wrap(r.o.targetHeading-180))<0.01f);
    r.i.onHome=true;r.tick();assert(r.c.state()==State::Return); // wrong place
    r.i.onHome=false;r.i.x=0;r.tick();assert(r.c.state()==State::SeekColour);
    r.i.onHome=true;r.tick();r.tick(620);assert(r.c.state()==State::Align);
    r.spin();r.tick(620);assert(r.o.openGate);r.unload();
    r.i.x=1000;r.i.onFloor=false;r.start(); // second trip same controller
    r.i.heldWeight=true;r.arrive();r.spin();r.tick(620);
    assert(!r.o.releaseHeld && r.o.openGate);assert(r.c.state()==State::Open);
    r.tick(1000);assert(!r.o.releaseHeld); // wait for actual open feedback
    r.i.gateOpen=true;r.tick();assert(r.c.state()==State::Unload);
    r.tick(2500);assert(!r.o.releaseHeld && !r.o.closeGate && r.i.heldWeight);
    r.tick(520);assert(r.o.releaseHeld && !r.o.closeGate && r.c.state()==State::Transfer);
    r.i.craneBusy=true;r.tick();r.tick(1000);assert(!r.o.openGate);
    r.i.craneBusy=false;r.i.heldWeight=false;r.tick();assert(!r.o.openGate && !r.o.closeGate);
    assert(r.c.state()==State::Unload);r.unload();
    Rig wrong;wrong.start();wrong.i.x=0;wrong.i.onHome=false;wrong.tick();wrong.tick(8020);
    assert(wrong.c.state()==State::SeekColour && !wrong.o.openGate);
    wrong.tick(40000);assert(wrong.c.state()==State::SeekColour);
    Rig debounce;debounce.start();debounce.i.x=0;debounce.i.onHome=true;debounce.tick();debounce.tick(400);
    debounce.i.onHome=false;debounce.tick();debounce.i.onHome=true;debounce.tick();debounce.tick(400);
    assert(debounce.c.state()==State::Confirm);debounce.tick(220);assert(debounce.c.state()==State::Align);
    Rig pos;pos.start();pos.i.poseValid=false;pos.tick();assert(pos.c.state()==State::Failed);
    pos.i.poseValid=true;pos.tick();assert(pos.o.left==0 && pos.o.right==0);
    Rig obstacle;obstacle.start();obstacle.i.front=100;obstacle.tick();assert(!obstacle.o.left && !obstacle.o.right);
    obstacle.i.front=200;obstacle.tick();assert(obstacle.o.navigate);
    Rig rear;rear.start();rear.arrive();rear.i.rearFresh=false;rear.tick();assert(rear.o.left && rear.o.right);
    rear.tick(12000);assert(rear.c.state()==State::Failed); // commanded turn made no progress
    Rig timeout;timeout.start();timeout.tick(25000);
    assert(timeout.c.state()==State::Return && timeout.o.navigate);
    timeout.tick(60000);assert(timeout.c.state()==State::Return && timeout.o.navigate);
    timeout.i.running=false;timeout.tick();assert(!timeout.o.left && !timeout.o.right);
    Rig late;late.start();late.i.remainingMs=10000;late.i.x=0;late.i.onHome=true;late.tick();late.tick(620);
    assert(late.c.state()==State::Failed && !late.o.openGate);
    Rig lost;lost.start();lost.arrive();lost.spin();lost.i.onHome=false;lost.tick();
    assert(lost.c.state()==State::Reconfirm && !lost.o.openGate && !lost.o.left && !lost.o.right);
    lost.tick(2000);assert(!lost.o.openGate);
    lost.i.onHome=true;lost.tick(300);assert(!lost.o.openGate);
    lost.tick(320);assert(!lost.o.openGate);lost.tick(300);assert(lost.o.openGate);
    // Observed home contacts rejected by the old 250 mm gate, plus boundaries.
    Policy broad;broad.arrivalMm=600;
    for(float distance:{255.f,266.f,302.f,339.f,452.f,533.f,600.f}) {
        Rig area;area.c=Controller(broad);area.start();area.i.x=distance;area.i.onHome=true;
        area.tick();area.tick(580);assert(area.c.state()==State::Confirm);
        area.tick(40);assert(area.c.state()==State::Align);area.spin();area.tick(620);
        assert(area.o.openGate);
    }
    for(bool ownColour:{false,true}) {
        Rig far;far.c=Controller(broad);far.start();far.i.x=ownColour?601:339;
        far.i.onHome=ownColour;far.tick();far.tick(620);
        assert(far.c.state()==State::Return && far.o.navigate && !far.o.openGate);
    }
    Rig gate;gate.start();gate.arrive();gate.spin();gate.tick(620);gate.tick(3020);
    assert(gate.c.state()==State::Failed && gate.o.closeGate && !gate.o.delivered);
    const std::string gateFailure=gate.c.reason();
    gate.i.running=false;gate.tick();gate.tick();assert(gateFailure==gate.c.reason());
    Rig noGate;noGate.i.gateReady=false;noGate.start();noGate.tick();
    assert(noGate.c.state()==State::Return && noGate.o.navigate);
    noGate.i.x=0;noGate.i.onHome=true;noGate.tick();noGate.tick(620);
    assert(noGate.c.state()==State::Align && !noGate.o.openGate && !noGate.o.releaseHeld);
    noGate.spin();noGate.tick(620);
    assert(noGate.c.state()==State::Reconfirm && !noGate.o.releaseHeld && !noGate.o.openGate);
    noGate.i.gateReady=true;noGate.tick();noGate.tick(620);
    assert(noGate.o.openGate);noGate.unload();
    // Corner-base distances must not block the same in-place pivot used in nav.
    Rig corner;corner.start();corner.arrive();corner.i.front=100;corner.i.rear=90;
    corner.i.sideL=100;corner.i.sideR=0;corner.tick();
    assert(corner.o.left && corner.o.right);corner.spin();corner.tick(620);
    assert(corner.o.openGate);
    // Waiting for a fresh front frame does not consume motor-on turn time.
    Rig stale;stale.start();stale.arrive();stale.i.frontFresh=false;stale.tick(13000);
    assert(stale.c.state()==State::Align && !stale.o.left && !stale.o.right);
    stale.i.frontFresh=true;stale.spin();stale.tick(620);assert(stale.o.openGate);
    // Gate reconnect closes the physical gate: repeat OPEN and its dwell.
    Rig reconnect;reconnect.start();reconnect.arrive();reconnect.spin();reconnect.tick(620);
    reconnect.i.gateOpen=true;reconnect.tick();assert(reconnect.c.state()==State::Unload);
    reconnect.i.gateReady=false;reconnect.i.gateOpen=false;reconnect.tick(4000);
    assert(reconnect.c.state()==State::Unload && !reconnect.o.delivered);
    reconnect.i.gateReady=true;reconnect.tick();assert(reconnect.o.openGate);
    reconnect.unload();
    Rig blocked;blocked.start();blocked.i.heading=0;blocked.i.rear=0;blocked.tick();
    assert(blocked.o.navigate); // clearance belongs to shared navigation
    blocked.i.front=200;blocked.tick();assert(blocked.o.navigate);
    blocked.i.front=1000;blocked.i.rearFresh=false;blocked.tick();assert(blocked.o.navigate);
    Rig noColour;noColour.i.homeKnown=false;noColour.start();noColour.tick();
    assert(noColour.o.navigate); // home label doesn't invalidate encoders
    noColour.i.x=0;noColour.i.onHome=true;noColour.tick();
    assert(noColour.c.state()==State::Failed && !noColour.o.openGate);
    Rig nan;nan.start();nan.i.heading=std::nanf("");nan.tick();assert(nan.c.state()==State::Failed);
    Rig spinRear;spinRear.start();spinRear.arrive();spinRear.i.rear=0;spinRear.tick();
    assert(spinRear.o.left<0 && spinRear.o.right>0); // rear zero does not block heading alignment
    Rig drop;drop.start();drop.arrive();drop.spin();drop.tick(620);drop.i.gateOpen=true;drop.tick();
    drop.i.running=false;drop.tick();assert(drop.o.closeGate && !drop.o.delivered);
    Rig transfer;transfer.start();transfer.i.heldWeight=true;transfer.arrive();transfer.spin();transfer.tick(620);
    transfer.i.gateOpen=true;transfer.tick();transfer.tick(3020);assert(transfer.o.releaseHeld);
    transfer.tick(8020);assert(transfer.c.state()==State::Failed && !transfer.o.openGate);
    Rig blockedThird;blockedThird.start();blockedThird.i.heldWeight=true;
    blockedThird.arrive();blockedThird.spin();blockedThird.tick(620);
    blockedThird.tick(3020);assert(blockedThird.c.state()==State::Failed && !blockedThird.o.releaseHeld);
    Rig lostGate;lostGate.start();lostGate.i.heldWeight=true;lostGate.arrive();lostGate.spin();lostGate.tick(620);
    lostGate.i.gateOpen=true;lostGate.tick();lostGate.tick(3020);assert(lostGate.o.releaseHeld);
    lostGate.i.craneBusy=true;lostGate.i.gateReady=false;lostGate.tick();
    assert(lostGate.c.state()==State::Failed && !lostGate.o.closeGate);
    Rig wrapRig;wrapRig.i.now=UINT32_MAX-300;wrapRig.start();wrapRig.arrive();wrapRig.spin();
    wrapRig.tick(620);assert(wrapRig.o.openGate);
    home::Policy movement;movement.movementOnly=true;
    Rig move;move.c=Controller(movement);move.i.gateReady=false;move.i.heldWeight=true;
    move.start();move.tick(30000);assert(move.c.state()==State::Return && move.o.navigate);
    move.i.x=0;move.i.onHome=true;move.i.remainingMs=1000;move.tick();move.tick(620);
    assert(move.c.state()==State::Arrived && !move.o.left && !move.o.right);
    assert(!move.o.openGate && !move.o.releaseHeld && !move.o.delivered && !move.o.resumeSearch);
    move.i.running=false;move.tick();assert(move.c.state()==State::Arrived);
    std::mt19937 rng(301);
    for(int j=0;j<100;++j) {
        Rig f;f.start();
        for(int k=0;k<1000;++k) {
            f.i.frontFresh=rng()%2;f.i.front=rng()%1000;f.i.rearFresh=rng()%2;f.i.rear=rng()%1000;
            f.i.onHome=rng()%2;f.i.x=int(rng()%2000)-1000;f.i.y=int(rng()%2000)-1000;
            f.i.heading=int(rng()%360)-180;f.tick();
            assert(!(f.o.left<0 && f.o.right<0));
            assert(std::abs(f.o.left)<=57 && std::abs(f.o.right)<=57);
            if(f.o.openGate) assert(f.i.onHome && std::hypot(f.i.x,f.i.y)<=250 && std::fabs(wrap(f.i.heading))<=10);
            if(!f.i.frontFresh) assert(!f.o.left && !f.o.right);
        }
    }
    std::cout<<"PASS homing regression scenarios plus 100,000 randomized control ticks\n";
}
