// Real navigation, round, crane, homing adapter/core, IMU, pose and drive.
// Only sensor I/O, gate and motor hardware are simulated. Each case is a
// separate process so firmware statics begin as they would after reset.
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "navigation.h"
#include "homing.h"
#include "round.h"
#include "collection.h"
#include "colour.h"
#include "imu.h"
#include "pose.h"
#include "drive.h"
#include "weight_detect.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>
unsigned long mockNow=100;int mockGo=0,mockPWM=0;
int realWeightCount=0,dummyCount=0;
bool metal=false;
uint16_t tofMM[5]={1000,1000,1000,1000,1000};
uint16_t &tofBL=tofMM[0],&tofBR=tofMM[1],&tofUpright=tofMM[2],&tofRear=tofMM[3],&tofTop=tofMM[4];
uint16_t irSideLMM=300,irSideRMM=300;
bool rearFresh=true,topFresh=true,frontFresh=true;
uint16_t frontMM=1500;int leftMM=-1,rightMM=-1;
bool tofOk(int n){return n==TOF_REAR?rearFresh:(n==TOF_TOP?topFresh:true);}
bool rearBlocked(){return tofRear && tofRear<REAR_STOP_MM;}
bool x8Fresh(){return frontFresh;}uint16_t x8FrontMM(){return frontMM;}
uint16_t x8LeftMM(){return leftMM<0?frontMM:leftMM;}uint16_t x8RightMM(){return rightMM<0?frontMM:rightMM;}
bool x8WallAhead(){return false;}
bool sideNearLeft(){return irSideLMM>0 && irSideLMM<SIDE_NEAR_MM;}
bool sideNearRight(){return irSideRMM>0 && irSideRMM<SIDE_NEAR_MM;}
bool inductiveMetalNow(){return metal;}
float travel=0;
float odomRawDistanceMM(){return travel;}bool odomStalled(){return false;}
ColourSurface surface=COLOUR_GREEN,home=COLOUR_UNKNOWN;
bool colourOk(){return true;}ColourSurface colourSurface(){return surface;}
ColourSurface colourHome(){return home;}void colourCaptureHome(){home=surface==COLOUR_FLOOR?COLOUR_UNKNOWN:surface;}
bool colourOnHomeBase(){return home!=COLOUR_UNKNOWN && surface==home;}
const char* colourName(ColourSurface){return "mock";}
bool gateReady=true;int gatePos=GATE_CLOSED_POS;int gateOpenCalls=0;
bool gate_found(){return gateReady;}int gate_readPos(){assert(!HOMING_MOVEMENT_ONLY);return gateReady?gatePos:-1;}
bool gate_isOpen(){return gatePos==GATE_OPEN_POS;}
void gate_close(){assert(!HOMING_MOVEMENT_ONLY);gatePos=GATE_CLOSED_POS;}
void gate_open(){assert(!HOMING_MOVEMENT_ONLY);gatePos=GATE_OPEN_POS;++gateOpenCalls;}
int motorL=0,motorR=0;
void motorForward(int p,int n){(n==1?motorL:motorR)=p;}
void motorBackward(int p,int n){(n==1?motorL:motorR)=-p;}
void motorStop(int n){(n==1?motorL:motorR)=0;}
void tick(unsigned long ms=20){
    mockNow+=ms;imuUpdate();poseUpdate();weightDetectUpdate();roundUpdate();
    if(roundOver()) {driveHardStop();if(collection_busy())collection_stop_motion();homingStop();}
    else collection_update();
    if(roundRunning()) navigationUpdate();else driveHardStop();
}
void put(unsigned r,int16_t v){Wire.regs[r]=v&255;Wire.regs[r+1]=uint16_t(v)>>8;}
void pickup(int expected){
    metal=true;auto began=mockNow;
    while(std::strcmp(modeName(),"PICKUP") && mockNow-began<2000)tick();
    assert(!std::strcmp(modeName(),"PICKUP"));began=mockNow;
    while(targetsOnBoard()<expected && mockNow-began<9000){
        if(mockNow-began>2800)metal=false;
        tick();
    }
    assert(targetsOnBoard()==expected && !collection_busy());
    assert(roundRunning());
}
struct NavSample {std::string mode;int left,right;};
std::vector<NavSample> avoidanceReplay(){
    std::vector<NavSample> samples;
    struct Case {int left,right,rear,sideL,sideR;bool fresh;};
    const Case cases[]={
        {220,310,0,204,176,true}, // recorded arena stop, now an ordinary escape
        {200,1500,0,300,300,true}, {1500,200,0,300,300,true},
        {NEAR_MM+20,1500,0,300,300,true},
        {1500,1500,0,100,300,true}, {1500,1500,0,300,100,true},
        {1500,1500,0,300,300,false}
    };
    for(const auto &c:cases){
        navigationInit();headingHoldReset();
        leftMM=c.left;rightMM=c.right;frontMM=std::min(c.left,c.right);
        tofRear=c.rear;irSideLMM=c.sideL;irSideRMM=c.sideR;frontFresh=c.fresh;
        for(int j=0;j<80;++j){
            tick();samples.push_back({modeName(),lastDriveLeftPct(),lastDriveRightPct()});
            assert(lastDriveLeftPct()!=0 || lastDriveRightPct()!=0);
        }
    }
    leftMM=rightMM=-1;frontMM=1500;tofRear=1000;irSideLMM=irSideRMM=300;frontFresh=true;
    navigationInit();headingHoldReset();
    return samples;
}
int main(int argc,char**argv){
    assert(argc==2);const std::string scenario=argv[1];
    if(scenario=="rear_unknown")tofRear=0;
    if(scenario=="gate_missing")gateReady=false;
    if(scenario=="no_home_colour")surface=COLOUR_FLOOR;
    Wire.regs[0]=0xA0;Wire.regs[0x3D]=8;
    const int startRaw=scenario=="delivery_start_210"?210:(scenario=="delivery_start_355"?355:30);
    put(0x1A,startRaw*16);put(0x18,0);put(0x1E,-20*16);
    assert(imuInit());collection_init();roundInit();
    mockNow+=20;roundUpdate();mockNow+=100;roundUpdate();
    mockGo=1;mockNow+=20;roundUpdate();mockNow+=100;roundUpdate();
    assert(roundJustStarted());mockGo=0;
    navigationInit();imuZero();poseReset();homingStartRound();surface=COLOUR_FLOOR;
    travel=1000;tick();assert(imuOk());
    if(scenario=="empty_home") {
        surface=home;
        for(int j=0;j<500;++j)tick();
        tick(180000);
        assert(roundRunning() && !roundWantsHome() && !homingNavigating() && gateOpenCalls==0);
        assert(!collection_busy() && targetsOnBoard()==0);
        std::cout<<"PASS own base with no load: no unload or return, still running past two minutes\n";return 0;
    }
    if(scenario.rfind("incidental_",0)==0) {
        const int load=scenario=="incidental_two"?2:1;
        pickup(1);while(!strcmp(modeName(),"REPOSITION"))tick();
        if(load==2){pickup(2);while(!strcmp(modeName(),"REPOSITION"))tick();}
        surface=COLOUR_BLUE;tick();assert(!roundWantsHome() && gateOpenCalls==0);
        surface=COLOUR_FLOOR;tick();assert(!roundWantsHome());
        if(scenario=="incidental_late")tick(180000);
        surface=home;
        for(int j=0;j<500 && targetsOnBoard()!=0;++j)tick();
        assert(gateOpenCalls==1 && targetsOnBoard()==0 && !collection_holding() && roundRunning());
        assert(poseDistHomeMM()>600); // colour confirmation, not proximity to estimated origin
        tick();surface=COLOUR_FLOOR;
        for(int j=0;j<40;++j)tick();
        assert(roundWantsWeights() && !homingNavigating());
        pickup(1);assert(targetsOnBoard()==1);
        std::cout<<"PASS incidental delivery and subsequent pickup: "<<scenario<<"\n";return 0;
    }
    if(scenario=="go_stop_held" || scenario=="go_stop_pickup") {
        if(scenario=="go_stop_held") {
            pickup(1);while(!strcmp(modeName(),"REPOSITION"))tick();
            pickup(2);while(!strcmp(modeName(),"REPOSITION"))tick();pickup(3);
            assert(collection_holding());
        } else {
            metal=true;while(strcmp(modeName(),"PICKUP"))tick();tick();tick();
            assert(collection_busy());
        }
        assert(mockPWM==255);
        mockGo=1;tick(20);tick(60);
        assert(roundOver() && !collection_busy() && mockPWM==255);
        const int stoppedAngle=collection_arm_angle();
        mockGo=0;for(int j=0;j<500;++j)tick();
        mockGo=1;tick(100);tick(100);
        assert(roundOver() && mockPWM==255 && collection_arm_angle()==stoppedAngle);
        assert(!lastDriveLeftPct() && !lastDriveRightPct());
        std::cout<<"PASS GO latches drive/crane stop and preserves energized magnet: "<<scenario<<"\n";return 0;
    }
    if(scenario=="collection_min_turn") {
        leftMM=1500;rightMM=220;tick();assert(!strcmp(modeName(),"TURN_L"));
        rightMM=1500;
        for(int j=0;j<21;++j)tick();
        assert(!strcmp(modeName(),"TURN_L"));
        for(int j=0;j<15;++j)tick();
        assert(!strcmp(modeName(),"FORWARD"));
        std::cout<<"PASS collection retains 700ms minimum turn\n";return 0;
    }
    if(scenario=="pickup_sample_gap"){
        metal=true;
        while(strcmp(modeName(),"PICKUP"))tick();
        tick();assert(collection_busy());
        while(collection_busy())tick();
        metal=false;tick(); // first clear sample, start confirmation
        tick(PICKUP_CLEAR_CONFIRM_MS+20); // unobserved interval cannot prove clear
        assert(targetsOnBoard()==0 && !strcmp(modeName(),"PICKUP"));
        for(int j=0;j<30 && targetsOnBoard()==0;++j)tick();
        assert(targetsOnBoard()==1 && pickupAttempts==1);
        std::cout<<"PASS pickup clear confirmation resets across a sampling gap\n";
        return 0;
    }
    if(scenario.rfind("pickup_",0)==0){
        const bool heldThird=scenario=="pickup_third_retry";
        if(heldThird){pickup(1);while(!strcmp(modeName(),"REPOSITION"))tick();pickup(2);while(!strcmp(modeName(),"REPOSITION"))tick();}
        const int startingCount=targetsOnBoard(),startingAttempts=pickupAttempts;
        const bool late=scenario=="pickup_late_miss";
        if(late)while(roundElapsedMs()<RETURN_HOME_AT_MS-1500)tick();
        const bool allMiss=scenario=="pickup_all_miss" || scenario=="pickup_flicker" || late;
        const int misses=(scenario=="pickup_second_success")?1:((scenario=="pickup_first_success")?0:2);
        metal=true;auto began=mockNow;
        while(strcmp(modeName(),"PICKUP") && mockNow-began<1000)tick();
        assert(!strcmp(modeName(),"PICKUP"));
        bool sawVerify=false;
        began=mockNow;
        while(!strcmp(modeName(),"PICKUP") && mockNow-began<30000){
            const int completed=pickupAttempts-startingAttempts;
            metal=allMiss || completed<misses;
            // One clear sample per 200ms while the crane is finished must
            // NEVER count. This also forces repeated clear-timer resets.
            if(scenario=="pickup_flicker" && !collection_busy() && ((mockNow/20)%10)==0)metal=false;
            tick();
            if(Serial.printed.find("PICKUP_VERIFY")!=std::string::npos)sawVerify=true;
            if(!strcmp(modeName(),"PICKUP")){
                assert(!lastDriveLeftPct() && !lastDriveRightPct());
                assert(targetsOnBoard()==startingCount);
            }
        }
        assert(sawVerify && strcmp(modeName(),"PICKUP"));
        const int expectedTries=allMiss?3:misses+1;
        assert(pickupAttempts-startingAttempts==expectedTries);
        assert(targetsOnBoard()==startingCount+(allMiss?0:1));
        if(allMiss && !late){
            metal=true;
            for(int j=0;j<150;++j)tick();
            assert(pickupAttempts-startingAttempts==3 && targetsOnBoard()==startingCount);
        }
        if(heldThird){tick();tick();assert(homingNavigating());}
        if(late){tick();tick();assert(!homingNavigating() && targetsOnBoard()==0);}
        if(heldThird)assert(collection_holding() && mockPWM==255);
        std::cout<<"PASS actual pickup verification/retries: "<<scenario<<" attempts="<<expectedTries<<" count="<<targetsOnBoard()<<"\n";
        return 0;
    }
    std::vector<NavSample> reference;
    if(scenario=="nav_parity") {
        reference=avoidanceReplay();
        uint32_t hash=2166136261u;
        for(const auto &s:reference) {
            for(unsigned char ch:s.mode) hash=(hash^ch)*16777619u;
            hash=(hash^uint32_t(s.left))*16777619u;
            hash=(hash^uint32_t(s.right))*16777619u;
        }
        assert(hash==523164021u); // captured from V8 BEFORE homing-only changes
    }
    if(scenario=="physical_left" || scenario=="physical_right" || scenario=="close_left" || scenario=="close_right"){
        const bool physicalLeft=scenario=="physical_left" || scenario=="close_left";
        const bool close=scenario.rfind("close_",0)==0;
        (physicalLeft?tofBL:tofBR)=close?240:350;
        for(int j=0;j<20;++j)tick();
        assert(!strcmp(modeName(),"APPROACH") && !weightCentreActive);
        const int sign=physicalLeft?1:-1;
        assert(weightSide==sign);
        const int speed=close?CREEP_SPEED_PCT:ONE_SIDE_APPROACH_SPEED_PCT;
        assert(lastDriveLeftPct()==speed+sign*ONE_SIDE_ARC_PCT);
        assert(lastDriveRightPct()==speed-sign*ONE_SIDE_ARC_PCT);
        assert(lastDriveLeftPct()>0 && lastDriveRightPct()>0);
        std::cout<<"PASS crossed 60-degree beam -> slow opposite-side arc: "<<scenario<<"\n";
        return 0;
    }
    if(scenario=="top_weight"){
        tofTop=400;
        for(int j=0;j<20;++j)tick();
        assert(weightCentreActive && weightSide==0);
        assert(!strcmp(modeName(),"APPROACH"));
        assert(lastDriveLeftPct()==APPROACH_SPEED_PCT && lastDriveRightPct()==APPROACH_SPEED_PCT);
        tofTop=240;tick();assert(!strcmp(modeName(),"CREEP"));
        for(int j=0;j<70;++j)tick(); // longer than ordinary blind-gap creep
        assert(!strcmp(modeName(),"CREEP"));assert(!collection_busy());
        pickup(1);assert(!collection_holding());
        std::cout<<"PASS integrated top-only approach, straight steering, sustained creep and inductive pickup\n";
        return 0;
    }
    if(scenario=="late_pickup"){
        // Start just before the timed trigger; it must finish this crane cycle.
        while(roundElapsedMs()<RETURN_HOME_AT_MS-1500)tick();
        pickup(1);assert(roundWantsHome());
    }else{
        pickup(1);while(!strcmp(modeName(),"REPOSITION"))tick();
        pickup(2);while(!strcmp(modeName(),"REPOSITION"))tick();
        pickup(3);assert(collection_holding() && mockPWM==255);
    }
    assert(roundWantsHome());
    if(scenario=="handoff_trace"){
        assert(targetsOnBoard()==3 && !collection_busy() && collection_holding());
        assert(!strcmp(modeName(),"FORWARD"));
        assert(!lastDriveLeftPct() && !lastDriveRightPct());
        std::cout<<"A: third counted; crane idle; magnet=255; mode=FORWARD; return requested; motors stopped\n";
        tick();homingPrintStatus();assert(Serial.printed.find("HOME state=RETURN")!=std::string::npos);
        assert(!lastDriveLeftPct() && !lastDriveRightPct());
        std::cout<<"B: next tick owns drive in RETURN; intentional one-tick neutral handoff\n";
        tick();assert(lastDriveLeftPct()!=0 || lastDriveRightPct()!=0);
        assert(motorL!=0 || motorR!=0);
        std::cout<<"C: following tick commands motors "<<lastDriveLeftPct()<<","<<lastDriveRightPct()
                 <<"; round still RUNNING; third weight held\nPASS isolated test 1: third-pickup handoff\n";
        return 0;
    }
    if(scenario.rfind("block_",0)==0){
        const std::string fault=scenario.substr(6);
        const char* expected="";bool blocked=true,recoverable=true;
        if(fault=="front_stale"){frontFresh=false;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="front_close"){frontMM=100;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="rear_stale"){rearFresh=false;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="rear_close"){tofRear=100;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="left_unknown"){irSideLMM=0;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="right_close"){irSideRMM=100;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="rear_unknown_front_186"){tofRear=0;frontMM=186;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="rear_unknown_clear"){tofRear=0;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="gate_top_missing"){gateReady=false;topFresh=false;blocked=false;expected="round navigation toward home waypoint";}
        else if(fault=="imu_loss_stationary"){
            Wire.fail=true;mockNow+=200;imuUpdate();expected="return paused: waiting for fresh IMU";
        }
        else if(fault=="imu_loss_moving"){
            tick();tick();assert(lastDriveLeftPct()!=0 || lastDriveRightPct()!=0);
            Wire.fail=true;mockNow+=200;imuUpdate();expected="position invalid; see poseLoss";recoverable=false;
        }
        else if(fault=="imu_reset"){
            Wire.regs[0x3D]=0;imuUpdate();expected="position invalid; see poseLoss";recoverable=false;
        }
        else {assert(false);}
        tick();tick();tick();
        Serial.printed.clear();homingPrintStatus();
        assert(Serial.printed.find(expected)!=std::string::npos);
        if(blocked)assert(!lastDriveLeftPct() && !lastDriveRightPct());
        else assert(lastDriveLeftPct()!=0 || lastDriveRightPct()!=0);
        std::cout<<fault<<": "<<(blocked?"STOP":"MOVE")<<" | "<<expected;
        frontFresh=rearFresh=topFresh=true;frontMM=1500;tofRear=1000;
        irSideLMM=irSideRMM=300;Wire.fail=false;
        // A new heading clears any obstacle avoidance latch after its 55deg turn.
        put(0x1A,120*16);
        for(int j=0;j<10;++j)tick();
        if(recoverable)assert(lastDriveLeftPct()!=0 || lastDriveRightPct()!=0);
        else assert(!lastDriveLeftPct() && !lastDriveRightPct());
        assert(targetsOnBoard()==3 && collection_holding() && mockPWM==255);
        std::cout<<" | after restoring inputs: "<<(recoverable?"MOVE":"LATCHED STOP")<<"\n";
        return 0;
    }
    tick();tick();tick();
    assert(lastDriveLeftPct()!=0 || lastDriveRightPct()!=0);
    assert(motorL!=0 || motorR!=0); // real drive mapping has reached motors
    assert(!collection_busy());assert(roundRunning());
    if(scenario=="home_min_turn" || scenario=="home_repetition") {
        auto obstacleTurn=[&](bool rightObstacle) {
            leftMM=rightObstacle?1500:220;rightMM=rightObstacle?220:1500;
            tick();assert(!strcmp(modeName(),"TURN_L"));
            leftMM=rightMM=1500;
            for(int j=0;j<20;++j)tick(); // 400ms: must not finish earlier
            assert(!strcmp(modeName(),"TURN_L"));
            tick();assert(!strcmp(modeName(),"FORWARD")); // first tick after minimum
            tick(); // observe the completed avoidance episode
        };
        obstacleTurn(true);
        if(scenario=="home_min_turn") {
            std::cout<<"PASS homing minimum turn 400ms\n";return 0;
        }
        obstacleTurn(false); // still commits left on the second episode
        // Opposite side obstructed: detect repetition but do not redirect there.
        rightMM=220;leftMM=1500;tick();assert(!strcmp(modeName(),"TURN_L"));
        rightMM=1500;for(int j=0;j<23;++j)tick();
        // Now right is open: recovery must override the old left commitment.
        leftMM=220;rightMM=1500;tick();assert(!strcmp(modeName(),"TURN_R"));
        assert(Serial.printed.find("HOME_REPEAT recovery direction=right")!=std::string::npos);
        assert(lastDriveLeftPct()>0 && lastDriveRightPct()<0);
        leftMM=1500;for(int j=0;j<23;++j)tick();
        // No displacement: cannot immediately switch back, even if requested.
        rightMM=220;tick();assert(!strcmp(modeName(),"TURN_R"));
        assert(collection_holding() && targetsOnBoard()==3 && !collection_busy());
        std::cout<<"PASS homing repetition, clearance veto, alternate direction, no rapid reversal\n";return 0;
    }
    if(scenario=="home_detour") {
        rightMM=220;leftMM=1500;
        for(int j=0;j<40;++j)tick();
        assert(!strcmp(modeName(),"TURN_L"));
        put(0x1A,355*16); // valid unsigned IMU yaw: -35 relative to raw 30 at start
        rightMM=leftMM=1500;
        for(int j=0;j<40;++j)tick();
        assert(!strcmp(modeName(),"FORWARD"));
        assert(lastDriveLeftPct()>0 && lastDriveRightPct()>0);
        // Direct home is behind/right; without commitment this pivoted back.
        for(int j=0;j<80;++j)tick(); // time alone is not progress
        assert(lastDriveLeftPct()>0 && lastDriveRightPct()>0);
        travel+=300;tick();assert(lastDriveLeftPct()>0 && lastDriveRightPct()>0);
        travel+=60;tick();
        assert(lastDriveLeftPct()*lastDriveRightPct()<0); // now rejoin home
        assert(Serial.printed.find("HOME_DETOUR pass heading=")!=std::string::npos);
        assert(collection_holding() && targetsOnBoard()==3 && !collection_busy());
        std::cout<<"PASS recorded right-obstacle turn conflict: forward pass before home pivot\n";
        return 0;
    }
    if(scenario=="late_pickup"){
        metal=true;tofBL=tofBR=tofTop=180;
        for(int j=0;j<100;++j)tick();
        assert(homingNavigating() && !collection_busy() && targetsOnBoard()==1);
        assert(strcmp(modeName(),"PICKUP") && strcmp(modeName(),"APPROACH") && strcmp(modeName(),"CREEP"));
    }
    if(scenario=="rear_unknown")assert(lastDriveLeftPct()>0 && lastDriveRightPct()<0);
    if(scenario=="nav_parity"){
        const auto returning=avoidanceReplay();assert(returning.size()==reference.size());
        for(unsigned j=0;j<reference.size();++j){
            assert(returning[j].mode==reference[j].mode);
            assert(returning[j].left==reference[j].left && returning[j].right==reference[j].right);
        }
        // Once clear, only the desired bearing changes; weights cannot divert return.
        metal=true;tofBL=tofBR=tofTop=180;
        put(0x1A,210*16);tick(); // facing home at +180
        assert(lastDriveLeftPct()>0 && lastDriveRightPct()>0);
        put(0x1A,180*16);tick(); // home is 30 degrees right
        assert(lastDriveLeftPct()>lastDriveRightPct());
        put(0x1A,240*16);tick(); // home is 30 degrees left
        assert(lastDriveLeftPct()<lastDriveRightPct());
        assert(!collection_busy() && collection_holding() && mockPWM==255 && targetsOnBoard()==3);
        std::cout<<"PASS 560-tick parity replay: collection and homing obstacle avoidance, rear zero, side nudges, stale front\n";
    }
    if(scenario.rfind("delivery_start_",0)==0){
        // No return timeout; with fixed sensor pose, navigation continues past 25s.
        for(int j=0;j<1600;++j)tick();
        assert(lastDriveLeftPct()!=0 || lastDriveRightPct()!=0);
        assert(roundRunning() && collection_holding() && targetsOnBoard()==3);
        travel=339;tick(); // first recorded home contact: used to reject >250 mm
        put(0x1A,((startRaw+50)%360)*16);tick();surface=home;
        for(int j=0;j<40;++j)tick();
        Serial.printed.clear();homingPrintStatus();
        assert(Serial.printed.find("HOME state=ALIGN_START")!=std::string::npos);
        assert(gateOpenCalls==0 && collection_holding());
        // Align to the captured GO heading even when raw yaw crosses 360/0.
        put(0x1A,((startRaw+15)%360)*16);tick(); // still outside tolerance
        assert(gateOpenCalls==0 && !collection_busy());
        put(0x1A,((startRaw+5)%360)*16);tick(); // within starting-heading tolerance
        assert(!lastDriveLeftPct() && !lastDriveRightPct());
        // Gate must open while the third weight remains on the magnet.
        for(int j=0;j<60 && gateOpenCalls==0;++j)tick();
        assert(gateOpenCalls==1 && collection_holding() && mockPWM==255 && !collection_busy());
        for(int j=0;j<100;++j) {tick();assert(collection_holding() && !collection_busy());}
        for(int j=0;j<600 && targetsOnBoard()==3;++j)tick();
        assert(gateOpenCalls==1 && !collection_holding() && mockPWM==0);
        assert(targetsOnBoard()==0 && roundRunning());
        put(0x1A,startRaw*16);tick();surface=COLOUR_FLOOR;
        for(int j=0;j<40;++j)tick();
        assert(!homingNavigating() && roundWantsWeights());
        Serial.printed.clear();homingPrintStatus();
        assert(Serial.printed.find("HOME state=COLLECT")!=std::string::npos);
        pickup(1);assert(targetsOnBoard()==1 && !collection_holding());
        std::cout<<"PASS full delivery: captured GO yaw="<<startRaw<<", heading alignment, colour, gate FIRST, third release, next-trip pickup\n";
    }
    std::cout<<"PASS integrated pickup -> homing -> motors: "<<scenario<<"\n";
}
