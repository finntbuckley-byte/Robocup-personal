#include <Arduino.h>
#include "config.h"
#include "home_core.h"
#include "homing.h"
#include "colour.h"
#include "round.h"
#include "pose.h"
#include "imu.h"
#include "drive.h"
#include "tof.h"
#include "x8.h"
#include "ir_sensors.h"
#include "collection.h"
#include "gate.h"
#include "navigation.h"
#include "weight_detect.h"
static home::Policy makeHomePolicy() {
    home::Policy p;
    p.movementOnly=HOMING_MOVEMENT_ONLY;
    p.speed=HOME_EXIT_SPEED;p.turn=HOME_DELIVERY_TURN_SPEED;
    p.turnTolerance=HOME_DELIVERY_HEADING_TOLERANCE_DEG;
    p.entryDistanceMm=HOME_ENTRY_DISTANCE_MM;p.entrySpeed=HOME_ENTRY_SPEED;
    p.entryHeadingKp=HOME_ENTRY_HEADING_KP;p.entryMaxSteer=HOME_ENTRY_MAX_STEER;
    return p;
}
static home::Controller controller(makeHomePolicy());
static bool navigateHome=false;
static float targetHeading=0;
bool homingNavigating() {return navigateHome;}
float homingTargetHeading() {return targetHeading;}
static bool trusted=false;
static const char* poseLoss="not started";
static int resetBaseline=0;
static int gatePosition=-1;
static uint32_t gateAt=0,telemetryAt=0;
static home::State printed=home::State::Idle;
void homingStartRound() {
    controller.reset();navigateHome=false;colourCaptureHome();
    trusted=imuOk();resetBaseline=imuResetCount();poseLoss=trusted?"none":"IMU invalid at GO";
    gatePosition=-1;if(!HOMING_MOVEMENT_ONLY) gate_close();
    Serial.println("EXPERIMENTAL_HOMING_V13_HOME_ADVANCE: advance 150mm on home before alignment; unlimited until GO stop");
}
void homingStop() {
    navigateHome=false;
    home::Input i;i.now=millis();
    auto o=controller.tick(i);
    if(!HOMING_MOVEMENT_ONLY && (o.closeGate || gate_isOpen())) gate_close();
    if(collection_busy()) collection_stop_motion();
    driveSetStartupBoost(false);driveHardStop();
}
bool homingUpdate(bool pickupInProgress) {
    navigateHome=false;
    if(trusted && imuResetCount()!=resetBaseline) {trusted=false;poseLoss="IMU reset since GO";}
    if(trusted && !imuOk() && (lastDriveLeftPct()!=0 || lastDriveRightPct()!=0)) {
        trusted=false;poseLoss="invalid IMU while drive commanded";
    }
    if(!HOMING_MOVEMENT_ONLY && !lastDriveLeftPct() && !lastDriveRightPct() && millis()-gateAt>=200) {
        gateAt=millis();gatePosition=gate_readPos();
    }
    home::Input i;
    i.now=millis();i.running=roundRunning();i.request=roundWantsHome();
    i.pickupBusy=pickupInProgress || collection_busy();
    const uint32_t elapsed=roundElapsedMs(),end=ROUND_MS-ROUND_END_MARGIN_MS;
    i.remainingMs=ROUND_TIME_LIMIT_ENABLED ? (elapsed<end?end-elapsed:0) : UINT32_MAX;
    i.poseValid=trusted;
    i.homeKnown=colourHome()==COLOUR_GREEN || colourHome()==COLOUR_BLUE;
    i.imuFresh=imuOk();i.x=poseXmm();i.y=poseYmm();i.heading=imuHeadingDeg();
    i.onHome=colourOnHomeBase();i.onFloor=colourOk() && colourSurface()==COLOUR_FLOOR;
    i.frontFresh=x8Fresh();i.front=x8FrontMM();i.left=x8LeftMM();i.right=x8RightMM();
    i.rear=tofRear;i.rearFresh=tofOk(TOF_REAR);i.sideL=irSideLMM;i.sideR=irSideRMM;
    i.heldWeight=collection_holding();i.craneBusy=collection_busy();i.gateReady=gate_found();
    const bool posFresh=i.now-gateAt<500 && gatePosition>=0;
    i.gateOpen=posFresh && abs(gatePosition-GATE_OPEN_POS)<=GATE_POSITION_TOLERANCE;
    i.gateClosed=posFresh && abs(gatePosition-GATE_CLOSED_POS)<=GATE_POSITION_TOLERANCE;
    auto o=controller.tick(i);
    if(controller.state()==home::State::Failed && collection_busy()) collection_stop_motion();
    if(o.beginReturn) {driveHardStop();navigationInit();roundAcknowledgeReturn();}
    if(o.releaseHeld) collection_release_held();
    if(!HOMING_MOVEMENT_ONLY && o.openGate) {gatePosition=-1;gate_open();}
    if(!HOMING_MOVEMENT_ONLY && o.closeGate) {gatePosition=-1;gate_close();}
    if(o.delivered) noteDelivered(targetsOnBoard());
    if(o.resumeSearch) {navigationInit();suppressTargetFor(1500);}
    navigateHome=o.navigate;targetHeading=o.targetHeading;
    if(o.ownsDrive) {
        driveSetStartupBoost(false);
        if(!navigateHome) {
            if(!o.left && !o.right) driveHardStop();else drive(o.left,o.right);
        }
    }
    if(printed!=controller.state()) {
        printed=controller.state();Serial.print("HOME_STATE ");Serial.print(home::name(printed));
        Serial.print(" : ");Serial.println(controller.reason());
    }
    return o.ownsDrive && !navigateHome;
}
void homingTelemetry() {
    if(millis()-telemetryAt<1000) return;
    telemetryAt=millis();
    homingPrintStatus();
}
void homingPrintStatus() {
    Serial.printf("FW=HOMING_V13_HOME_ADVANCE HOME state=%s colour=%s home=%s poseOK=%d imuOK=%d distance=%.0f heading=%.1f gate=%d held=%d load=%d\n",
        home::name(controller.state()),colourName(colourSurface()),colourName(colourHome()),trusted,
        imuOk(),poseDistHomeMM(),imuHeadingDeg(),gatePosition,collection_holding(),targetsOnBoard());
    Serial.printf("HOME_REASON=%s poseLoss=%s gateReady=%d frontFresh=%d rear=%u rearFresh=%d sides=%u,%u round=%s elapsed=%lu\n",
        controller.reason(),poseLoss,gate_found(),x8Fresh(),tofRear,tofOk(TOF_REAR),
        irSideLMM,irSideRMM,roundPhaseName(),roundElapsedMs());
    Serial.printf("HOME_ENTRY request=%d pickupBusy=%d nav=%s drive=%d,%d imuResets=%d TOP_WEIGHT=1 centreTarget=%d timed=%d\n",
        roundWantsHome(),collection_busy(),modeName(),lastDriveLeftPct(),lastDriveRightPct(),imuResetCount(),weightCentreActive,
        ROUND_TIME_LIMIT_ENABLED);
    Serial.printf("HOME_ARRIVAL onHome=%d distance=%.0f front=%u turnProgress=%.1f gateID=%d startHeadingError=%.1f\n",
        colourOnHomeBase(),poseDistHomeMM(),x8FrontMM(),controller.turnProgress(),GATE_ID,
        home::wrap(-imuHeadingDeg()));
    Serial.printf("HOME_ADVANCE progress=%.0f target=%.0f\n",controller.entryProgress(),HOME_ENTRY_DISTANCE_MM);
}
