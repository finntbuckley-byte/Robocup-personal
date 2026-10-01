#include "home_core.h"
#include <cmath>
namespace home {
float wrap(float a) {while(a>180)a-=360;while(a<-180)a+=360;return a;}
const char* name(State s) {
    const char* n[]={"COLLECT","RETURN","SEEK_COLOUR","CONFIRM_HOME","ADVANCE_HOME","ALIGN_START",
        "RECHECK_HOME","TRANSFER_HELD","OPEN_GATE","UNLOAD","CLOSE_GATE","EXIT_TURN","EXIT_BASE","HOME_STOP","HOME_ARRIVED"};
    return n[static_cast<unsigned>(s)];
}
void Controller::enter(State s,const Input& i,const char* why) {
    state_=s;entered_=i.now;confirming_=false;reason_=why;
}
const char* Controller::pivotBlocker(const Input& i) const {
    if(!i.frontFresh) return "turn blocked: front stale";
    // Use the proven navigation's in-place pivot policy: one track forward,
    // one backward. Nearby walls trigger pivots there; requiring all four
    // sides clear here deadlocks at a corner base. Its rear guard applies
    // to BOTH tracks reversing, which this controller never commands.
    return nullptr;
}
Output Controller::fail(const Input& i,const char* why) {
    Output o;o.ownsDrive=true;
    o.closeGate=state_==State::Open || state_==State::Unload || state_==State::Close;
    enter(State::Failed,i,why);return o;
}
Output Controller::tick(const Input& i) {
    Output o;
    if(!i.running) {
        o.ownsDrive=true;
        o.closeGate=state_==State::Open || state_==State::Unload || state_==State::Close;
        if(state_!=State::Idle && state_!=State::Failed && state_!=State::Arrived)
            enter(State::Failed,i,"round stopped; retain load");
        return o;
    }
    if(state_==State::Idle) {
        if(!i.request) {reason_="collecting";return o;}
        if(i.pickupBusy) {reason_="return requested; finishing pickup";return o;}
        seekIndex_=0;
        enter(State::Return,i,"return requested");
        o.ownsDrive=true;o.beginReturn=true;
        return o;
    }
    o.ownsDrive=true;
    if(state_==State::Failed || state_==State::Arrived) return o;
    if(!i.poseValid) return fail(i,"position invalid; see poseLoss");
    if(!std::isfinite(i.x) || !std::isfinite(i.y) || !std::isfinite(i.heading))
        return fail(i,"non-finite position or heading");
    if(!i.imuFresh) {
        // With position still trusted, a dropout while stationary at the
        // pickup/return handoff is recoverable. Own the motors at zero until
        // heading returns. The adapter separately revokes pose trust on a
        // real reset or a dropout while commanded to move.
        if(state_==State::Return || state_==State::SeekColour || state_==State::Confirm) {
            confirming_=false;
            reason_="return paused: waiting for fresh IMU";
            return o;
        }
        return fail(i,"IMU unavailable");
    }
    // A transient gate disconnect must not latch HOME_STOP or prevent the
    // arrival pivot. Wait before transferring/releasing, then retry the gate
    // command after its driver reconnects (the driver closes on recovery).
    // Once the crane is releasing through the open gate, losing gate feedback
    // must stop that motion rather than letting it finish into a closed bin.
    if(state_==State::Transfer && !i.gateReady)
        return fail(i,"gate disconnected during held release");
    const bool needsGate=state_==State::Reconfirm ||
        state_==State::Open || state_==State::Unload || state_==State::Close;
    if(!p_.movementOnly && needsGate && !i.gateReady) {
        waitingGate_=true;confirming_=false;
        reason_="delivery paused: waiting for gate reconnect";
        return o;
    }
    if(waitingGate_ && i.gateReady) {
        waitingGate_=false;entered_=i.now;confirming_=false;
        if(state_==State::Open || state_==State::Unload) {
            // Require current home confirmation before reopening after loss.
            if(!(i.homeKnown && i.onHome &&
                 std::fabs(wrap(-i.heading))<=p_.turnTolerance))
                return fail(i,"home or starting heading lost during gate reconnect");
            enter(State::Open,i,"gate recovered; retry open");o.openGate=true;return o;
        }
        if(state_==State::Close) {o.closeGate=true;reason_="gate recovered; retry close";return o;}
    }
    const uint32_t held=i.now-entered_;
    const float dist=std::hypot(i.x,i.y);
    const bool arrived=i.homeKnown && i.onHome;
    // imuZero() captures the actual orientation at GO every round. Zero here
    // means that stored orientation, not a fixed compass/arena direction.
    const float headingError=wrap(-i.heading);
    const bool aligned=std::fabs(headingError)<=p_.turnTolerance;
    auto turn=[&](int sign) {o.left=sign*p_.turn;o.right=-o.left;return o;};
    auto guardedTurn=[&](int sign) {
        if(const char* why=pivotBlocker(i)) {
            reason_=why;return o;
        }
        reason_="turning with clearance";return turn(sign);
    };
    auto driveTo=[&](float x,float y) {
        reason_="round navigation toward home waypoint";
        o.navigate=true;
        o.targetHeading=std::atan2(y-i.y,x-i.x)*57.2957795f;
        return o;
    };
    if(state_==State::Return || state_==State::SeekColour || state_==State::Confirm) {
        if(!i.homeKnown && dist<100) return fail(i,"at estimated home; start colour was not captured");
        if(arrived) {
            if(state_!=State::Confirm) enter(State::Confirm,i,"own home colour detected; confirm before delivery");
            if(!confirming_) {confirming_=true;confirmAt_=i.now;}
            if(i.now-confirmAt_>=p_.confirmMs) {
                if(p_.movementOnly) {
                    enter(State::Arrived,i,"home confirmed; movement test complete; retain load");
                    return o;
                }
                const uint32_t budget=15000+(i.heldWeight?4000:0);
                if(i.remainingMs<=budget) return fail(i,"not enough time for delivery; retain load");
                entryX_=i.x;entryY_=i.y;entryHeading_=i.heading;entryProgress_=0;
                enter(State::Advance,i,"advance onto home before restoring starting heading");
            }
            return o;
        }
        if(state_==State::Confirm) enter(State::Return,i,"home confirmation interrupted");
        if(state_==State::Return && dist<100) enter(State::SeekColour,i,"search close to estimated home");
        if(state_==State::SeekColour) {
            const float xy[4][2]={{180,0},{0,180},{-180,0},{0,-180}};
            if(std::hypot(xy[seekIndex_][0]-i.x,xy[seekIndex_][1]-i.y)<60) seekIndex_=(seekIndex_+1)%4;
            return driveTo(xy[seekIndex_][0],xy[seekIndex_][1]);
        }
        return driveTo(0,0);
    }
    if(state_==State::Advance) {
        // Signed progress along the arrival heading: turning or sideways drift
        // must not be mistaken for the requested forward travel.
        const float rad=entryHeading_*0.01745329252f;
        entryProgress_=(i.x-entryX_)*std::cos(rad)+(i.y-entryY_)*std::sin(rad);
        auto beginAlignment=[&](const char* why) {
            spinProgress_=0;lastHeading_=i.heading;
            turnActiveMs_=0;turnTickAt_=i.now;turnCommanded_=false;
            enter(State::Align,i,why);
            return o; // stop before pivoting on the next tick
        };
        if(entryProgress_>=p_.entryDistanceMm)
            return beginAlignment("home advance complete; restore heading recorded at GO");
        if(!i.frontFresh) {
            reason_="home advance paused: front stale";return o;
        }
        if((i.front && i.front<p_.frontStopMm) ||
           (i.left && i.left<p_.frontStopMm) || (i.right && i.right<p_.frontStopMm) ||
           (i.sideL && i.sideL<p_.sideClearMm) || (i.sideR && i.sideR<p_.sideClearMm))
            return beginAlignment("home advance shortened by obstacle; align before rechecking colour");
        const float error=wrap(entryHeading_-i.heading);
        const float correction=error*p_.entryHeadingKp;
        const int steer=static_cast<int>(std::fmax(-p_.entryMaxSteer,
                                                  std::fmin(p_.entryMaxSteer,correction)));
        reason_="advancing onto home along arrival heading";
        o.left=p_.entrySpeed+steer;o.right=p_.entrySpeed-steer;
        return o;
    }
    if(state_==State::Align) {
        if(turnCommanded_) turnActiveMs_+=i.now-turnTickAt_;
        turnTickAt_=i.now;turnCommanded_=false;
        spinProgress_+=std::fabs(wrap(i.heading-lastHeading_));lastHeading_=i.heading;
        if(aligned) {
            enter(State::Reconfirm,i,"starting heading reached; confirm heading and home at rest");
            confirming_=arrived;confirmAt_=i.now;return o;
        }
        if(turnActiveMs_>=p_.turnTimeoutMs) return fail(i,"starting heading alignment timed out while motors commanded");
        if(const char* why=pivotBlocker(i)) {reason_=why;return o;}
        reason_="aligning to heading recorded at GO";turnCommanded_=true;
        return turn(headingError>0?1:-1);
    }
    if(state_==State::Reconfirm) {
        // Hold still through colour dropout. Both home and the starting heading
        // must stay confirmed for the full dwell before opening the gate.
        if(!arrived) {
            confirming_=false;reason_="waiting for home colour after alignment; retain load";
            return o;
        }
        if(!aligned) {
            turnTickAt_=i.now;turnCommanded_=false;lastHeading_=i.heading;
            enter(State::Align,i,"heading drifted while settling; realign before opening");
            return o;
        }
        if(!confirming_) {confirming_=true;confirmAt_=i.now;}
        if(i.now-confirmAt_>=p_.confirmMs) {
            enter(State::Open,i,"open gate before releasing stored weights");o.openGate=true;
        }
        return o;
    }
    if(state_==State::Transfer) {
        if(!arrived) return fail(i,"home lost during transfer");
        if(!aligned) return fail(i,"starting heading lost during transfer");
        if(!i.gateOpen) return fail(i,"gate no longer open during held release");
        if(i.craneBusy) craneSeen_=true;
        if(held>=p_.transferTimeoutMs) return fail(i,"held transfer timed out");
        if(craneSeen_ && !i.craneBusy && !i.heldWeight)
            enter(State::Unload,i,"third weight released; keep gate open to clear");
        return o;
    }
    if(state_==State::Open || state_==State::Unload) {
        if(!arrived) return fail(i,"home lost during unloading");
        if(!aligned) return fail(i,"starting heading lost during unloading");
        if(state_==State::Open) {
            if(i.gateOpen) enter(State::Unload,i,"gate open; allow stored weights to clear");
            else if(held>=p_.gateTimeoutMs) return fail(i,"gate did not reach open position");
        } else {
            if(!i.gateOpen) return fail(i,"gate position lost during unloading");
            if(held>=p_.unloadMs) {
                if(i.heldWeight) {
                    craneSeen_=false;enter(State::Transfer,i,"stored weights given time to clear; release third through open gate");
                    o.releaseHeld=true;
                } else {enter(State::Close,i,"close before searching");o.closeGate=true;}
            }
        }
        return o;
    }
    if(state_==State::Close) {
        if(i.gateClosed) {
            // No exit beam: this is an ASSUMED delivery following the timed
            // unloading sequence. Field testing must prove the bin empties.
            o.delivered=true;enter(State::ExitTurn,i,"delivery attempted; face starting direction");
        } else if(held>=p_.gateTimeoutMs) return fail(i,"gate did not close; retain count");
        return o;
    }
    if(state_==State::ExitTurn) {
        if(held>=p_.turnTimeoutMs) return fail(i,"exit alignment timed out");
        const float err=wrap(-i.heading);
        if(std::fabs(err)<=8) {enter(State::Exit,i,"leave base before collecting again");return o;}
        return guardedTurn(err>0?1:-1);
    }
    if(state_==State::Exit) {
        if(held>=p_.exitTimeoutMs) return fail(i,"base exit timed out");
        if(i.onFloor) {
            if(!confirming_) {confirming_=true;confirmAt_=i.now;}
            if(i.now-confirmAt_>=p_.confirmMs) {
                enter(State::Idle,i,"floor confirmed; resume searching");o.resumeSearch=true;return o;
            }
        } else confirming_=false;
        if(!i.frontFresh || (i.front && i.front<p_.frontStopMm) ||
           (i.sideL && i.sideL<p_.sideClearMm) || (i.sideR && i.sideR<p_.sideClearMm)) return o;
        o.left=o.right=p_.speed;return o;
    }
    return o;
}
}
