#pragma once
// ============================================================================
//  config.h  -  full RoboCup navigation + collection build. ALL pins/tuning
//  here.
//
//  HARDWARE:
//    Drive:  2x DC motor via DFR0513 PPM driver (motor.cpp/h, PARTNER'S CODE
//            - unchanged). Motor 1 = LEFT (pin 0), Motor 2 = RIGHT (pin 1).
//    Pickup: swing-arm crane + 2 electromagnets (collection.cpp/h, PARTNER'S
//            CODE - unchanged, just exposes collection_busy()).
//    Sensing (THIS build):
//      7x ToF via SX1509 XSHUT lines:
//        2x low, short range (VL53L0X)  - weight detection, under the robot
//        4x top, long range  (VL53L1X)  - front + angled corner obstacle
//        1x rear, long range (VL53L1X)  - reversing clearance
//      3x analog IR (GP2Y-style):
//        2x side, short range - wall/scrape nudge
//        1x inside the robot at the end of the funnel - weight presence
//      1x inductive proximity sensor at the end of the funnel - metal
//        (real weight) vs non-metal (dummy/Sphero) discrimination
//
//  ARCHITECTURE: same DISCRETE (bang-bang+hysteresis) / CONTINUOUS
//  (proportional/PD) split used throughout this project. See navigation.h
//  for the state machine and funnel_sensor.h for the sorting logic.
//
//  WHAT'S DIFFERENT FROM EARLIER NAV-ONLY BUILDS: there's now a real pickup
//  mechanism (collection.cpp). APPROACH still PD-steers onto a weight
//  candidate using the low ToF pair, but instead of a placeholder "drive
//  through it" capture, the robot now STOPS, calls collection_start(), and
//  waits for the crane to finish before moving on. See navigation.h's
//  MODE_PICKUP.
//
//  FUTURE HOOKS (not implemented - no hardware specifics given yet):
//    IMU        would replace the timed SCAN spin and open-loop CAPTURE/
//               REPOSITION straights with actual heading control. Search
//               for "IMU HOOK" comments in navigation.h for where it'd plug
//               in.
//    IR beacon  would let the robot home in on the base for a reliable
//               return-to-base, rather than relying on arena-wall avoidance
//               alone. Search for "BEACON HOOK" in navigation.h.
//
//  PIN CONFIDENCE - read this before flashing:
//    CONFIRMED   motor pins (from motor.cpp), collection pins (from
//                collection.cpp), low-front ToF XSHUT (4,3), front-top ToF
//                XSHUT (6,5) - these were confirmed in earlier conversation.
//    PLACEHOLDER everything else below marked CONFIRM - corner/rear ToF
//                XSHUT, all 3 IR analog pins, and the inductive digital pin.
//                None of these were specified - pick real pins, wire them,
//                and edit here before flashing.
// ============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// MODULE ENABLE SWITCHES
// ---------------------------------------------------------------------------
#define USE_CORNER_TOF  1     // 4th+5th top-tier ToF (angled corners)
#define USE_REAR_TOF    1
#define USE_SIDE_IR     1
#define USE_FUNNEL_SORT 1     // funnel IR + inductive classification
#define USE_SCAN        1

// ---------------------------------------------------------------------------
// ROBOT GEOMETRY  (mm)
// ---------------------------------------------------------------------------
const int ROBOT_WIDTH_MM   = 165;
const int ROBOT_HEIGHT_MM  = 170;
const int WEIGHT_HEIGHT_MM = 70;

// ---------------------------------------------------------------------------
// ToF ARRAY  -  7 sensors, XSHUT via SX1509 @ 0x3F
// ---------------------------------------------------------------------------
const byte SX1509_ADDRESS = 0x3F;
#define TOF_ADDRESS_START 0x30      // re-addressed 0x30..0x36 in index order

// index order used EVERYWHERE:
const int TOF_BL   = 0;   // low, left   (VL53L0X, short range) - weight tier
const int TOF_BR   = 1;   // low, right  (VL53L0X, short range) - weight tier
const int TOF_FL   = 2;   // top, front-left  (VL53L1X, long range)
const int TOF_FR   = 3;   // top, front-right (VL53L1X, long range)
const int TOF_CL   = 4;   // top, corner-left  angled out (VL53L1X, long)
const int TOF_CR   = 5;   // top, corner-right angled out (VL53L1X, long)
const int TOF_REAR = 6;   // top, rear (VL53L1X, long range)
const int TOF_COUNT = 7;

// SX1509 XSHUT pin for each index.
//   BL/BR/FL/FR = CONFIRMED (matches your wired-and-tested map).
//   CL/CR/REAR  = PLACEHOLDER (XSHUT2/1/0, i.e. CON29/28/27) - not yet
//                 specified, CONFIRM against your actual wiring.
const int XSHUT_TOF[TOF_COUNT] = { 4, 3, 6, 5, 2, 1, 0 };

// sensor model at each index: 0 = VL53L0X (short), 1 = VL53L1X (long)
const int TOF_TYPE[TOF_COUNT] = { 0, 0, 1, 1, 1, 1, 1 };

// ---------------------------------------------------------------------------
// ANALOG IR SENSORS  -  2x side (obstacle) + 1x funnel (weight presence)
// ---------------------------------------------------------------------------
// Side L/R = pins CONFIRMED to exist (A8Z/A9Z) but WHICH IS PHYSICALLY WHICH
// SIDE is not yet confirmed - see IR_LeftRight_Test (earlier in this
// project's history) if you haven't already. Funnel pin is a PLACEHOLDER.
const int PIN_IR_SIDE_L = A8;      // GP2Y0A41SK - CONFIRM L/R with test sketch
const int PIN_IR_SIDE_R = A9;      // GP2Y0A41SK - CONFIRM L/R with test sketch
const int PIN_IR_FUNNEL = A7;      // PLACEHOLDER - confirm your wiring

const float IR_ADC_VREF   = 3.3f;   // Teensy analog reference
const float IR_ADC_COUNTS = 1023.0f;// default 10-bit analogRead
const int   IR_SAMPLES    = 5;      // simple averaging - these are noisy

// GP2Y0A41SK curve (side + funnel sensors, both short range ~40-300mm)
const float IR_SIDE_A = 12080.0f;
const float IR_SIDE_B = -1.058f;
const int   IR_SIDE_MIN_MM = 40;
const int   IR_SIDE_MAX_MM = 300;

const int SIDE_NEAR_MM   = 80;
const int SIDE_NUDGE_PCT = 15;

// funnel: "something is present" just needs a much shorter range than the
// side-obstacle use of the same curve - the sensor sits right at the chute.
const int FUNNEL_PRESENT_MM = 60;

// ---------------------------------------------------------------------------
// INDUCTIVE SENSOR  -  metal (real weight) vs non-metal (dummy) at the
// funnel end. Digital output, PLACEHOLDER pin - confirm your wiring.
// Most inductive proximity sensors (e.g. LJ18A3-8-Z/BY) are NPN, active
// LOW when metal is detected - set INDUCTIVE_ACTIVE_LOW to match yours.
// ---------------------------------------------------------------------------
const int  PIN_INDUCTIVE        = 2;      // PLACEHOLDER - confirm your wiring
const bool INDUCTIVE_ACTIVE_LOW = true;

// how long after the funnel IR first sees something to keep sampling the
// inductive sensor before declaring a verdict (metal can trigger briefly
// mid-slide, not necessarily at the exact instant the IR trips)
const unsigned long SORT_CLASSIFY_WINDOW_MS = 200;

// ---------------------------------------------------------------------------
// OBSTACLE AVOIDANCE  (top-tier + corner ToF; bottom pair is weights only)
// ---------------------------------------------------------------------------
const int NEAR_MM       = 250;
const int FAR_MM        = 500;
const int CAUTION_MM    = 430;
const float KC_AVOID    = 0.11f;
const int CAUTION_VEER_MAX = 22;
const int CORNER_NEAR_MM = 200;   // angled corner sensors - tighter cone
const int REAR_STOP_MM   = 250;   // rear ToF: stop reversing inside this

const unsigned long MIN_TURN_MS    = 700;
const unsigned long MAX_TURN_MS    = 2500;
const unsigned long FLIP_WINDOW_MS = 2000;
const int  FLIPS_BEFORE_ESCAPE     = 2;
const unsigned long ESCAPE_REV_MS  = 450;
const unsigned long ESCAPE_SPIN_MS = 900;

// ---------------------------------------------------------------------------
// SPEEDS (%)
// ---------------------------------------------------------------------------
const int CRUISE_SPEED_PCT   = 100;
const int TURN_SPEED_PCT     = 100;
const int APPROACH_SPEED_PCT = 55;
const int SCAN_SPIN_PCT      = 50;
const int REPOSITION_SPEED_PCT = 55;
const unsigned long REPOSITION_TURN_MS = 700;

// ---------------------------------------------------------------------------
// WEIGHT DETECTION  (low-front-vs-top differential)
// ---------------------------------------------------------------------------
const int WEIGHT_MIN_MM        = 60;
const int WEIGHT_MAX_MM        = 700;
const int DIFF_CLEAR_MARGIN_MM = 250;
const int WEIGHT_STICK_MS      = 120;
const int PICKUP_TRIGGER_MM    = 115;   // close/aligned enough to stop + pick up

// ---------------------------------------------------------------------------
// PD STEERING for APPROACH (mm imbalance -> % differential)
// ---------------------------------------------------------------------------
const float KP_APPROACH = 0.16f;
const float KD_APPROACH = 0.045f;
const int   APPROACH_STEER_MAX = 22;
const int   ONE_SIDE_ARC_PCT   = 16;

// ---------------------------------------------------------------------------
// PICKUP / REPOSITION / suppression
// ---------------------------------------------------------------------------
// after a pickup ATTEMPT (regardless of confirmed success - a dummy weight
// won't be grabbed by the magnets, and we have no way to tell at ground
// level), suppress re-detecting the same spot so we don't loop on it.
const unsigned long TARGET_SUPPRESS_MS = 1500;

// ---------------------------------------------------------------------------
// SEARCH SCAN  (timed - no IMU on this rig yet, see "IMU HOOK" comments)
// ---------------------------------------------------------------------------
const unsigned long SCAN_TRIGGER_MS = 6000;
const unsigned long SCAN_MAX_MS     = 2600;

// ---------------------------------------------------------------------------
// SCHEDULING (ms)
// ---------------------------------------------------------------------------
const unsigned long TOF_READ_MS  = 55;
const unsigned long IR_READ_MS   = 30;
const unsigned long TELEMETRY_MS = 200;
