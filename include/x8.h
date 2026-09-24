#ifndef __X8_H
#define __X8_H
// ============================================================================
//  x8.h  -  DFRobot SEN0628 8x8 matrix ToF, front facing. The robot's only
//  forward obstacle sensor (replaces the old top-front + corner ToFs).
//
//  Everything below works on the OBSTACLE BAND (rows X8_BAND_LO..HI in
//  config.h), which must look above weight height so floor weights aren't
//  walls. Distances in mm, 0 = nothing in range (same as tof.h).
// ============================================================================

#include <Arduino.h>

void x8Init();
void x8Update();

bool x8Ok();          // initialised
bool x8Fresh();       // a frame arrived within X8_STALE_MS

uint16_t x8FrontMM();                 // nearest anywhere in the band
uint16_t x8LeftMM();                  // nearest in the robot's left half
uint16_t x8RightMM();                 // nearest in the robot's right half
float    x8BearingDeg(uint16_t *distOut);  // to the nearest return, + = right
bool     x8WallAhead();               // flat return across the width = wall

void x8PrintGrid();                   // whole 8x8, for orientation checks

#endif /* __X8_H */
