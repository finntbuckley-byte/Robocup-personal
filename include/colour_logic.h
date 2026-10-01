#pragma once
#include "colour.h"
#include <cstdint>
inline ColourSurface classifyColour(uint16_t c,uint16_t r,uint16_t g,uint16_t b,
                                   uint16_t minimum,float baseRatio,float blueRatio) {
    // 50.4 ms / 2.4 ms * 1024 = 21504 full-scale; leave headroom.
    if(c<minimum || c>=21000 || !r || !g) return COLOUR_UNKNOWN;
    if(float(g)<float(r)*baseRatio) return COLOUR_FLOOR;
    return float(b)>=float(g)*blueRatio?COLOUR_BLUE:COLOUR_GREEN;
}
