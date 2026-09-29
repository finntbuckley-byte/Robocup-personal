#pragma once
#include <cstdint>

// Convert physical mounting/index coordinates into robot viewing coordinates.
constexpr int lowerViewIndex(bool left, bool crossed, int physicalLeft, int physicalRight) {
    return (left != crossed) ? physicalLeft : physicalRight;
}
constexpr unsigned matrixViewIndex(unsigned row, unsigned col, bool flipRows, bool flipCols) {
    return (flipRows ? 7-row : row)*8 + (flipCols ? 7-col : col);
}
