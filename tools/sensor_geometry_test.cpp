#include "sensor_geometry.h"
#include <cassert>
#include <iostream>
int main() {
    assert(lowerViewIndex(true,true,0,1)==1);
    assert(lowerViewIndex(false,true,0,1)==0);
    assert(lowerViewIndex(true,false,0,1)==0);
    assert(lowerViewIndex(false,false,0,1)==1);
    for(unsigned r=0;r<8;++r) for(unsigned c=0;c<8;++c) {
        assert(matrixViewIndex(r,c,false,false)==r*8+c);
        assert(matrixViewIndex(r,c,true,false)==(7-r)*8+c);
        assert(matrixViewIndex(r,c,false,true)==r*8+7-c);
        assert(matrixViewIndex(r,c,true,true)==(7-r)*8+7-c);
    }
    std::cout << "260 sensor geometry assertions passed\n";
}
