#include "home_detour.h"
#include <cassert>
#include <iostream>
int main() {
    home::Detour d(350,500);
    assert(d.choose(-1)==-1 && d.choose(1)==-1);
    d.update(100,true,false,0,0,-30);
    d.update(200,false,true,0,0,-30);
    assert(d.passing() && d.heading()==-30);
    d.update(10000,false,true,0,0,150);
    assert(d.passing()); // no translation: elapsed time and rotation cannot end it
    d.update(10100,false,true,260,-150,-30);assert(d.active()); // ~300 mm
    d.update(10200,false,false,350,-202,-30);assert(d.active()); // >350, not clear
    d.update(10300,false,true,350,-202,-30);assert(d.active());
    d.update(10799,false,true,350,-202,-30);assert(d.active());
    d.update(10800,false,true,350,-202,-30);assert(!d.active());
    assert(d.choose(1)==1);
    d.update(11000,false,true,0,0,0);
    d.update(11500,false,true,-400,0,0);assert(d.active()); // backwards isn't a pass
    d.update(11600,true,false,400,0,45);assert(!d.passing() && d.active());
    assert(d.choose(-1)==1); // new obstacle keeps committed side
    d.update(12000,false,true,400,0,90);assert(d.heading()==90);
    d.update(13000,false,true,400,349,90);assert(d.active());
    d.update(13100,false,true,400,351,90);assert(!d.active());
    d.choose(-1);d.update(UINT32_MAX-100,false,true,0,0,0);
    d.update(399,false,true,400,0,0);assert(!d.active()); // timer wrap
    d.choose(1);d.reset();assert(!d.active() && !d.passing());
    home::Repetition r(150,12000,2);
    r.update(100,true,0,0);r.update(600,false,0,0);assert(!r.repeated());
    r.update(700,true,20,0);r.update(1200,false,20,0);assert(r.repeated());
    r.redirected();assert(!r.repeated());
    r.update(14000,true,20,0);r.update(15000,false,20,0);
    assert(!r.repeated()); // no second reversal just because time elapsed
    r.update(15100,true,170,0);r.update(15600,false,170,0);
    r.update(15700,true,170,0);r.update(16200,false,170,0);assert(r.repeated());
    r.reset();r.update(0,true,0,0);r.update(500,false,0,0);
    r.update(13000,true,0,0);r.update(13500,false,0,0);assert(!r.repeated());
    r.reset();r.update(0,true,0,0);r.update(10000,true,0,0);assert(!r.repeated()); // one long turn
    r.reset();r.update(UINT32_MAX-1000,true,0,0);r.update(UINT32_MAX-500,false,0,0);
    r.update(0,true,0,0);r.update(500,false,0,0);assert(r.repeated());
    r.reset();r.update(0,true,0,0);r.update(500,false,151,0);assert(!r.repeated()); // real progress
    std::cout<<"PASS return detour: commitment, translation, fresh clearance, re-obstacle, reset, timer wrap\n";
}
