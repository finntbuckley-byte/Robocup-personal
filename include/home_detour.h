#pragma once
#include <cmath>
#include <cstdint>

namespace home {
// Only used on the return trip. Keep the avoidance side until the robot has
// actually travelled beyond the turn, rather than undoing it toward home.
class Detour {
public:
    Detour(float passMm, uint32_t clearMs) : passMm_(passMm), clearMs_(clearMs) {}
    void reset() { active_=passing_=clearTracking_=false; direction_=0; }
    int choose(int suggested) {
        if (!active_) { active_=true; direction_=suggested; }
        return direction_;
    }
    void update(uint32_t now, bool avoiding, bool clear, float x, float y, float heading) {
        if (!active_) return;
        if (avoiding) { passing_=clearTracking_=false; return; }
        if (!passing_) {
            passing_=true; x_=x; y_=y; heading_=heading;
            clearTracking_=false;
        }
        if (!clear) clearTracking_=false;
        else if (!clearTracking_) { clearTracking_=true; clearAt_=now; }
        const float radians=heading_*0.01745329252f;
        const float progress=(x-x_)*std::cos(radians)+(y-y_)*std::sin(radians);
        // Pivoting, a short clear glimpse, or elapsed time alone cannot release
        // commitment. A new obstacle resets the pass origin after its turn.
        if (clearTracking_ && now-clearAt_>=clearMs_ && progress>=passMm_) reset();
    }
    bool active() const { return active_; }
    bool passing() const { return active_ && passing_; }
    float heading() const { return heading_; }
private:
    float passMm_,x_=0,y_=0,heading_=0;
    uint32_t clearMs_,clearAt_=0;
    int direction_=0;
    bool active_=false,passing_=false,clearTracking_=false;
};
}
