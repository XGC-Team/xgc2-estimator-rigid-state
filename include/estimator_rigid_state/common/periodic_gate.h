#pragma once

#include <cmath>

namespace estimator_rigid_state {

class PeriodicGate {
   public:
    bool due(double now_sec, double period_sec) {
        if (!std::isfinite(now_sec) || !std::isfinite(period_sec) || period_sec <= 0.0) {
            return false;
        }
        if (!initialized_ || now_sec < last_fire_sec_ || now_sec - last_fire_sec_ >= period_sec) {
            last_fire_sec_ = now_sec;
            initialized_ = true;
            return true;
        }
        return false;
    }

    void reset() {
        initialized_ = false;
        last_fire_sec_ = 0.0;
    }

   private:
    bool initialized_{false};
    double last_fire_sec_{0.0};
};

}  // namespace estimator_rigid_state
