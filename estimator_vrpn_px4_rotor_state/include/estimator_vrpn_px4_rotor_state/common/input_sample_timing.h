#pragma once

#include <algorithm>
#include <cmath>

namespace estimator_vrpn_px4_rotor_state::input_timing {

constexpr double kTimestampDuplicateToleranceSec = 1.0e-9;
constexpr double kMinRateDeltaSec = 1.0e-6;
// Same admission tolerance as xgc2_math::sampleTimeJumped. This is not a
// measurement-delay correction: the original sample timestamp is preserved.
constexpr double kFutureTimestampToleranceSec = 0.05;

// Update timing metadata before replacing sample.stamp_sec / sample.received.
// now_sec is the ROS receipt clock, never the timestamp of the incoming sample.
// Age limits remain in health_checks; a delayed, ordered sample is not a jump.
template <typename Sample>
void updateSampleTiming(Sample& sample, double stamp_sec, double now_sec) {
    const bool has_prev = sample.received && std::isfinite(sample.stamp_sec);
    const double raw_dt_sec =
        has_prev && std::isfinite(stamp_sec) ? stamp_sec - sample.stamp_sec : 0.0;
    const bool finite_dt = std::isfinite(raw_dt_sec);
    const bool invalid_clock = !std::isfinite(now_sec) || !std::isfinite(stamp_sec) ||
                               stamp_sec > now_sec + kFutureTimestampToleranceSec;
    sample.time_jump = invalid_clock ||
                       (has_prev && (!finite_dt || raw_dt_sec < -kTimestampDuplicateToleranceSec));
    sample.last_dt_sec = has_prev && finite_dt ? std::max(0.0, raw_dt_sec) : 0.0;
    if (!has_prev || sample.time_jump) {
        sample.estimated_rate_hz = 0.0;
    } else if (raw_dt_sec > kMinRateDeltaSec) {
        sample.estimated_rate_hz = 1.0 / raw_dt_sec;
    }
}

}  // namespace estimator_vrpn_px4_rotor_state::input_timing
