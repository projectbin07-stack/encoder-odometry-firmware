// Wheel velocity estimation from encoder snapshots.
//
// Two classic estimators, and a blend of the two:
//
//  * Fixed-period ("M method"): counts gained during one control period
//    divided by the period. Good at high speed; at low speed it is quantised
//    to multiples of 1 / period (100 counts/s at a 100 Hz loop), so a wheel
//    turning at 50 counts/s reads 0, 100, 0, 100, ...
//  * Period ("T method", 1/T): counts between the two most recent edge
//    timestamps divided by the time between them. Exact for a constant speed
//    whatever the speed, but it needs edge times and goes stale when edges
//    stop; it is therefore bounded by 1 / (time since last edge) and forced to
//    zero after `zero_timeout_us` without an edge.
//  * Blended: weight 0 (pure period) below `low_cps`, weight 1 (pure fixed)
//    above `high_cps`, linear in between, chosen on the fixed-period speed.
//
// All speeds are in encoder counts per second. 32-bit count and timestamp
// wraparound are handled with modular arithmetic.
#pragma once

#include <stdint.h>

#include "odocore/quadrature.h"

namespace odocore {

struct VelocityConfig {
  float low_cps = 200.0f;              ///< Below this, use the period estimate only.
  float high_cps = 800.0f;             ///< Above this, use the fixed-period estimate only.
  uint32_t zero_timeout_us = 250000u;  ///< No edge for this long means stopped.
};

class VelocityEstimator {
 public:
  explicit VelocityEstimator(const VelocityConfig& config = VelocityConfig()) : config_(config) {}

  void configure(const VelocityConfig& config) { config_ = config; }
  const VelocityConfig& config() const { return config_; }
  void reset();

  /// Update with a snapshot taken at `now_us`; returns the blended estimate.
  /// The first call only primes the estimator and returns 0.
  float update(const EncoderSnapshot& snapshot, uint32_t now_us);

  float fixed_cps() const { return fixed_cps_; }
  float period_cps() const { return period_cps_; }
  float blended_cps() const { return blended_cps_; }
  /// Weight of the fixed-period estimate in the last blend, 0..1.
  float blend_weight() const { return weight_; }

  /// Blend weight for a given fixed-period speed (exposed for tests).
  static float weight_for(float fixed_cps, const VelocityConfig& config);

 private:
  VelocityConfig config_;
  bool primed_ = false;
  int32_t prev_count_ = 0;
  uint32_t prev_us_ = 0;
  bool have_edge_ref_ = false;
  int32_t edge_count_ = 0;
  uint32_t edge_us_ = 0;
  float fixed_cps_ = 0.0f;
  float period_cps_ = 0.0f;
  float blended_cps_ = 0.0f;
  float weight_ = 0.0f;
};

}  // namespace odocore
