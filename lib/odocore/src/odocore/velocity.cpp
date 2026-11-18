#include "odocore/velocity.h"

#include <cmath>

namespace odocore {

void VelocityEstimator::reset() {
  primed_ = false;
  have_edge_ref_ = false;
  fixed_cps_ = 0.0f;
  period_cps_ = 0.0f;
  blended_cps_ = 0.0f;
  weight_ = 0.0f;
}

float VelocityEstimator::weight_for(float fixed_cps, const VelocityConfig& config) {
  const float speed = std::fabs(fixed_cps);
  if (config.high_cps <= config.low_cps) {
    return speed >= config.low_cps ? 1.0f : 0.0f;
  }
  if (speed <= config.low_cps) {
    return 0.0f;
  }
  if (speed >= config.high_cps) {
    return 1.0f;
  }
  return (speed - config.low_cps) / (config.high_cps - config.low_cps);
}

float VelocityEstimator::update(const EncoderSnapshot& s, uint32_t now_us) {
  if (!primed_) {
    primed_ = true;
    prev_count_ = s.count;
    prev_us_ = now_us;
    // The edge before priming may be arbitrarily old, so it is not a usable
    // reference for the period estimate.
    have_edge_ref_ = false;
    edge_count_ = s.count;
    edge_us_ = s.last_edge_us;
    return blended_cps_;
  }

  // Fixed-period estimate.
  const uint32_t dt = elapsed_us(now_us, prev_us_);
  const int32_t delta = count_delta(s.count, prev_count_);
  if (dt > 0) {
    fixed_cps_ = static_cast<float>(delta) * 1.0e6f / static_cast<float>(dt);
  }
  prev_count_ = s.count;
  prev_us_ = now_us;

  // Period estimate.
  const bool new_edge = (s.count != edge_count_) || (s.last_edge_us != edge_us_);
  if (new_edge) {
    const int32_t dc = count_delta(s.count, edge_count_);
    const uint32_t de = elapsed_us(s.last_edge_us, edge_us_);
    if (dc == 0) {
      // Edges came and went back to the same count: dithering on an edge.
      period_cps_ = 0.0f;
    } else if (have_edge_ref_ && de > 0) {
      period_cps_ = static_cast<float>(dc) * 1.0e6f / static_cast<float>(de);
    }
    edge_count_ = s.count;
    edge_us_ = s.last_edge_us;
    have_edge_ref_ = true;
  } else if (have_edge_ref_) {
    const uint32_t since = elapsed_us(now_us, edge_us_);
    if (since >= config_.zero_timeout_us) {
      period_cps_ = 0.0f;
      have_edge_ref_ = false;  // the next edge starts a fresh measurement
    } else if (since > 0) {
      // Had the wheel still been turning at period_cps_, an edge would have
      // arrived by now; the true speed is at most one count per `since`.
      const float bound = 1.0e6f / static_cast<float>(since);
      if (std::fabs(period_cps_) > bound) {
        period_cps_ = period_cps_ > 0.0f ? bound : -bound;
      }
    }
  }

  weight_ = weight_for(fixed_cps_, config_);
  blended_cps_ = weight_ * fixed_cps_ + (1.0f - weight_) * period_cps_;
  return blended_cps_;
}

}  // namespace odocore
