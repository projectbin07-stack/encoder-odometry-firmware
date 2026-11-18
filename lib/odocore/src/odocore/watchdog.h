// Command watchdog: if velocity commands stop arriving, stop the motors.
#pragma once

#include <stdint.h>

namespace odocore {

class CommandWatchdog {
 public:
  explicit CommandWatchdog(uint32_t timeout_us = 500000u) : timeout_us_(timeout_us) {}

  void set_timeout_us(uint32_t timeout_us) { timeout_us_ = timeout_us; }
  uint32_t timeout_us() const { return timeout_us_; }

  /// A valid command arrived at `now_us`.
  void feed(uint32_t now_us) {
    last_feed_us_ = now_us;
    expired_ = false;
  }

  /// Call once per control period. Returns true while commands are fresh.
  ///
  /// The watchdog starts expired (no command yet) without counting a trip. A
  /// trip is counted once per fresh -> expired transition. Once expired it
  /// stays expired until the next `feed`, so a 32-bit clock wrap during a long
  /// silence cannot make an old command look fresh again.
  bool update(uint32_t now_us) {
    if (expired_) {
      return false;
    }
    if (now_us - last_feed_us_ > timeout_us_) {
      expired_ = true;
      ++trips_;
      return false;
    }
    return true;
  }

  bool expired() const { return expired_; }
  uint32_t trips() const { return trips_; }

 private:
  uint32_t timeout_us_;
  uint32_t last_feed_us_ = 0;
  bool expired_ = true;
  uint32_t trips_ = 0;
};

}  // namespace odocore
