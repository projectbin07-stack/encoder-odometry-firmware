// Fixed-rate tick from a hardware timer. The ISR only increments a counter;
// the control computation runs in loop() so that it can use Serial and
// floating point freely. loop() compares the tick count with the number of
// ticks it has serviced: more than one pending tick means the loop overran.
#pragma once

#include <stdint.h>

void control_timer_begin(uint32_t rate_hz);

/// Ticks since start (wraps after 2^32; compare with unsigned subtraction).
uint32_t control_timer_ticks();
