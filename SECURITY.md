# Security policy

## Supported versions

Only the latest release receives fixes.

## Reporting a vulnerability

Please report security problems privately through GitHub's
[private vulnerability reporting](https://github.com/projectbin07-stack/encoder-odometry-firmware/security/advisories/new)
rather than in a public issue. Include the affected version or commit, what
you observed and, if possible, a way to reproduce it. You should get a reply
within two weeks.

## Scope notes

- The serial protocol has no authentication or encryption. Anyone who can
  reach the serial port can drive the motors and change parameters. Do not
  expose the port (for example through a network serial bridge) to untrusted
  parties.
- The command watchdog stops the motors when commands stop arriving. It is a
  convenience, not a safety system: it does not cover firmware crashes with
  outputs latched, driver faults or runaway caused by wrong encoder polarity.
  Use a physical emergency stop that cuts motor power.
- Received frames are treated as untrusted input on both sides: anything that
  fails COBS, CRC, version, type or length checks is counted and dropped, and
  non-finite velocity commands and out-of-range parameters are rejected.
