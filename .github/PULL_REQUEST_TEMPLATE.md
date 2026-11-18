## What this changes

## Why

## How it was checked

- [ ] `pio test -e native`
- [ ] `pio run` for the board environments affected
- [ ] `cd host && ruff check . && ruff format --check . && mypy && pytest` (if `host/` changed)
- [ ] `odobridge vectors` (if the protocol changed; vectors regenerated and committed)
- [ ] Tried on real hardware (say which board, driver and motors, or "not tested on hardware")

## Firmware size (if firmware code changed)

Paste the RAM / Flash lines PlatformIO prints.
