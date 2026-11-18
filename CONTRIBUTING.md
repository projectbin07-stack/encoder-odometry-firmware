# Contributing

Bug reports, results from real hardware and small focused pull requests are
all welcome.

## Before you open a pull request

1. **C++ unit tests** (no hardware needed):

   ```sh
   pio test -e native
   ```

2. **Firmware builds** for every board environment:

   ```sh
   pio run -e esp32dev
   pio run -e esp32-s3-devkitc-1
   pio run -e esp32dev_drv_isr
   pio run -e nucleo_f401re
   ```

   If your change touches firmware code, include the RAM / Flash lines
   PlatformIO prints in the pull request.

3. **Host tool**:

   ```sh
   cd host
   pip install -e ".[dev]"
   ruff check . && ruff format --check . && mypy && pytest
   odobridge vectors
   ```

CI runs all of the above on every push and pull request.

## Guidelines

- `lib/odocore` must stay free of Arduino and vendor headers so it keeps
  building and testing on the host. Hardware access belongs in `src/`.
- New maths needs a test with expected values derived by hand, with the
  derivation in a comment, not numbers copied from the implementation's output.
  Where the Python firmware model mirrors the code, add the same case to
  `host/tests` too.
- **Protocol changes** touch four places: `lib/odocore/src/odocore/protocol.*`,
  `host/src/odobridge/protocol.py`, `docs/protocol.md`, and the shared vectors
  (`odobridge vectors --write`, then commit `test/vectors/protocol_vectors.json`).
  Changing an existing message's layout or meaning needs a new protocol
  version number.
- **Parameter changes** touch `params.cpp`, `host/src/odobridge/params.py`
  and the table in `docs/protocol.md`.
- Keep commits small, with an imperative subject line that says what changed.

## Reporting hardware results

The README states that nothing has been run on physical hardware yet. If you
flash it, please open an issue with the board, motor driver, motors and
encoders, the `board_config.h` changes you needed, and what you observed,
including anything that did not work, so the "Hardware validation status"
section can be updated accurately.
