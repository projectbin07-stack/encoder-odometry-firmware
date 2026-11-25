# odobridge

Host-side bridge for [encoder-odometry-firmware](../README.md): an encoder and
decoder for the firmware's COBS/CRC-16 serial protocol, a CLI to stream
velocity commands and log odometry, and a pure-Python model of the firmware
control loop (`--port sim`, `odobridge simulate`) for testing without hardware.

Runtime dependencies: the standard library and `pyserial` (plus `tomli` on
Python 3.10). See the main README for usage and
[docs/host-api.md](../docs/host-api.md) for the supported firmware-model API.

```sh
pip install -e "./host[dev]"
odobridge --help
```
