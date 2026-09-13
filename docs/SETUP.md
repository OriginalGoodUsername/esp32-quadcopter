# Source and reproduction notes

## Transmitter

The supplied `tx_s3_espfc.ino` sketch targets an ESP32-S3 and reads an F310 in its D-switch mode through USB host support. The local source specifies the ESP32S3 Dev Module board and USB CDC On Boot disabled. Consult the source comments for the particular board's USB connection assumptions.

The local development libraries identify themselves as:

| Dependency | Locally reported version | Note |
|---|---|---|
| EspNowRcLink | 0.1.1 | Local library metadata describes flattened includes for Arduino IDE; the sketch includes `Transmitter.h` |
| EspUsbHost | 2.1.0 | The sketch uses USB HID input and device connection callbacks |
| Arduino ESP32 core | Not recorded in this repository | Exact working version still needs to be captured |

These metadata values do not prove that a fresh upstream download matches the locally installed library source. A clean dependency install and compile have not been performed for this repository. Exact dependency revisions and the working aircraft configuration remain to be added before claiming reproducible builds.

The source sets a nominal 20 ms transmitter scheduling interval. That is a code setting, not a measured end-to-end control latency.

## ESC calibration utility

The calibration utility immediately commands maximum throttle signals on the four configured pins, then switches to minimum when the board's BOOT button is pressed. **Remove all propellers before using this utility.** Its pin mapping and signal configuration belong to the original hardware; review them before adapting it.

The confirmed flying configuration used **esp-fc**. An older `fc_flight` sketch remains outside this release; it is not the source of the reported outdoor flight.

Source comments were shortened and corrected during publication; executable C++ tokens were checked against the original files and remain unchanged. No firmware was flashed and no connected hardware was operated.
