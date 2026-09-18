# Setup

Open each `.ino` in its matching sketch folder in the Arduino IDE.

## F310 transmitter

- Board: ESP32S3 Dev Module, USB CDC On Boot disabled.
- F310: D-switch mode, connected to the native USB host port. The source assumes VBUS through the board's OTG solder bridge; use the UART port for power and serial. With this wiring, do not connect the native port to a PC.
- Libraries: EspUsbHost and rtlopez's EspNowRcLink. The sketch expects the flattened `Transmitter.h` include layout.
- Power the transmitter before the esp-fc flight controller for automatic binding.

Channels are AETR: 0 roll, 1 pitch, 2 throttle, 3 yaw, 4 arm (AUX1), and 6 altitude-hold request (AUX3). The send interval is 20 ms.

Hold throttle down and press RB to arm; LB disarms. RT toggles the altitude-hold request. The spring-centered throttle gives roughly half throttle in manual mode. On altitude-hold selection, the transmitter sends center until the stick is re-centered. Check channel direction and receiver mode assignments with propellers removed.

## ESC calibration

Use ESP32 Dev Module and ESP32Servo. **Remove all propellers.** The utility immediately sends 2000 microseconds on GPIO 13, 25, 14 and 27 at 500 Hz. GPIO 0 (BOOT) switches all four outputs to 1000 microseconds. Follow the full battery/USB sequence and calibration-tone guidance in the source, then disconnect power and restore the flight firmware.

## Arduino flight experiment

`experiments/arduino-flight/` contains `fc_flight` and `tx_s3_arduino`. The FC implements MPU6500 complementary angle fusion, cascaded PID, a QUADX mixer and BMP280 altitude hold. The transmitter sends a raw `ControlPacket` with four `uint16_t` channel values and two `uint8_t` flags (`arm`, `altHold`) on Wi-Fi channel 1. This differs from the primary transmitter's EspNowRcLink protocol.

The FC uses ESP32Servo and the Arduino ESP32 core; its transmitter also needs EspUsbHost. Keep both packet definitions and channel settings aligned. The experiment is marked **UNTESTED**. Preserve the propellers-off checks, sensor calibration values and motor-direction checks in its source. The calibration utility and experimental FC label the first two motor positions differently; confirm physical motor mapping before use.
