# ESP32 quadcopter

I assembled an F450 quadcopter with an ESP32 flight controller running [esp-fc](https://github.com/rtlopez/esp-fc). I wired an MPU-6500 IMU, BMP280 barometer and QMC5883L magnetometer over I2C, calibrated the four ESCs, and checked motor mapping and spin direction.

My controller uses a Logitech F310 USB gamepad connected to an ESP32-S3. I integrated its inputs with the esp-fc receiver over ESP-NOW, including stick mapping, deadzones, response shaping, throttle slew limiting and arming controls. Flight stabilization, sensor fusion, PID control and motor mixing come from esp-fc. I flew the quadcopter outdoors with manual gamepad control and self-level stabilization.

![Assembled F450 quadcopter](media/assembled-quadcopter.jpg)

![Betaflight Configurator Setup tab with a tilted quadcopter model](media/betaflight-configurator.jpg)

Betaflight Configurator setup view; the aircraft uses esp-fc firmware. More [build photos](media/README.md).

## Files and use

| Path | Purpose |
|---|---|
| [firmware/tx_s3_espfc/](firmware/tx_s3_espfc/) | F310 transmitter for the esp-fc receiver |
| [tools/fc_esc_calibrate/](tools/fc_esc_calibrate/) | Four-ESC throttle-range calibration |
| [experiments/arduino-flight/](experiments/arduino-flight/) | Separate, untested Arduino flight controller and raw ESP-NOW transmitter |
| [docs/SETUP.md](docs/SETUP.md) | Board settings, dependencies and controls |
| [docs/TESTING.md](docs/TESTING.md) | Observations from the completed esp-fc build |

The transmitter uses [EspUsbHost](https://github.com/tanakamasayuki/EspUsbHost) for USB input and [EspNowRcLink](https://github.com/rtlopez/espnow-rclink) for the RC link. The calibration utility uses ESP32Servo. Remove all propellers for bench checks and ESC calibration; follow the pin mapping and power sequence in the sketch.
