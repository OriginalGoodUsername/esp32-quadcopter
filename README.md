# ESP32 Quadcopter with F310 / ESP-NOW Control

A custom ESP32 quadcopter flown outdoors using an F310 USB gamepad, an ESP32-S3 transmitter, and ESP-NOW. The aircraft runs **esp-fc flight firmware**; my integration connects the gamepad to its receiver and configures the aircraft for controlled flight.

**Project by Vraj Patel · July–August 2026**

## Demonstrated behaviour

- Outdoor takeoff, hover, and manual flight with self-level stabilization.
- Live receiver-monitor response for roll, pitch, throttle, yaw, and arm/disarm commands.
- All four motors stopped within approximately 500 ms when the wireless control link was cut.
- All four ESCs calibrated, with motor mapping and spin direction checked before flight.

These are my observations from the completed build. Flight recordings and test logs are not uploaded yet; the approximate cutoff is an observed result, not a worst-case timing guarantee.

## Control path

```mermaid
flowchart LR
    G[Logitech F310 gamepad] -->|USB HID| T[ESP32-S3 transmitter]
    T -->|EspNowRcLink / ESP-NOW| F[ESP32 running esp-fc]
    F --> E[Four ESCs]
    E --> M[Four motors]
```

## My contribution

- Built and integrated the F310 transmitter interface with the esp-fc receiver.
- Mapped raw gamepad reports to flight-control and auxiliary channels, including arm/disarm handling.
- Configured stick direction, deadzone, response shaping, and throttle-command shaping in the transmitter sketch.
- Calibrated the ESCs, checked motor mapping and direction, and validated receiver input, outdoor flight, and wireless link-loss behaviour.

The transmitter uses the **EspUsbHost** and **EspNowRcLink** libraries. Flight stabilization and the receiver's flight-controller behaviour are provided by **esp-fc**. This repository does not claim that I wrote esp-fc's sensor fusion, PID controller, motor mixer, or ESP-NOW transport library.

## Contents

| Path | Contents |
|---|---|
| `firmware/tx_s3_espfc/tx_s3_espfc.ino` | F310 transmitter sketch copied from the local project |
| `tools/fc_esc_calibrate/fc_esc_calibrate.ino` | ESC calibration utility retained from development |
| [docs/SETUP.md](docs/SETUP.md) | Local dependencies and reproduction status |
| [docs/TESTING.md](docs/TESTING.md) | Confirmed test observations and limits |
| [media/](media/README.md) | Location for actual build photos and demo links |

The transmitter source also contains an altitude-hold auxiliary-channel toggle. Its presence in code does not establish a validated altitude-hold result; the confirmed flight claim is stabilized manual flight.

## Upstream projects

- [esp-fc](https://github.com/rtlopez/esp-fc): aircraft flight firmware.
- [EspNowRcLink](https://github.com/rtlopez/espnow-rclink): ESP-NOW RC transport.
- [EspUsbHost](https://github.com/tanakamasayuki/EspUsbHost): USB host support.

Upstream firmware and libraries are not copied into this repository. This is an integration project with local source and documentation, not a packaged, independently reproduced flight-controller release.
