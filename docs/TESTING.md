# Quadcopter validation

These are my reported observations from the completed build. No physical flight or motor tests were performed while preparing this repository.

| Check | Observed result |
|---|---|
| Receiver input | F310 inputs registered live in the flight-controller receiver monitor |
| Channel mapping | Roll, pitch, throttle, yaw, and arm/disarm commands worked |
| ESC bring-up | All four ESCs calibrated; motor mapping and spin direction verified |
| Outdoor flight | Took off, hovered, and flew under manual gamepad control using esp-fc |
| Self-level mode | Aircraft held level and was controllable in flight |
| Wireless link loss | Cutting the wireless control link stopped all four motors within approximately 500 ms |

## Interpretation

The link-loss observation is not a measured worst-case guarantee or a multi-trial latency distribution. The test method, trial count, and measurement equipment were not recorded in the evidence supplied for this repository. A gamepad-disconnection code path is present in the transmitter; it is a separate condition from the confirmed wireless-link-loss test.

Flight stabilization belongs to esp-fc. The transmitter's auxiliary altitude-hold switch does not establish tested altitude hold. No positioning precision, endurance, repeated-flight success rate, or autonomous flight result is claimed.

## Useful next evidence

- A short outdoor flight video with takeoff, hover, and manual control visible.
- Receiver-monitor capture showing the four primary control channels and arm/disarm.
- A documented propeller-off link-loss check with a visible timing reference.
- A configuration export identifying the esp-fc build and settings used for the confirmed flight.
