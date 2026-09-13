// ============================================================================
// fc_esc_calibrate.ino  --  one-shot ESC throttle-range calibration (all 4)
// ----------------------------------------------------------------------------
// Restores the ESC-sync step from the original repo's build order (we deleted
// its motor_calibration sketch). Mismatched ESC calibration = unequal thrust
// at the same throttle = tips over / rams on takeoff.
//
//   >>>>>  PROPS OFF. ABSOLUTELY NO PROPS FOR THIS ONE.  <<<<<
//
// PROCEDURE (no S3 needed -- uses the FC's BOOT button):
//   1. Flash this sketch over USB. Unplug USB.
//   2. Battery UNPLUGGED, wait 5 s. Then plug in the BATTERY.
//      -> the sketch is already holding all 4 signals at 2000 (MAX).
//      -> ESCs power up, see MAX throttle, enter calibration (special beeps).
//      -> LED blinks FAST while at MAX.
//   3. After the beeps (~2 s), press the FC's **BOOT** button once.
//      -> all 4 signals drop to 1000 (MIN). ESCs beep confirmation. LED solid.
//   4. Done -- all 4 ESCs now share the exact same 1000..2000 range.
//      Unplug battery, reflash fc_flight.
//
// Board: "ESP32 Dev Module"
// ============================================================================

#include <ESP32Servo.h>

const int motPins[4] = {13, 25, 14, 27};   // M1 FR, M2 RR, M3 RL, M4 FL
const int ESC_FREQ = 500;                  // same frequency the flight code uses
const int LED_PIN  = 2;
const int BOOT_BTN = 0;                    // BOOT button on the DevKit = GPIO0

Servo mot[4];
bool atMax = true;

void writeAll(int us) { for (int i = 0; i < 4; i++) mot[i].writeMicroseconds(us); }

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BOOT_BTN, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < 4; i++) {
    mot[i].attach(motPins[i], 1000, 2000);
    mot[i].setPeriodHertz(ESC_FREQ);
  }
  writeAll(2000);                          // MAX immediately: ESCs boot into cal mode
}

void loop() {
  if (atMax) {
    digitalWrite(LED_PIN, (millis() / 100) % 2);      // fast blink = holding MAX
    if (digitalRead(BOOT_BTN) == LOW) {               // BOOT pressed -> finish cal
      writeAll(1000);
      atMax = false;
      digitalWrite(LED_PIN, HIGH);                    // solid = MIN written, done
    }
  }
  delay(10);
}
