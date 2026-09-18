// fc_esc_calibrate.ino  --  one-shot ESC throttle-range calibration (all 4)

// Calibrates the throttle endpoints of the four ESCs used in this build.
// Mismatched calibration can produce unequal thrust at the same command.
//
// Remove all propellers before running this sketch.
//
// PROCEDURE (no S3 needed -- uses the FC's BOOT button):
//   1. Flash this sketch over USB. Unplug USB.
//   2. Battery UNPLUGGED, wait 5 s. Then plug in the BATTERY.
//      -> the sketch is already holding all 4 signals at 2000 (MAX).
//      -> ESCs power up, see MAX throttle, enter calibration (special beeps).
//      -> LED blinks FAST while at MAX.
//   3. After the beeps (~2 s), press the FC's **BOOT** button once.
//      -> all 4 signals drop to 1000 (MIN). ESCs beep confirmation. LED solid.
//   4. Confirm each ESC's calibration tones. Unplug the battery and restore
//      the flight firmware. The flown configuration uses esp-fc.
//
// Board: "ESP32 Dev Module"

#include <ESP32Servo.h>

const int motPins[4] = {13, 25, 14, 27};   // M1 FR, M2 RR, M3 RL, M4 FL
const int ESC_FREQ = 500;                  // calibration signal frequency, Hz
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
