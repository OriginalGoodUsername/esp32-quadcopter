// ============================================================================
// tx_s3_espfc.ino  --  Logitech F310 (USB host) -> EspNowRcLink -> esp-fc
// ----------------------------------------------------------------------------
// Sends the F310 sticks as an 8-channel RC link that esp-fc's BUILT-IN ESP-NOW
// receiver understands (same EspNowRcLink library it uses). Auto-binds.
//   >>> PROPS OFF for bench testing. <<<
//
// BOARD: "ESP32S3 Dev Module", USB CDC On Boot = DISABLED.
//   F310 (switch D) -> native "USB" port (VBUS via the OTG solder-bridge).
//   Power + Serial via the UART port.  *** Never plug the native port into a PC.
//
// LIBRARY: install "espnow-rclink" (rtlopez) -- download the repo ZIP and
//   Arduino IDE > Sketch > Include Library > Add .ZIP Library.
//
// CHANNEL MAP (AETR -- matches esp-fc's default input map):
//   ch0 = roll  ch1 = pitch  ch2 = throttle  ch3 = yaw  ch4 = ARM(AUX1)  ch6 = ALTHOLD(AUX3)
//
// BINDING: power THIS transmitter first, then power the FC -- it auto-binds.
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "EspUsbHost.h"
#include "Transmitter.h"   // EspNowRcLink (flattened lib); namespace still EspNowRcLink::

// ---- F310 "Dual Action" raw report indices (discovered earlier) ----
#define IDX_LX 0    // left  X -> yaw
#define IDX_LY 1    // left  Y -> throttle
#define IDX_RX 2    // right X -> roll
#define IDX_RY 3    // right Y -> pitch
#define IDX_BTN 5
#define BTN_ARM  0x02   // RB
#define BTN_STOP 0x01   // LB
#define BTN_ALTHOLD 0x08 // RT -> toggles altitude hold (sent on AUX3/ch6)

// ---- flip if a stick is backwards (check esp-fc Receiver tab) ----
bool invRoll = false, invPitch = true, invYaw = false;   // pitch reversed per user
bool thrUpIsLow = true;              // F310: pushing the stick UP gives a LOW value

// ---- stick feel (lower rate = LESS sensitive) ----
float RC_RATE_RP  = 0.6f;            // ROLL/PITCH authority (was 0.8) -> less sensitive. Lower = calmer.
float RC_EXPO_RP  = 0.55f;           // ROLL/PITCH center softening (higher = gentler around center)
float RC_RATE_YAW = 0.8f;            // yaw authority (left as-is)
float RC_EXPO_YAW = 0.45f;           // yaw center softening
float applyExpo(float x, float e);   // defined below (soft curve, keeps the endpoints)

// ---- throttle feel: CENTERED climb/descend stick for esp-fc ALTHOLD (baro altitude hold) ----
//   push UP = climb    release (stick centers) = HOLD height    push DOWN = descend
// esp-fc reads throttle center (1500) as "hold". The F310 throttle springs to center, so
// letting go = hold. Toggle ALTHOLD with RT (sent on AUX3). With ALTHOLD off the same stick
// is manual throttle (center = ~half). Arm = hold the throttle DOWN + RB.
float    THR_EXPO  = 0.5f;            // softens the climb/descend response near center (higher = gentler)
const uint16_t THR_SLEW = 8;          // ramps the throttle command so climb/descend isn't abrupt (0 = instant)

EspNowRcLink::Transmitter tx;
EspUsbHost usb;

volatile uint8_t rep[8] = {127, 127, 127, 124, 8, 0, 8, 255};
volatile bool gpPresent = false;
bool armed = false, altHold = false, altRecenter = false; uint8_t lastBtn = 0;
uint32_t lastSend = 0;
int thrCmd = 1000;                   // slew-limited throttle actually sent

uint16_t mapAxis(uint8_t v, bool inv, float rate, float expo) {  // 0..255 -> 1000..2000, deadzone + expo + rate
  const int c = 127, dz = 8;
  float x;                                          // normalized stick, -1..1
  if (abs((int)v - c) < dz) x = 0.0f;
  else if (v > c)           x =  (float)(v - c) / (255 - c);
  else                      x = -(float)(c - v) / c;
  x = applyExpo(x, expo) * rate;                    // soften center + scale authority
  int out = 1500 + (int)(x * 500.0f);
  if (inv) out = 3000 - out;
  return (uint16_t)constrain(out, 1000, 2000);
}

float applyExpo(float x, float e) {               // x in [-1,1], e in [0,1]; softens near 0, keeps the ends
  return x * ((1.0f - e) + e * x * x);
}

uint16_t mapThrottleCentered(uint8_t v) {         // ALTHOLD climb-rate stick: down=1000, CENTER=1500(hold), up=2000
  const int c = 127, dz = 8;
  float x;                                          // -1..+1  (+ = climb)
  if (abs((int)v - c) < dz) x = 0.0f;
  else if (v > c)           x =  (float)(v - c) / (255 - c);
  else                      x = -(float)(c - v) / c;
  if (thrUpIsLow) x = -x;                            // F310: up = LOW value -> flip so UP = climb
  x = applyExpo(x, THR_EXPO);
  return (uint16_t)constrain(1500 + (int)(x * 500.0f), 1000, 2000);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  usb.onHIDInput([](const EspUsbHostHIDInput &in) {
    for (size_t i = 0; i < in.length && i < 8; i++) rep[i] = in.data[i];
  });
  usb.onDeviceConnected([](const EspUsbHostDeviceInfo &d) { gpPresent = true;  Serial.println("[F310] connected"); });
  usb.onDeviceDisconnected([](const EspUsbHostDeviceInfo &d) { gpPresent = false; Serial.println("[F310] GONE"); });
  usb.begin();

  WiFi.mode(WIFI_STA);               // STA mode (no softAP) -> exact channel control for discovery
  WiFi.disconnect();                 // not joining any AP, so the channel is free to hop/lock
  tx.begin(false);                   // we manage WiFi; NO hidden AP (fixes post-pair channel drift)
  esp_wifi_set_ps(WIFI_PS_NONE);     // keep the radio awake so USB-host reports aren't starved

  Serial.println("[TX] esp-fc link ready. PROPS OFF. Hold thr DOWN + RB = arm; RT = ALT-HOLD toggle; LB = disarm.");
}

void loop() {
  // ---- map the F310 ----
  uint16_t roll   = mapAxis(rep[IDX_RX], invRoll,  RC_RATE_RP,  RC_EXPO_RP);
  uint16_t pitch  = mapAxis(rep[IDX_RY], invPitch, RC_RATE_RP,  RC_EXPO_RP);
  uint16_t yaw    = mapAxis(rep[IDX_LX], invYaw,   RC_RATE_YAW, RC_EXPO_YAW);
  uint16_t thrRaw = mapThrottleCentered(rep[IDX_LY]);

  // ---- arm: hold throttle DOWN + RB; LB disarms; RT toggles alt-hold ----
  uint8_t btn = rep[IDX_BTN];
  if ((btn & BTN_ARM) && !(lastBtn & BTN_ARM) && thrRaw < 1050) armed = true;
  if (btn & BTN_STOP) armed = false;
  if ((btn & BTN_ALTHOLD) && !(lastBtn & BTN_ALTHOLD)) { altHold = !altHold; if (altHold) altRecenter = true; }  // RT toggles; engage = hold until re-centered
  lastBtn = btn;
  if (!gpPresent) armed = false;     // controller unplugged -> disarm
  if (!armed) altHold = false;       // never leave alt-hold engaged while disarmed

  uint32_t now = millis();
  if (now - lastSend >= 20) {         // 50 Hz
    lastSend = now;
    // Safe alt-hold engage: after toggling ON, command HOLD (1500) and ignore the
    // stick until it's re-centered -> engaging mid-climb can't run into the ceiling.
    if (altRecenter && abs((int)thrRaw - 1500) < 40) altRecenter = false;
    // throttle: DISARMED -> idle 1000 (clears esp-fc THROTTLE arm-check).
    // ARMED: hold(1500) while re-centering, else the centered stick, slew-limited.
    uint16_t thrTarget = !armed ? 1000 : ((altHold && altRecenter) ? 1500 : thrRaw);
    if (THR_SLEW == 0 || (unsigned)abs((int)thrTarget - thrCmd) <= THR_SLEW)
      thrCmd = thrTarget;
    else
      thrCmd += (thrTarget > thrCmd) ? (int)THR_SLEW : -(int)THR_SLEW;

    // default all channels to center, then set the ones we use
    for (uint8_t c = EspNowRcLink::RC_CHANNEL_MIN; c <= EspNowRcLink::RC_CHANNEL_MAX; c++)
      tx.setChannel(c, 1500);
    tx.setChannel(0, roll);
    tx.setChannel(1, pitch);
    tx.setChannel(2, (uint16_t)thrCmd);
    tx.setChannel(3, yaw);
    tx.setChannel(4, armed ? 2000 : 1000);       // AUX1 = ARM
    tx.setChannel(6, altHold ? 2000 : 1000);     // AUX3 = ALTHOLD toggle (RT)
    tx.commit();

    static uint32_t dbg = 0;
    if (now - dbg >= 500) { dbg = now;
      Serial.printf("%s st=%d thr=%u(raw%u) roll=%u arm=%s alt=%s txOK=%lu txFail=%lu\n",
                    gpPresent ? "GP" : "--", (int)tx.getState(),
                    (unsigned)thrCmd, (unsigned)thrRaw, roll, armed ? "YES" : "no",
                    altHold ? "ON" : "off",
                    (unsigned long)tx.dbgSendOk(), (unsigned long)tx.dbgSendFail());
    }
  }

  tx.update();                        // send + handle telemetry/binding
}
