// tx_s3_arduino.ino  --  F310 (USB host) -> ESP-NOW ControlPacket -> fc_flight
// Transmitter for the experimental Arduino flight controller (fc_flight).
// Sends a raw ControlPacket over ESP-NOW on a FIXED channel (must match the FC).
// F310 input mapping, expo/rates, centered throttle and RT altitude-hold toggle.
// Remove all propellers for bench testing.
//
// BOARD: "ESP32S3 Dev Module", USB CDC On Boot = DISABLED.
//   F310 (switch D) -> native "USB" port.  Power + Serial via the UART port.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "EspUsbHost.h"

#define WIFI_CHANNEL 1        // MUST match fc_flight's WIFI_CHANNEL

// F310 "Dual Action" report indices
#define IDX_LX 0   // yaw
#define IDX_LY 1   // throttle
#define IDX_RX 2   // roll
#define IDX_RY 3   // pitch
#define IDX_BTN 5
#define BTN_ARM     0x02   // RB
#define BTN_STOP    0x01   // LB
#define BTN_ALTHOLD 0x08   // RT -> toggle altitude hold

// stick response
bool invRoll = false, invPitch = true, invYaw = false;    // pitch reversed
bool thrUpIsLow = true;
float RC_RATE_RP = 0.6f, RC_EXPO_RP = 0.55f;              // roll/pitch: less sensitive
float RC_RATE_YAW = 0.8f, RC_EXPO_YAW = 0.45f;            // yaw
float THR_EXPO = 0.5f;
const uint16_t THR_SLEW = 8;

// ControlPacket -- MUST match fc_flight byte-for-byte
typedef struct {
  uint16_t roll, pitch, throttle, yaw;
  uint8_t  arm, altHold;
} ControlPacket;

static const uint8_t BCAST[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
EspUsbHost usb;
volatile uint8_t rep[8] = {127,127,127,124,8,0,8,255};
volatile bool gpPresent = false;
bool armed = false, altHold = false, altRecenter = false; uint8_t lastBtn = 0;
uint32_t lastSend = 0;
int thrCmd = 1000;

float applyExpo(float x, float e) { return x * ((1.0f - e) + e * x * x); }

uint16_t mapAxis(uint8_t v, bool inv, float rate, float expo) {  // centered stick -> 1000..2000, expo+rate
  const int c = 127, dz = 8;
  float x;
  if (abs((int)v - c) < dz) x = 0;
  else if (v > c)           x =  (float)(v - c) / (255 - c);
  else                      x = -(float)(c - v) / c;
  x = applyExpo(x, expo) * rate;
  int out = 1500 + (int)(x * 500.0f);
  if (inv) out = 3000 - out;
  return (uint16_t)constrain(out, 1000, 2000);
}

uint16_t mapThrottleCentered(uint8_t v) {         // down=1000, CENTER=1500(hold), up=2000
  const int c = 127, dz = 8;
  float x;
  if (abs((int)v - c) < dz) x = 0;
  else if (v > c)           x =  (float)(v - c) / (255 - c);
  else                      x = -(float)(c - v) / c;
  if (thrUpIsLow) x = -x;
  x = applyExpo(x, THR_EXPO);
  return (uint16_t)constrain(1500 + (int)(x * 500.0f), 1000, 2000);
}

void setup() {
  Serial.begin(115200); delay(300);
  usb.onHIDInput([](const EspUsbHostHIDInput &in) {
    for (size_t i = 0; i < in.length && i < 8; i++) rep[i] = in.data[i];
  });
  usb.onDeviceConnected([](const EspUsbHostDeviceInfo &d) { gpPresent = true;  Serial.println("[F310] connected"); });
  usb.onDeviceDisconnected([](const EspUsbHostDeviceInfo &d) { gpPresent = false; Serial.println("[F310] GONE"); });
  usb.begin();

  WiFi.mode(WIFI_STA); WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_now_init();
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.ifidx = WIFI_IF_STA; peer.channel = WIFI_CHANNEL; peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("[TX] Arduino-FC link ready. PROPS OFF. Hold thr DOWN + RB = arm; RT = alt-hold; LB = disarm.");
}

void loop() {
  uint16_t roll   = mapAxis(rep[IDX_RX], invRoll,  RC_RATE_RP,  RC_EXPO_RP);
  uint16_t pitch  = mapAxis(rep[IDX_RY], invPitch, RC_RATE_RP,  RC_EXPO_RP);
  uint16_t yaw    = mapAxis(rep[IDX_LX], invYaw,   RC_RATE_YAW, RC_EXPO_YAW);
  uint16_t thrRaw = mapThrottleCentered(rep[IDX_LY]);

  uint8_t btn = rep[IDX_BTN];
  if ((btn & BTN_ARM) && !(lastBtn & BTN_ARM) && thrRaw < 1050) armed = true;
  if (btn & BTN_STOP) armed = false;
  if ((btn & BTN_ALTHOLD) && !(lastBtn & BTN_ALTHOLD)) { altHold = !altHold; if (altHold) altRecenter = true; }  // engage = hold until re-centered
  lastBtn = btn;
  if (!gpPresent) armed = false;
  if (!armed) altHold = false;

  uint32_t now = millis();
  if (now - lastSend >= 20) {         // 50 Hz
    lastSend = now;
    // On altitude-hold selection, send 1500 until the stick is re-centered.
    if (altRecenter && abs((int)thrRaw - 1500) < 40) altRecenter = false;
    uint16_t thrTarget = !armed ? 1000 : ((altHold && altRecenter) ? 1500 : thrRaw);
    if (THR_SLEW == 0 || (unsigned)abs((int)thrTarget - thrCmd) <= THR_SLEW) thrCmd = thrTarget;
    else thrCmd += (thrTarget > thrCmd) ? (int)THR_SLEW : -(int)THR_SLEW;

    ControlPacket pkt;
    pkt.roll = roll; pkt.pitch = pitch; pkt.throttle = (uint16_t)thrCmd; pkt.yaw = yaw;
    pkt.arm = armed ? 1 : 0; pkt.altHold = altHold ? 1 : 0;
    esp_now_send(BCAST, (const uint8_t*)&pkt, sizeof(pkt));

    static uint32_t dbg = 0;
    if (now - dbg >= 500) { dbg = now;
      Serial.printf("%s thr=%u roll=%u pitch=%u yaw=%u arm=%s alt=%s\n",
                    gpPresent ? "GP" : "--", (unsigned)thrCmd, roll, pitch, yaw,
                    armed ? "YES" : "no", altHold ? "ON" : "off");
    }
  }
}
