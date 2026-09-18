// fc_flight.ino: experimental ESP32 flight controller with barometric altitude hold
// MPU6500 IMU -> complementary angle fusion ->
// cascaded angle+rate PID -> QUADX mixer -> 4 PWM ESCs, sticks over ESP-NOW,
// plus BMP280 BAROMETRIC ALTITUDE HOLD (RT toggle on the transmitter).
// Uses the raw ESP-NOW ControlPacket sent by tx_s3_arduino.
//
// UNTESTED. Keep propellers off until bench checks pass.
// Verify self-level direction and failsafe before any prop-on flight.
//
// BOARD: "ESP32 Dev Module".
// HARDWARE: M1=GPIO13 rear-right, M2=GPIO25 front-right,
//   M3=GPIO14 rear-left, M4=GPIO27 front-left. MPU6500 @0x68, BMP280 @0x76,
//   I2C SDA21/SCL22.
//
// BENCH TEST (props off, armed, tiny throttle, alt-hold OFF):
//   tilt RIGHT     -> right motors (M1,M2) speed up   (else ROLL_DIR = -1)
//   tilt NOSE-DOWN -> rear motors  (M1,M3) speed up   (else PITCH_DIR = -1)
//   yaw: verify in the air, low & slow                (else YAW_DIR = -1)

#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

#define WIFI_CHANNEL 1
#define FAILSAFE_MS  500

// self-level direction flips (set -1 if a bench tilt test is backwards)
int ROLL_DIR = 1, PITCH_DIR = 1, YAW_DIR = 1;

// motor positions
const int mot1_pin = 13, mot2_pin = 25, mot3_pin = 14, mot4_pin = 27;
int ESCfreq = 500;
int ThrottleIdle = 1170, ThrottleCutOff = 1000;
#define THR_MAX_MIX 1800

// attitude PID (angle=self-level, rate=inner)
float PAngleRoll = 3, PAnglePitch = 3, IAngleRoll = 0.5, IAnglePitch = 0.5, DAngleRoll = 0.007, DAnglePitch = 0.007;
float PRateRoll = 0.625, IRateRoll = 2.1, DRateRoll = 0.0088;
float PRatePitch = 0.625, IRatePitch = 2.1, DRatePitch = 0.0088;
float PRateYaw = 4, IRateYaw = 3, DRateYaw = 0;

// ALTITUDE HOLD (baro) -- START values, TUNE in flight
float ALT_MAX_CLIMB = 1.2;   // m/s at full throttle-stick deflection
float altP = 180;            // us of throttle per meter of altitude error
float altD = 120;            // us per (m/s) vertical speed (damps bobbing)
float altI = 25;             // us per (meter*second) -> finds the hover throttle
float ALT_HOVER = 1500;      // base throttle guess; altI trims the rest
//   Bobs up/down -> lower altP (and/or raise altD).  Sinks/climbs slowly -> raise altI.

const float t = 0.004;       // 250 Hz loop
uint32_t LoopTimer;
Servo mot1, mot2, mot3, mot4;

// ESP-NOW control packet (matches tx_s3_arduino)
typedef struct { uint16_t roll, pitch, throttle, yaw; uint8_t arm, altHold; } ControlPacket;
volatile int ReceiverValue[4] = {1500, 1500, 1000, 1500};
volatile bool armed = false, altHoldReq = false;
volatile uint32_t lastPacketTime = 0;

// IMU
float RateRoll, RatePitch, RateYaw;
float RateCalRoll = 0.936, RateCalPitch = 0.724, RateCalYaw = -0.564;
float AccX, AccY, AccZ, AccCalX = -0.027, AccCalY = -0.018, AccCalZ = 0.048;
float AngleRoll, AnglePitch, compAngleRoll = 0, compAnglePitch = 0;

// PID scratch
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float InputRoll, InputPitch, InputYaw, InputThrottle;
float PrevErrAngleRoll, PrevErrAnglePitch, PrevItermAngleRoll, PrevItermAnglePitch;
float PrevErrRateRoll, PrevErrRatePitch, PrevErrRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float M1, M2, M3, M4;

// BARO (BMP280 @0x76)
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
double pRef = 0;
float baroAlt = 0, baroAltPrev = 0, vario = 0;
uint32_t lastBaro = 0;
bool baroOK = false;
// althold runtime
float targetAlt = 0, altIterm = 0;
bool altHoldPrev = false;
uint32_t loopCount = 0, lastTelem = 0;

void handlePacket(const uint8_t *data, int len) {
  if (len != (int)sizeof(ControlPacket)) return;
  ControlPacket p; memcpy(&p, data, sizeof(p));
  ReceiverValue[0] = constrain((int)p.roll,  1000, 2000);
  ReceiverValue[1] = constrain((int)p.pitch, 1000, 2000);
  ReceiverValue[3] = constrain((int)p.yaw,   1000, 2000);
  ReceiverValue[2] = constrain((int)p.throttle, 1000, 2000);
  armed = (p.arm == 1);
  altHoldReq = (p.altHold == 1);
  lastPacketTime = millis();
}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t*, const uint8_t *d, int n) { handlePacket(d, n); }
#else
void onDataRecv(const uint8_t*, const uint8_t *d, int n) { handlePacket(d, n); }
#endif

void readIMU() {
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t ax = Wire.read() << 8 | Wire.read(), ay = Wire.read() << 8 | Wire.read(), az = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t gx = Wire.read() << 8 | Wire.read(), gy = Wire.read() << 8 | Wire.read(), gz = Wire.read() << 8 | Wire.read();
  RateRoll = (float)gx / 65.5 - RateCalRoll; RatePitch = (float)gy / 65.5 - RateCalPitch; RateYaw = (float)gz / 65.5 - RateCalYaw;
  AccX = (float)ax / 4096 - AccCalX; AccY = (float)ay / 4096 - AccCalY; AccZ = (float)az / 4096 - AccCalZ;
  AngleRoll  =  atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
}

float pid(float err, float P, float I, float D, float &prevErr, float &prevIterm) {
  float It = prevIterm + I * (err + prevErr) * (t / 2); It = constrain(It, -400, 400);
  float out = constrain(P * err + It + D * (err - prevErr) / t, -400, 400);
  prevErr = err; prevIterm = It; return out;
}
void resetIntegrators() {
  PrevErrRateRoll = PrevErrRatePitch = PrevErrRateYaw = 0; PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
  PrevErrAngleRoll = PrevErrAnglePitch = 0; PrevItermAngleRoll = PrevItermAnglePitch = 0;
}

// BMP280
void bWrite(uint8_t reg, uint8_t val) { Wire.beginTransmission(0x76); Wire.write(reg); Wire.write(val); Wire.endTransmission(); }
bool baroBegin() {
  Wire.beginTransmission(0x76); Wire.write(0xD0); Wire.endTransmission();
  Wire.requestFrom(0x76, 1); if (Wire.read() != 0x58) return false;   // BMP280 chip id
  bWrite(0xE0, 0xB6); delay(10);
  Wire.beginTransmission(0x76); Wire.write(0x88); Wire.endTransmission();
  Wire.requestFrom(0x76, 24); uint8_t b[24]; for (int i = 0; i < 24; i++) b[i] = Wire.read();
  dig_T1 = b[0] | (b[1] << 8); dig_T2 = b[2] | (b[3] << 8); dig_T3 = b[4] | (b[5] << 8);
  dig_P1 = b[6] | (b[7] << 8); dig_P2 = b[8] | (b[9] << 8); dig_P3 = b[10] | (b[11] << 8);
  dig_P4 = b[12] | (b[13] << 8); dig_P5 = b[14] | (b[15] << 8); dig_P6 = b[16] | (b[17] << 8);
  dig_P7 = b[18] | (b[19] << 8); dig_P8 = b[20] | (b[21] << 8); dig_P9 = b[22] | (b[23] << 8);
  bWrite(0xF5, 0x10); bWrite(0xF4, 0x37);   // IIR x16, temp x1 press x16 normal
  return true;
}
double baroPressurePa() {
  Wire.beginTransmission(0x76); Wire.write(0xF7); Wire.endTransmission();
  Wire.requestFrom(0x76, 6);
  uint8_t pm = Wire.read(), pl = Wire.read(), px = Wire.read(), tm = Wire.read(), tl = Wire.read(), tx = Wire.read();
  int32_t adc_P = ((uint32_t)pm << 12) | ((uint32_t)pl << 4) | (px >> 4);
  int32_t adc_T = ((uint32_t)tm << 12) | ((uint32_t)tl << 4) | (tx >> 4);
  double v1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * (double)dig_T2;
  double v2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * (double)dig_T3;
  double t_fine = v1 + v2;
  v1 = (t_fine / 2.0) - 64000.0; v2 = v1 * v1 * (double)dig_P6 / 32768.0; v2 = v2 + v1 * (double)dig_P5 * 2.0;
  v2 = (v2 / 4.0) + ((double)dig_P4 * 65536.0);
  v1 = ((double)dig_P3 * v1 * v1 / 524288.0 + (double)dig_P2 * v1) / 524288.0; v1 = (1.0 + v1 / 32768.0) * (double)dig_P1;
  if (v1 == 0) return 0;
  double p = 1048576.0 - (double)adc_P; p = (p - (v2 / 4096.0)) * 6250.0 / v1;
  v1 = (double)dig_P9 * p * p / 2147483648.0; v2 = p * (double)dig_P8 / 32768.0;
  return p + (v1 + v2 + (double)dig_P7) / 16.0;
}

void setup() {
  Serial.begin(115200); delay(200);
  Serial.println("\n[FC] BOOT -- Arduino flight controller + baro alt-hold");
  pinMode(15, OUTPUT);

  WiFi.mode(WIFI_STA); WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { while (true) { digitalWrite(15, !digitalRead(15)); delay(80); } }
  esp_now_register_recv_cb(onDataRecv);

  Wire.setClock(400000); Wire.begin(); delay(250);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();  // wake MPU

  baroOK = baroBegin();
  Serial.printf("[FC] baro %s\n", baroOK ? "OK (BMP280)" : "NOT FOUND -> alt-hold disabled");
  if (baroOK) { double s = 0; for (int i = 0; i < 20; i++) { s += baroPressurePa(); delay(20); } pRef = s / 20.0; }

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  mot1.attach(mot1_pin, 1000, 2000); mot1.setPeriodHertz(ESCfreq);
  mot2.attach(mot2_pin, 1000, 2000); mot2.setPeriodHertz(ESCfreq);
  mot3.attach(mot3_pin, 1000, 2000); mot3.setPeriodHertz(ESCfreq);
  mot4.attach(mot4_pin, 1000, 2000); mot4.setPeriodHertz(ESCfreq);
  mot1.writeMicroseconds(1000); mot2.writeMicroseconds(1000); mot3.writeMicroseconds(1000); mot4.writeMicroseconds(1000);
  delay(2000);
  LoopTimer = micros();
}

void loop() {
  if (millis() - lastPacketTime > FAILSAFE_MS) armed = false;   // link-loss failsafe

  readIMU();
  compAngleRoll  = constrain(0.991 * (compAngleRoll  + RateRoll  * t) + 0.009 * AngleRoll,  -20, 20);
  compAnglePitch = constrain(0.991 * (compAnglePitch + RatePitch * t) + 0.009 * AnglePitch, -20, 20);

  // ---- baro update (~30 Hz) ----
  if (baroOK && millis() - lastBaro >= 33) {
    lastBaro = millis();
    float altRaw = 44330.0f * (1.0f - pow(baroPressurePa() / pRef, 1.0f / 5.255f));
    baroAlt += 0.25f * (altRaw - baroAlt);                    // smooth altitude
    vario   += 0.10f * ((baroAlt - baroAltPrev) / 0.033f - vario);  // smooth vertical speed
    baroAltPrev = baroAlt;
  }

  // ---- attitude PID ----
  DesiredRateRoll  = pid(0.1 * (ReceiverValue[0] - 1500) - compAngleRoll,  PAngleRoll,  IAngleRoll,  DAngleRoll,  PrevErrAngleRoll,  PrevItermAngleRoll);
  DesiredRatePitch = pid(0.1 * (ReceiverValue[1] - 1500) - compAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrAnglePitch, PrevItermAnglePitch);
  DesiredRateYaw   = 0.15 * (ReceiverValue[3] - 1500);
  InputRoll  = pid(DesiredRateRoll  - RateRoll,  PRateRoll,  IRateRoll,  DRateRoll,  PrevErrRateRoll,  PrevItermRateRoll);
  InputPitch = pid(DesiredRatePitch - RatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrRatePitch, PrevItermRatePitch);
  InputYaw   = pid(DesiredRateYaw   - RateYaw,   PRateYaw,   IRateYaw,   DRateYaw,   PrevErrRateYaw,   PrevItermRateYaw);

  // ---- throttle: manual, or BARO ALTITUDE HOLD ----
  bool altActive = altHoldReq && armed && baroOK;
  if (altActive) {
    if (!altHoldPrev) { targetAlt = baroAlt; altIterm = 0; }         // just engaged -> hold here
    float dev = (float)ReceiverValue[2] - 1500.0f;
    float climbCmd = (fabs(dev) > 40) ? (dev / 500.0f) * ALT_MAX_CLIMB : 0.0f;  // stick -> climb rate
    targetAlt += climbCmd * t;                                       // stick moves the held altitude
    float altErr = targetAlt - baroAlt;
    altIterm = constrain(altIterm + altI * altErr * t, -250, 250);
    InputThrottle = constrain(ALT_HOVER + altP * altErr - altD * vario + altIterm, (float)ThrottleIdle, (float)THR_MAX_MIX);
  } else {
    InputThrottle = constrain((float)ReceiverValue[2], 1000, THR_MAX_MIX);
  }
  altHoldPrev = altActive;

  // ---- QUADX mix (M1=RR, M2=FR, M3=RL, M4=FL) ----
  float r = InputRoll * ROLL_DIR, p = InputPitch * PITCH_DIR, y = InputYaw * YAW_DIR;
  M1 = InputThrottle - r + p - y;  M2 = InputThrottle - r - p + y;
  M3 = InputThrottle + r + p + y;  M4 = InputThrottle + r - p - y;
  M1 = constrain(M1, ThrottleIdle, 1999); M2 = constrain(M2, ThrottleIdle, 1999);
  M3 = constrain(M3, ThrottleIdle, 1999); M4 = constrain(M4, ThrottleIdle, 1999);

  if (!armed || ReceiverValue[2] < 1030) {   // disarmed / throttle-down -> cut + reset
    M1 = M2 = M3 = M4 = ThrottleCutOff; resetIntegrators();
  }
  mot1.writeMicroseconds(M1); mot2.writeMicroseconds(M2); mot3.writeMicroseconds(M3); mot4.writeMicroseconds(M4);

  loopCount++;
  if (millis() - lastTelem >= 500) {
    Serial.printf("[FC] %-8s arm=%d alt=%s roll=%.1f pitch=%.1f | baroAlt=%.2f vario=%.2f | M %.0f %.0f %.0f %.0f\n",
                  (millis() - lastPacketTime > FAILSAFE_MS) ? "FAILSAFE" : "LINK-OK", armed,
                  altActive ? "HOLD" : "man", compAngleRoll, compAnglePitch, baroAlt, vario, M1, M2, M3, M4);
    loopCount = 0; lastTelem = millis();
  }

  while (micros() - LoopTimer < (t * 1000000)) { if ((long)(t * 1000000) - (long)(micros() - LoopTimer) > 1500) delay(1); }
  LoopTimer = micros();
}
