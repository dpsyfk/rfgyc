/* ============================================================================
   ROBOT A - FINAL MISSION  (Arduino UNO R4 WiFi + Cytron MDD10A + JGB37 motors)

   DEFAULT MOVEMENT (USE_WALL_MISSION = false), all from the calibrated motion engine
   (trapezoid profile + feed-forward + PI on both wheels, encoders only):
        1. forward   85 mm
        2. turn LEFT 90 deg   (wheel arc = angle x track / 2, slower turn speed)
        3. forward  380 mm
   After every step and at the end the odometry is printed in the calibration format:
        odom x=84.7 y=333.1 th=90.51deg
        counts L=3657 R=7275
   The gyro is NOT used for control in this mode (its reading is only printed for comparison).
   Set USE_WALL_MISSION = true to get the older mission: slow left turn + straight until the ToF reads ~280 mm.

   CALIBRATION (robot A, latest)
     cpr 1462.25 / 1457.55   corr 1.87264 / 1.87264   wheel 70 mm   track 187.01 mm
     dead 14.4 / 14.4   kv 0.87669 / 0.87769 (see the note at KV_L)   vmax 223.4 mm/s
     kp 3   ki 0.5   move v 150 mm/s  a 300 mm/s2   turn v 100 mm/s
   HOW THE CALIBRATION IS USED:  counts per mm = cpr / (pi x wheel) x corr   ->   mm per count = pi x wheel / (cpr x corr).
   (corr DIVIDES the mm-per-count. Multiplying would make 1 mm = 3.5 counts instead of ~12.4 and the robot would
   move only about a third of every commanded distance.)

   IF THE WHEELS DO NOT MOVE  (the problem this version is built to settle)
     Boot log line  [MOTORS] map ... source=default|saved   shows which PWM/DIR pins are used.
     Serial commands (115200), accepted during the countdown, after a fault, or when done - NEVER while moving:
        p = WIRING PROBE (robot ON BLOCKS!): tries every PWM/DIR pin combination on D5 D6 D9 D10, watches the encoders,
            finds which pins drive which wheel and in which direction, adopts that map and SAVES it in EEPROM.
            If no wheel produces a single encoder count with ANY combination it says so: that is a POWER problem
            (motor battery / MDD10A B+ B- / common GND / motor leads), not a pin map problem.
        e = raw encoder monitor (motors off): turn a wheel by hand, its counter must rise
        z = forget the saved motor map (back to the defaults in the sketch)
        x = emergency stop

   WIRING
     MDD10A  LEFT : PWM D10  DIR D9      RIGHT: PWM D5  DIR D6      common GND
     Encoder LEFT : A D3  B D7           RIGHT: A D2    B D4        VCC 5V
     I2C (A4 SDA / A5 SCL) -> TCA9548A (0x70)
        mux channel 0 : MPU-6050/6500 gyro   (SD0 / SC0)   (optional in the movement mode)
        mux channel 3 : VL53L1X front ToF    (SD3 / SC3)   XSHUT -> A0 (optional)

   LED MATRIX
     boot: 1..12 columns lit = boot stage reached (freeze = that stage hangs)
     running: row 3-4 = ToF bar (0..1200 mm), row 6 = heading error, row 7 = countdown
     DONE = square outline, FAULT = X
   SERIAL (115200): telemetry every 250 ms while moving. Send 'x' = emergency stop.
   ============================================================================ */

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_VL53L1X.h>
#include "Arduino_LED_Matrix.h"
#include <math.h>

// ============================================================================
// 1. HARDWARE  (pins / signs copied from the calibrated jgb37_wifi.ino)
// ============================================================================
constexpr uint8_t PIN_ENC_L_A = 3;     // D2 / D3 are the only interrupt pins on the UNO R4
constexpr uint8_t PIN_ENC_L_B = 7;
constexpr uint8_t PIN_ENC_R_A = 2;
constexpr uint8_t PIN_ENC_R_B = 4;

// DEFAULT motor map = the one of the calibrated jgb37_wifi.ino. The wiring probe ('p') can replace it; the result is kept in
// EEPROM and loaded at every boot (set USE_SAVED_MOTOR_MAP = false to ignore it, or send 'z' to erase it).
constexpr uint8_t DEF_L_PWM = 10, DEF_L_DIR = 9, DEF_R_PWM = 5, DEF_R_DIR = 6;
constexpr int8_t  DEF_L_SIGN = 1, DEF_R_SIGN = 1;      // +1: DIR HIGH moves the wheel forward (= encoder counts go positive)
constexpr bool    USE_SAVED_MOTOR_MAP = true;
uint8_t pinLPwm = DEF_L_PWM, pinLDir = DEF_L_DIR, pinRPwm = DEF_R_PWM, pinRDir = DEF_R_DIR;
int8_t  motorLSign = DEF_L_SIGN, motorRSign = DEF_R_SIGN;
const char *motorMapSource = "default";

constexpr int8_t ENC_L_SIGN   = -1;
constexpr int8_t ENC_R_SIGN   = 1;

constexpr uint8_t PIN_XSHUT_TOF = A0;

// I2C mux + sensors (updated connections)
constexpr uint8_t TCA_I2C_ADDR = 0x70;
constexpr uint8_t TCA_CH_GYRO  = 0;    // MPU  : SD0 / SC0
constexpr uint8_t TCA_CH_TOF   = 3;    // VL53L1X : SD3 / SC3
constexpr uint8_t TOF_ADDR     = 0x29;
constexpr uint8_t GYRO_ADDR    = 0x68;

// ============================================================================
// 2. CALIBRATION  (robot A, from the calibration page)
// ============================================================================
constexpr float WHEEL_DIAM_MM = 70.0f;
constexpr float TRACK_MM      = 187.01f;
constexpr float CPR_L = 1462.25f, CPR_R = 1457.55f;
constexpr float CORR_L = 1.87264f, CORR_R = 1.87264f;
constexpr float DEAD_L = 14.4f,   DEAD_R = 14.4f;
// kv = PWM per (mm/s). The calibration page was quoted as "8.7669 / 8.7769". That is almost certainly 0.87669 / 0.87769
// with the decimal point in the wrong place: 8.77 PWM per mm/s would need >1300 PWM at 150 mm/s (the maximum is 255),
// while 0.877 x vmax 223.4 = 196 PWM fits. readinessCheck() refuses to run if kv x vmax + dead exceeds PWM_MAX.
constexpr float KV_L = 0.87669f,  KV_R = 0.87769f;
constexpr float VMAX_MMPS = 223.4f;
constexpr float KP = 3.0f, KI = 0.5f;
// corr DIVIDES: counts per mm = cpr / (pi x wheel) x corr
constexpr float COUNTS_PER_MM_L = CPR_L / (PI * WHEEL_DIAM_MM) * CORR_L;
constexpr float COUNTS_PER_MM_R = CPR_R / (PI * WHEEL_DIAM_MM) * CORR_R;
constexpr float MM_PER_COUNT_L = 1.0f / COUNTS_PER_MM_L;
constexpr float MM_PER_COUNT_R = 1.0f / COUNTS_PER_MM_R;
constexpr int   PWM_MAX = 255;
constexpr unsigned long CONTROL_DT_MS = 10;
// Optional motor-battery guard: wire the battery (+) through a divider to an analog pin (e.g. 100k from B+ to A1, 33k from A1 to GND,
// ratio 4.03) and set BATTERY_SENSE_PIN = A1. -1 = not fitted. The robot then refuses to start below BATTERY_MIN_V.
constexpr int   BATTERY_SENSE_PIN = -1;
constexpr float BATTERY_DIV_RATIO = 4.03f;
constexpr float BATTERY_MIN_V     = 6.0f;
constexpr float MOVE_END_TOL_MM = 1.0f;       // a move ends when both wheels are within this of the target

// ============================================================================
// 3. MISSION SETTINGS  (the only numbers you normally touch)
// ============================================================================
// ---- the fixed movement (default) ----
constexpr bool  USE_WALL_MISSION = false;    // false = forward / left turn / forward (below).  true = old: turn, then straight to the ToF target
constexpr float SEQ_FWD1_MM   = 85.0f;       // 1. forward
constexpr float SEQ_TURN_DEG  = 90.0f;       // 2. turn left (positive = left)
constexpr float SEQ_FWD2_MM   = 380.0f;      // 3. forward
constexpr float MOVE_SPEED_MMPS  = 150.0f;   // forward speed (v 150)
constexpr float MOVE_ACCEL_MMPS2 = 300.0f;   // acceleration / deceleration (a 300)
constexpr unsigned long SEQ_PAUSE_MS = 300;  // stand still between the steps and before the final report
constexpr float TOF_SAFETY_STOP_MM = 0.0f;   // >0: abort a forward step if the filtered ToF reads less than this (0 = off; the ToF is only printed)

constexpr float LEFT_TURN_DEG     = 90.0f;   // left turn angle for the wall mission (positive = left)
constexpr float TURN_SPEED_MMPS   = 100.0f;  // turn speed tv 100: wheel speed (body = 2 x 100 / 187 rad/s = ~61 deg/s)
constexpr float TURN_ACCEL_MMPS2  = 300.0f;
constexpr float TURN_TRIM_TOL_DEG = 2.5f;    // gyro says turn was off by more than this -> small corrective turn
constexpr float TURN_TRIM_MAX_DEG = 30.0f;
constexpr uint8_t TURN_TRIM_MAX_COUNT = 2;

constexpr float WALL_TARGET_MM    = 280.0f;  // stop when the ToF reads this
constexpr float WALL_TOL_MM       = 15.0f;   // accepted final window: target +/- this
constexpr float CRUISE_MMPS       = 120.0f;  // straight-line speed
constexpr float APPROACH_MIN_MMPS = 35.0f;   // creep speed near the target
constexpr float APPROACH_ACCEL_MMPS2 = 150.0f;
constexpr float APPROACH_DECEL_MMPS2 = 150.0f;   // planned braking (sets where the slow-down starts)
constexpr float APPROACH_STOP_DECEL_MMPS2 = 500.0f;
constexpr float APPROACH_LEAD_MM  = 4.0f;    // stop this much before the target (coast)
constexpr float APPROACH_LATENCY_S = 0.12f;  // + speed x this (ToF filter lag)
constexpr uint8_t WALL_CONFIRM_COUNT = 3;    // consecutive NEW filtered samples at/below the trigger
constexpr float APPROACH_MAX_ERR_MM = 40.0f; // clamp of the position error (anti wind-up)
constexpr float NUDGE_SPEED_MMPS  = 40.0f;
constexpr uint8_t NUDGE_MAX_COUNT = 2;
constexpr bool  ALLOW_REVERSE_NUDGE = false; // false = if it overshoots, just report it

constexpr bool  USE_GYRO_HEADING_HOLD = true;
constexpr float HEADING_K   = 2.5f;          // 1/s : heading error -> wheel offset
constexpr float HEADING_OFF_MAX_MM = 30.0f;

constexpr unsigned long BOOT_DELAY_MS   = 5000;  // stationary countdown
constexpr unsigned long GYRO_RECAL_LEAD_MS = 1800; // gyro bias re-measured this long before the countdown ends
constexpr unsigned long SETTLE_MS       = 500;
constexpr unsigned long WALL_SEE_TIMEOUT_MS = 2000;  // after the turn the ToF must show a wall within this
constexpr unsigned long APPROACH_TOF_LOSS_MS = 600;
constexpr unsigned long APPROACH_TIMEOUT_BASE_MS = 8000;
constexpr unsigned long APPROACH_TIMEOUT_MS_PER_MM = 40;
constexpr unsigned long STALL_MS = 800;

// ---- ToF / gyro low level ----
constexpr uint16_t TOF_MIN_MM = 30;
constexpr uint16_t TOF_MAX_MM = 4000;
constexpr uint16_t TOF_TIMING_BUDGET_MS = 33;
constexpr uint16_t TOF_INTERMEASURE_MS  = 35;
constexpr unsigned long TOF_POLL_MS = 8;
constexpr uint8_t  TOF_FILTER_N   = 5;
constexpr uint8_t  TOF_FILTER_MIN = 3;
constexpr unsigned long TOF_STALE_MS = 300;

constexpr float GYRO_LSB_PER_DPS = 65.5f;      // +/-500 dps
constexpr unsigned long GYRO_INTERVAL_MS = 5;
constexpr float GYRO_SIGN_GUESS = -1.0f;       // only a first guess: the first turn measures the real sign
constexpr float GYRO_MIN_TURN_DEG = 30.0f;     // gyro must see at least this much of the turn to be trusted
constexpr int   GYRO_CAL_SAMPLES = 300;
constexpr uint8_t GYRO_CAL_ATTEMPTS = 5;
constexpr float GYRO_CAL_OUTLIER = 120.0f;
constexpr int   GYRO_CAL_MAX_OUTLIER_PCT = 5;
constexpr float GYRO_CAL_MAX_STD = 30.0f;
constexpr float GYRO_CAL_MAX_HALF_DIFF = 40.0f;

// ============================================================================
// 4. STATE
// ============================================================================
enum State : uint8_t {
  S_BOOT, S_READY_CHECK, S_COUNTDOWN, S_TURN, S_SETTLE, S_APPROACH,
  S_FINAL_SETTLE, S_NUDGE, S_SEQ_MOVE, S_SEQ_PAUSE, S_DONE, S_FAULT, S_BENCH
};

enum SeqKind : uint8_t { SK_FWD, SK_TURN };
struct SeqStep { SeqKind kind; float value; };
const char *stateName(State s) {
  switch (s) {
    case S_BOOT: return "BOOT";               case S_READY_CHECK: return "READY_CHECK";
    case S_COUNTDOWN: return "COUNTDOWN";     case S_TURN: return "LEFT_TURN";
    case S_SETTLE: return "SETTLE";           case S_APPROACH: return "WALL_APPROACH";
    case S_FINAL_SETTLE: return "FINAL_SETTLE"; case S_NUDGE: return "NUDGE";
    case S_SEQ_MOVE: return "SEQ_MOVE";       case S_SEQ_PAUSE: return "SEQ_PAUSE";
    case S_DONE: return "DONE";               case S_FAULT: return "FAULT";
    case S_BENCH: return "BENCH";
  }
  return "?";
}
State state = S_BOOT;
unsigned long stateMs = 0;
String faultReason;

ArduinoLEDMatrix matrix;
uint8_t ledFrame[8][12];
Adafruit_VL53L1X tof;

// mux
uint8_t tcaCurrent = 0xFF;
uint8_t tofChannel = TCA_CH_TOF;

// gyro
bool  gyroOnline = false, gyroTrusted = false;
float gyroBiasZ = 0, gyroRateDps = 0, rawHeadingDeg = 0, gyroSign = GYRO_SIGN_GUESS;
unsigned long lastGyroUs = 0, lastGyroReadMs = 0;
uint8_t gyroFailCount = 0;
inline float headingDeg() { return gyroSign * rawHeadingDeg; }

// ToF
bool tofOnline = false, tofValid = false;
int16_t  tofRawMm = 0;
uint16_t tofFiltMm = 0, tofHist[TOF_FILTER_N];
uint8_t  tofHistN = 0, tofHistPos = 0, tofFailCount = 0;
uint32_t tofFiltSeq = 0, tofValidCount = 0, tofInvalidCount = 0;
unsigned long lastTofPollMs = 0, tofLastValidMs = 0;
bool tofTuneEnabled = true;
uint8_t i2cRecoveries = 0;

// motors / encoders
volatile long cntL = 0, cntR = 0;
float lastUL = 0, lastUR = 0;

// motion engine (calibrated profile + PI)
float pD, pV, pA, pTa, pTc, pT;
bool mAct = false, moveAborted = false;
String moveAbortReason;
float mFL, mFR, mIL, mIR;
long mSL, mSR, mLastCL, mLastCR;
unsigned long mT0, mTick, mMovL, mMovR;

// turn
float turnStartRaw = 0;
uint8_t trimCount = 0;
unsigned long turnEvalAt = 0;
bool turnWaitingEval = false;

// wall approach
float apS = 0, apV = 0, apIL = 0, apIR = 0, hOff = 0, holdHeading = 0, d0Mm = 0;
long apStartL = 0, apStartR = 0, apLastCL = 0, apLastCR = 0;
unsigned long apTick = 0, apT0 = 0, apLastFresh = 0, apMovL = 0, apMovR = 0, apTimeout = 0;
uint32_t apLastSeq = 0;
uint8_t apConfirm = 0;
bool apStopping = false;

uint8_t nudgeCount = 0;
float finalDistMm = 0;

// odometry (differential drive, from the encoders): x forward at the start, y to the left, th = left turn positive
float odomX = 0, odomY = 0, odomTh = 0;
long  odomLastL = 0, odomLastR = 0;

// fixed movement sequence
const SeqStep SEQ[] = { {SK_FWD, SEQ_FWD1_MM}, {SK_TURN, SEQ_TURN_DEG}, {SK_FWD, SEQ_FWD2_MM} };
constexpr uint8_t SEQ_N = sizeof(SEQ) / sizeof(SEQ[0]);
uint8_t seqIdx = 0;
float seqGyroStartRaw = 0;

unsigned long lastTelemetryMs = 0, lastMatrixMs = 0;
bool recalDone = false;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ============================================================================
// 5. BOOT PROGRESS ON THE LED MATRIX
// ============================================================================
void showStage(uint8_t n) {
  memset(ledFrame, 0, sizeof(ledFrame));
  for (uint8_t c = 0; c < n && c < 12; c++)
    for (uint8_t r = 0; r < 8; r++) ledFrame[r][c] = 1;
  matrix.renderBitmap(ledFrame, 8, 12);
}

// ============================================================================
// 6. MOTORS + ENCODERS (calibrated sketch)
// ============================================================================
// A wheel never changes direction while it is being driven: it must pass through zero output and stay there for
// REVERSE_NEUTRAL_MS first, and the DIR pin is only written when PWM is about to be non-zero (MDD10A protection).
constexpr unsigned long REVERSE_NEUTRAL_MS = 60;
struct DirGuard { int8_t dir = 0; unsigned long zeroSince = 0; };
DirGuard guardL, guardR;
void driveMotor(uint8_t pwmPin, uint8_t dirPin, int8_t sign, float cmd, DirGuard &g) {
  int p = (int)clampf(cmd * sign, -PWM_MAX, PWM_MAX);
  int8_t want = p > 0 ? 1 : (p < 0 ? -1 : 0);
  unsigned long now = millis();
  if (want != 0 && g.dir != 0 && want != g.dir) {              // reversal requested: only after the neutral time at zero output
    if (g.zeroSince == 0) g.zeroSince = now;
    if (now - g.zeroSince >= REVERSE_NEUTRAL_MS) { g.dir = want; g.zeroSince = 0; }
    else { p = 0; want = 0; }
  }
  if (want == 0) {
    analogWrite(pwmPin, 0);                                    // PWM first, DIR untouched
    if (g.zeroSince == 0) g.zeroSince = now;
    return;
  }
  g.dir = want; g.zeroSince = 0;
  digitalWrite(dirPin, p > 0 ? HIGH : LOW);                    // DIR before PWM, output is zero or same direction here
  analogWrite(pwmPin, abs(p));
}
void setMotors(float l, float r) {
  lastUL = l; lastUR = r;
  driveMotor(pinLPwm, pinLDir, motorLSign, l, guardL);
  driveMotor(pinRPwm, pinRDir, motorRSign, r, guardR);
}
void stopMotors() { setMotors(0, 0); }

void isrL() {
  bool a = digitalRead(PIN_ENC_L_A), b = digitalRead(PIN_ENC_L_B);
  cntL += ((a == b) ? 1 : -1) * ENC_L_SIGN;
}
void isrR() {
  bool a = digitalRead(PIN_ENC_R_A), b = digitalRead(PIN_ENC_R_B);
  cntR += ((a == b) ? 1 : -1) * ENC_R_SIGN;
}
long getL() { noInterrupts(); long v = cntL; interrupts(); return v; }
long getR() { noInterrupts(); long v = cntR; interrupts(); return v; }

// ============================================================================
// 7. I2C MUX, GYRO, ToF
// ============================================================================
bool tcaSelect(uint8_t ch) {
  if (ch > 7) return false;
  if (tcaCurrent == ch) return true;
  Wire.beginTransmission(TCA_I2C_ADDR);
  Wire.write((uint8_t)(1u << ch));
  if (Wire.endTransmission() == 0) { tcaCurrent = ch; return true; }
  tcaCurrent = 0xFF;
  return false;
}
void tcaDisableAll() {
  Wire.beginTransmission(TCA_I2C_ADDR);
  Wire.write((uint8_t)0);
  Wire.endTransmission();
  tcaCurrent = 0xFF;
}
bool checkI2CPresence(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ---- gyro ----
bool gyroWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool gyroReadBytes(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)GYRO_ADDR, (size_t)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
bool gyroReadRawZ(int16_t &gz) {
  if (!tcaSelect(TCA_CH_GYRO)) return false;
  uint8_t b[2];
  if (!gyroReadBytes(0x47, b, 2)) { tcaCurrent = 0xFF; return false; }
  gz = (int16_t)((b[0] << 8) | b[1]);
  return true;
}
bool gyroCalibrate() {
  static int16_t buf[GYRO_CAL_SAMPLES];
  for (uint8_t attempt = 0; attempt < GYRO_CAL_ATTEMPTS; attempt++) {
    int good = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
      int16_t gz;
      if (gyroReadRawZ(gz)) buf[good++] = gz;
      delay(3);
    }
    if (good < (GYRO_CAL_SAMPLES * 5) / 6) { Serial.println(F("[GYRO] calibration: too many failed reads, retrying")); continue; }
    float mean = 0;
    for (int i = 0; i < good; i++) mean += buf[i];
    mean /= good;
    for (uint8_t pass = 0; pass < 2; pass++) {
      float s1 = 0; int n = 0;
      for (int i = 0; i < good; i++) if (fabsf((float)buf[i] - mean) <= GYRO_CAL_OUTLIER) { s1 += buf[i]; n++; }
      if (n == 0) break;
      mean = s1 / n;
    }
    int n = 0, outliers = 0;
    float var = 0, h1 = 0, h2 = 0; int n1 = 0, n2 = 0;
    for (int i = 0; i < good; i++) {
      float d = (float)buf[i] - mean;
      if (fabsf(d) > GYRO_CAL_OUTLIER) { outliers++; continue; }
      n++; var += d * d;
      if (i < good / 2) { h1 += buf[i]; n1++; } else { h2 += buf[i]; n2++; }
    }
    if (n < good / 2) { Serial.println(F("[GYRO] calibration: robot is moving, retrying")); continue; }
    float sd = sqrtf(var / n);
    float halfDiff = (n1 > 0 && n2 > 0) ? fabsf(h1 / n1 - h2 / n2) : 0.0f;
    if (outliers * 100 > good * GYRO_CAL_MAX_OUTLIER_PCT || sd > GYRO_CAL_MAX_STD || halfDiff > GYRO_CAL_MAX_HALF_DIFF) {
      Serial.print(F("[GYRO] calibration: not still enough (std=")); Serial.print(sd, 1);
      Serial.println(F("), retrying - keep the robot still"));
      continue;
    }
    gyroBiasZ = mean;
    gyroRateDps = 0.0f;
    lastGyroUs = micros();
    lastGyroReadMs = millis();
    Serial.print(F("[GYRO] bias Z = ")); Serial.print(gyroBiasZ, 1);
    Serial.print(F(" counts (std=")); Serial.print(sd, 1); Serial.println(')');
    return true;
  }
  lastGyroUs = micros();
  return false;
}
bool gyroBegin() {
  if (!tcaSelect(TCA_CH_GYRO)) { Serial.println(F("[BOOT] Gyro: mux channel select failed")); return false; }
  delay(5);
  if (!checkI2CPresence(GYRO_ADDR)) {
    Serial.print(F("[BOOT] Gyro: nothing at 0x")); Serial.print(GYRO_ADDR, HEX);
    Serial.print(F(" on TCA channel ")); Serial.println(TCA_CH_GYRO);
    return false;
  }
  uint8_t who = 0;
  if (!gyroReadBytes(0x75, &who, 1)) return false;
  if (!gyroWriteReg(0x6B, 0x01)) return false;   // wake, PLL clock
  delay(50);
  gyroWriteReg(0x1A, 0x03);                      // DLPF ~42 Hz
  gyroWriteReg(0x19, 0x04);                      // 200 Hz
  gyroWriteReg(0x1B, 0x08);                      // +/-500 dps
  delay(20);
  Serial.print(F("[BOOT] Gyro WHO_AM_I=0x")); Serial.println(who, HEX);
  return gyroCalibrate();
}

// ---- ToF register helpers ----
static bool tofReadReg16(uint16_t reg, uint16_t &val) {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write((uint8_t)(reg >> 8)); Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)TOF_ADDR, (size_t)2) != 2) return false;
  uint8_t h = Wire.read(), l = Wire.read();
  val = ((uint16_t)h << 8) | l;
  return true;
}
static bool tofSetInterMeasurementMs(uint16_t ms) {
  uint16_t osc = 0;
  if (!tofReadReg16(0x00DE, osc)) return false;
  osc &= 0x03FF;
  if (osc == 0) return false;
  uint32_t v = (uint32_t)((float)osc * (float)ms * 1.075f);
  Wire.beginTransmission(TOF_ADDR);
  Wire.write((uint8_t)0x00); Wire.write((uint8_t)0x6C);
  Wire.write((uint8_t)(v >> 24)); Wire.write((uint8_t)(v >> 16));
  Wire.write((uint8_t)(v >> 8));  Wire.write((uint8_t)v);
  return Wire.endTransmission() == 0;
}
void tofFilterFlush() { tofHistN = 0; tofHistPos = 0; }
bool tofFilteredFresh(unsigned long now) {
  return tofOnline && tofHistN >= TOF_FILTER_MIN && (now - tofLastValidMs) <= TOF_STALE_MS;
}
static uint16_t tofMedianOfHistory() {
  uint16_t t[TOF_FILTER_N];
  for (uint8_t i = 0; i < tofHistN; i++) t[i] = tofHist[i];
  for (uint8_t i = 1; i < tofHistN; i++) {
    uint16_t v = t[i]; int8_t j = (int8_t)i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return t[tofHistN / 2];
}

bool i2cRecover(const char *who) {
  stopMotors();
  if (i2cRecoveries >= 8) { Serial.println(F("[I2C] too many bus recoveries, giving up")); return false; }
  i2cRecoveries++;
  Serial.print(F("[I2C] bus error seen by ")); Serial.print(who);
  Serial.print(F(" - recovery #")); Serial.println(i2cRecoveries);
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Wire.end();
#if defined(PIN_WIRE_SDA) && defined(PIN_WIRE_SCL)
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE_SCL, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
      digitalWrite(PIN_WIRE_SCL, LOW);  delayMicroseconds(6);
      digitalWrite(PIN_WIRE_SCL, HIGH); delayMicroseconds(6);
    }
    pinMode(PIN_WIRE_SDA, OUTPUT); digitalWrite(PIN_WIRE_SDA, LOW); delayMicroseconds(6);
    digitalWrite(PIN_WIRE_SCL, HIGH); delayMicroseconds(6);
    digitalWrite(PIN_WIRE_SDA, HIGH); delayMicroseconds(6);
#endif
    Wire.begin();
    Wire.setClock(100000);
    tcaCurrent = 0xFF;
    delay(5);
    bool ok = true;
    if (gyroOnline && tcaSelect(TCA_CH_GYRO)) {
      gyroWriteReg(0x6B, 0x01); delay(5);
      gyroWriteReg(0x1A, 0x03); gyroWriteReg(0x19, 0x04); gyroWriteReg(0x1B, 0x08);
      int16_t gz; ok = gyroReadRawZ(gz);
    }
    if (ok && tofOnline) { ok = tcaSelect(tofChannel); if (ok) tof.startRanging(); }
    lastGyroUs = micros(); lastGyroReadMs = millis(); lastTofPollMs = millis();
    if (ok) { gyroFailCount = 0; tofFailCount = 0; Serial.println(F("[I2C] bus recovered")); return true; }
    delay(20);
  }
  Serial.println(F("[I2C] recovery FAILED"));
  return false;
}

void gyroUpdate(unsigned long nowMs) {
  if (!gyroOnline || nowMs - lastGyroReadMs < GYRO_INTERVAL_MS) return;
  lastGyroReadMs = nowMs;
  int16_t gz;
  unsigned long nowUs = micros();
  if (!gyroReadRawZ(gz)) {
    if (++gyroFailCount >= 5) {
      gyroFailCount = 0;
      if (!i2cRecover("gyro")) {
        gyroOnline = false; gyroTrusted = false;
        Serial.println(F("GYRO LOST: heading hold disabled"));
      }
    }
    return;
  }
  gyroFailCount = 0;
  float dt = (nowUs - lastGyroUs) * 1e-6f;
  lastGyroUs = nowUs;
  if (dt > 0.2f) dt = 0.2f;
  float rate = ((float)gz - gyroBiasZ) / GYRO_LSB_PER_DPS;    // raw sign; gyroSign is applied in headingDeg()
  if (fabsf(rate) < 0.3f) rate = 0.0f;
  gyroRateDps = rate;
  rawHeadingDeg += rate * dt;
}

void tofUpdate(unsigned long now) {
  if (!tofOnline || now - lastTofPollMs < TOF_POLL_MS) return;
  lastTofPollMs = now;
  if (!tcaSelect(tofChannel)) {
    if (++tofFailCount >= 5) {
      tofFailCount = 0;
      if (!i2cRecover("ToF")) { tofOnline = false; tofValid = false; Serial.println(F("TOF LOST")); }
    }
    return;
  }
  tofFailCount = 0;
  if (!tof.dataReady()) return;
  int16_t d = tof.distance();
  tof.clearInterrupt();
  tofRawMm = d;
  if (d >= (int16_t)TOF_MIN_MM && d <= (int16_t)TOF_MAX_MM) {
    tofValid = true;
    tofValidCount++;
    if (tofHistN > 0 && (now - tofLastValidMs) > TOF_STALE_MS) tofFilterFlush();
    if (tofHistN < TOF_FILTER_N) { tofHist[tofHistN++] = (uint16_t)d; tofHistPos = tofHistN % TOF_FILTER_N; }
    else { tofHist[tofHistPos] = (uint16_t)d; tofHistPos = (tofHistPos + 1) % TOF_FILTER_N; }
    tofFiltMm = tofMedianOfHistory();
    tofLastValidMs = now;
    tofFiltSeq++;
  } else {
    tofValid = false;
    tofInvalidCount++;
  }
}

void i2cScanReport() {
  Serial.println(F("[I2C] scan of every mux channel:"));
  for (uint8_t ch = 0; ch < 8; ch++) {
    if (!tcaSelect(ch)) continue;
    Serial.print(F("  ch")); Serial.print(ch); Serial.print(F(":"));
    bool any = false;
    for (uint8_t a = 0x08; a < 0x78; a++) {
      if (a == TCA_I2C_ADDR) continue;
      if (checkI2CPresence(a)) { Serial.print(F(" 0x")); Serial.print(a, HEX); any = true; }
    }
    if (!any) Serial.print(F(" (nothing)"));
    Serial.println();
  }
  tcaDisableAll();
}

bool tofBegin() {
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    bool useReset = (attempt < 3);
    Serial.print(F("[TOF] attempt ")); Serial.print(attempt); Serial.println(useReset ? F(" (XSHUT reset)") : F(" (no reset)"));
    if (useReset) { pinMode(PIN_XSHUT_TOF, OUTPUT); digitalWrite(PIN_XSHUT_TOF, LOW); }
    tcaDisableAll();
    delay(40);
    bool muxOk = tcaSelect(tofChannel);
    delay(10);
    if (useReset) pinMode(PIN_XSHUT_TOF, INPUT);
    delay(60);
    bool present = muxOk && checkI2CPresence(TOF_ADDR);
    if (muxOk && !present) {
      for (uint8_t ch = 0; ch < 8 && !present; ch++) {
        if (ch == tofChannel) continue;
        if (tcaSelect(ch) && checkI2CPresence(TOF_ADDR)) {
          Serial.print(F("[BOOT] ToF found on mux channel ")); Serial.print(ch);
          Serial.print(F(" (TCA_CH_TOF says ")); Serial.print(tofChannel); Serial.println(F(") - using it"));
          tofChannel = ch; present = true;
        }
      }
      if (!present) tcaSelect(tofChannel);
    }
    if (present) {
      uint16_t modelId = 0;
      bool idOk = tofReadReg16(0x010F, modelId);          // expect 0xEACC
      Serial.print(F("[TOF] model ID ")); Serial.print(idOk ? F("ok = 0x") : F("FAILED = 0x")); Serial.println(modelId, HEX);
      if (!idOk || modelId == 0x0000 || modelId == 0xFFFF) present = false;
    }
    if (present && tof.begin(TOF_ADDR, &Wire)) {
      Wire.setClock(100000);
      tcaCurrent = 0xFF; tcaSelect(tofChannel);
      tof.setTimingBudget(TOF_TIMING_BUDGET_MS);
      bool tuned = tofTuneEnabled && tofSetInterMeasurementMs(TOF_INTERMEASURE_MS);
      if (tofTuneEnabled && !tuned) Serial.println(F("[BOOT] ToF fast-mode write FAILED: default ~10 Hz"));
      if (tof.startRanging()) {
        if (tuned) {
          uint8_t n = 0;
          unsigned long t0 = millis();
          while (millis() - t0 < 600) {
            if (tof.dataReady()) { tof.distance(); tof.clearInterrupt(); n++; }
            delay(2);
          }
          Serial.print(F("[BOOT] ToF fast mode: ")); Serial.print(n); Serial.println(F(" samples in 0.6 s"));
          if (n < 11) { tofTuneEnabled = false; delay(100); continue; }   // fast mode did not work -> re-init with defaults
        }
        tofValid = false; tofFilterFlush(); tofFailCount = 0;
        Serial.print(F("[BOOT] ToF mux channel = ")); Serial.print(tofChannel);
        Serial.println(tofChannel == TCA_CH_TOF ? F(" (as configured)") : F(" (DIFFERENT from TCA_CH_TOF)"));
        return true;
      }
    }
    Serial.print(F("[BOOT] ToF init attempt ")); Serial.print(attempt);
    if (!muxOk) Serial.println(F(" failed (mux did not answer)"));
    else if (!present) Serial.println(F(" failed (no VL53L1X at 0x29)"));
    else Serial.println(F(" failed (library init/start failed)"));
    delay(150);
  }
  i2cScanReport();
  Serial.println(F("[BOOT] ToF checklist: VIN/GND, SDA->SD3, SCL->SC3, XSHUT, loose wires"));
  return false;
}

// ============================================================================
// 8. MOTION ENGINE (trapezoid profile + feed-forward + PI, calibrated)
// ============================================================================
void profileInit(float D, float vmax, float acc) {
  pD = D; pA = acc; pV = vmax;
  if (pV * pV / acc > D) pV = sqrtf(D * acc);
  pTa = pV / acc;
  pTc = (D - pV * pTa) / pV;
  if (pTc < 0) pTc = 0;
  pT = 2 * pTa + pTc;
}
void profileAt(float t, float &s, float &v) {
  if (t >= pT) { s = pD; v = 0; return; }
  if (t < pTa) { s = 0.5f * pA * t * t; v = pA * t; }
  else if (t < pTa + pTc) { s = 0.5f * pA * pTa * pTa + pV * (t - pTa); v = pV; }
  else {
    float td = t - pTa - pTc;
    s = 0.5f * pA * pTa * pTa + pV * pTc + pV * td - 0.5f * pA * td * td;
    v = pV - pA * td;
  }
}
void startMove(float dL, float dR, float v, float acc) {
  float D = max(fabsf(dL), fabsf(dR));
  if (D < 0.01f) return;
  mFL = dL / D; mFR = dR / D;
  profileInit(D, min(v, VMAX_MMPS), acc);
  mSL = getL(); mSR = getR();
  mLastCL = mSL; mLastCR = mSR;
  mIL = mIR = 0;
  mT0 = mTick = mMovL = mMovR = millis();
  mAct = true;
  moveAborted = false;
}
void abortMove(const String &why) {
  mAct = false;
  stopMotors();
  moveAborted = true;
  moveAbortReason = why;
}
void motionTick() {
  if (!mAct) return;
  unsigned long now = millis();
  if (now - mTick < CONTROL_DT_MS) return;
  float dt = (now - mTick) / 1000.0f;
  mTick = now;
  float t = (now - mT0) / 1000.0f;

  float s, v; profileAt(t, s, v);
  long cl = getL(), cr = getR();
  float posL = (cl - mSL) * MM_PER_COUNT_L;
  float posR = (cr - mSR) * MM_PER_COUNT_R;
  float eL = s * mFL - posL, eR = s * mFR - posR;
  mIL = clampf(mIL + eL * dt, -50, 50);
  mIR = clampf(mIR + eR * dt, -50, 50);

  float uL = KV_L * v * mFL + KP * eL + KI * mIL;
  float uR = KV_R * v * mFR + KP * eR + KI * mIR;
  if (fabsf(uL) > 1) uL += (uL > 0 ? DEAD_L : -DEAD_L);
  if (fabsf(uR) > 1) uR += (uR > 0 ? DEAD_R : -DEAD_R);
  setMotors(uL, uR);

  if (cl != mLastCL) { mLastCL = cl; mMovL = now; }
  if (cr != mLastCR) { mLastCR = cr; mMovR = now; }
  bool stallL = (fabsf(uL) > 60 && now - mMovL > STALL_MS), stallR = (fabsf(uR) > 60 && now - mMovR > STALL_MS);
  if (stallL || stallR) {
    String why = String("no encoder counts on ") + (stallL ? "LEFT " : "") + (stallR ? "RIGHT " : "") + "wheel while driven at PWM "
                 + String((int)max(fabsf(uL), fabsf(uR))) + " -> check: MOTOR BATTERY on + MDD10A B+/B- + common GND, motor leads, PWM/DIR wires, "
                 + "encoder power/wires. Send 'p' (wiring probe, robot on blocks) and 'e' (encoder monitor)";
    abortMove(why);
    return;
  }
  if (t > 0.3f) {                                                    // a wheel running the wrong way would make the PI loop run away
    if (mFL != 0 && posL * mFL < -15.0f) { abortMove("LEFT wheel moves OPPOSITE to the command (motor direction sign wrong) -> send 'p'"); return; }
    if (mFR != 0 && posR * mFR < -15.0f) { abortMove("RIGHT wheel moves OPPOSITE to the command (motor direction sign wrong) -> send 'p'"); return; }
  }
  if (t > pT && fabsf(eL) < MOVE_END_TOL_MM && fabsf(eR) < MOVE_END_TOL_MM) { stopMotors(); mAct = false; }
  else if (t > pT + 2.0f) { stopMotors(); mAct = false; }
}

// ============================================================================
// 9. MISSION
// ============================================================================
void enterState(State s) {
  state = s;
  stateMs = millis();
  Serial.print(F("[STATE] ")); Serial.println(stateName(s));
}
void fault(const String &why) {
  stopMotors();
  mAct = false;
  faultReason = why;
  Serial.print(F("[FAULT] ")); Serial.println(why);
  enterState(S_FAULT);
}

float avgTravelSinceApproach() {
  float l = (getL() - apStartL) * MM_PER_COUNT_L;
  float r = (getR() - apStartR) * MM_PER_COUNT_R;
  return (l + r) * 0.5f;
}

// ---- left turn ----
void startTurnMove(float deg) {
  float arc = (deg * PI / 180.0f) * TRACK_MM * 0.5f;      // +deg = left: left wheel back, right wheel forward
  startMove(-arc, arc, TURN_SPEED_MMPS, TURN_ACCEL_MMPS2);
}
void startTurn() {
  turnStartRaw = rawHeadingDeg;
  trimCount = 0;
  turnWaitingEval = false;
  Serial.print(F("[TURN] slow left turn ")); Serial.print(LEFT_TURN_DEG, 0);
  Serial.print(F(" deg at ")); Serial.print(TURN_SPEED_MMPS, 0); Serial.println(F(" mm/s"));
  enterState(S_TURN);
  startTurnMove(LEFT_TURN_DEG);
  if (!mAct) fault("turn move did not start");
}
void updateTurn(unsigned long now) {
  motionTick();
  if (moveAborted) { fault(String("TURN: ") + moveAbortReason); return; }
  if (mAct) return;
  if (!turnWaitingEval) { turnWaitingEval = true; turnEvalAt = now + 350; return; }   // let the robot and gyro settle
  if (now < turnEvalAt) return;
  turnWaitingEval = false;

  float rawDelta = rawHeadingDeg - turnStartRaw;
  bool seen = gyroOnline && fabsf(rawDelta) >= GYRO_MIN_TURN_DEG && fabsf(rawDelta) <= 170.0f;
  if (!seen) {
    gyroTrusted = false;
    Serial.print(F("[TURN] gyro saw ")); Serial.print(rawDelta, 1);
    Serial.println(F(" deg raw -> NOT trusted: no trim, no heading hold (encoder turn only)"));
  } else {
    float s = (rawDelta > 0) ? 1.0f : -1.0f;               // the commanded turn IS a left turn, so this sign = "left"
    if (s != gyroSign) Serial.println(F("[TURN] gyro sign corrected from the measured left turn"));
    gyroSign = s;
    gyroTrusted = true;
    float turned = fabsf(rawDelta);
    float err = LEFT_TURN_DEG - turned;
    Serial.print(F("[TURN] gyro measured ")); Serial.print(turned, 1);
    Serial.print(F(" deg, error ")); Serial.print(err, 1); Serial.println(F(" deg"));
    if (fabsf(err) > TURN_TRIM_TOL_DEG && fabsf(err) <= TURN_TRIM_MAX_DEG && trimCount < TURN_TRIM_MAX_COUNT) {
      trimCount++;
      Serial.print(F("[TURN] trim #")); Serial.print(trimCount); Serial.print(F(": ")); Serial.print(err, 1); Serial.println(F(" deg"));
      startTurnMove(err);                                   // turnStartRaw is kept: the next check measures the TOTAL turn
      if (mAct) return;
    }
  }
  holdHeading = headingDeg();
  Serial.print(F("[TURN] done, heading ")); Serial.println(holdHeading, 1);
  tofFilterFlush();
  enterState(S_SETTLE);
}

// ---- settle after the turn, then start the straight approach ----
void startApproach(unsigned long now) {
  d0Mm = tofFiltMm;
  apS = apV = apIL = apIR = hOff = 0;
  apStartL = apLastCL = getL(); apStartR = apLastCR = getR();
  apTick = apT0 = apLastFresh = apMovL = apMovR = now;
  apLastSeq = tofFiltSeq;
  apConfirm = 0; apStopping = false;
  float expected = d0Mm - WALL_TARGET_MM;
  apTimeout = APPROACH_TIMEOUT_BASE_MS + (unsigned long)(expected * APPROACH_TIMEOUT_MS_PER_MM);
  if (gyroTrusted) holdHeading = headingDeg();
  Serial.print(F("[APPROACH] wall at ")); Serial.print(d0Mm, 0);
  Serial.print(F(" mm, target ")); Serial.print(WALL_TARGET_MM, 0);
  Serial.print(F(" mm, travel ~")); Serial.print(expected, 0);
  Serial.print(F(" mm, heading hold ")); Serial.println((USE_GYRO_HEADING_HOLD && gyroTrusted) ? F("ON") : F("OFF"));
  enterState(S_APPROACH);
}
void updateSettle(unsigned long now) {
  if (now - stateMs < SETTLE_MS) return;
  if (!tofFilteredFresh(now)) {
    if (now - stateMs > SETTLE_MS + WALL_SEE_TIMEOUT_MS) fault("no valid ToF reading after the turn (is the wall in front? check mux ch3 wiring)");
    return;
  }
  if ((float)tofFiltMm <= WALL_TARGET_MM + WALL_TOL_MM) {
    Serial.println(F("[APPROACH] already at the target distance, no approach needed"));
    enterState(S_FINAL_SETTLE);
    return;
  }
  startApproach(now);
}

void updateApproach(unsigned long now) {
  if (now - apTick < CONTROL_DT_MS) return;
  float dt = (now - apTick) / 1000.0f;
  apTick = now;

  if (!tofOnline) { fault("ToF lost during the approach"); return; }
  if (now - apT0 > apTimeout) { fault("approach timeout (robot too slow / ToF not closing)"); return; }

  // ---- ToF freshness + stop confirmation ----
  bool fresh = tofFilteredFresh(now);
  if (fresh) apLastFresh = now;
  else if (now - apLastFresh > APPROACH_TOF_LOSS_MS) { fault("ToF readings lost during the approach"); return; }

  float lead = APPROACH_LEAD_MM + apV * APPROACH_LATENCY_S;
  float remaining = (float)tofFiltMm - WALL_TARGET_MM - lead;
  if (fresh && tofFiltSeq != apLastSeq) {
    apLastSeq = tofFiltSeq;
    if (remaining <= 0) apConfirm++; else apConfirm = 0;
    if (apConfirm >= WALL_CONFIRM_COUNT && !apStopping) {
      apStopping = true;
      Serial.print(F("[APPROACH] stop trigger at ")); Serial.print(tofFiltMm); Serial.println(F(" mm"));
    }
  }

  // ---- travel sanity ----
  float travel = avgTravelSinceApproach();
  if (travel > (d0Mm - WALL_TARGET_MM) + 120.0f) { fault("travelled further than the ToF distance allows (ToF not closing)"); return; }
  if (travel > 150.0f && (d0Mm - (float)tofFiltMm) < 0.4f * travel) { fault("ToF is not closing with the encoder travel (wrong target in front?)"); return; }

  // ---- reference speed ----
  float vDes;
  if (apStopping) vDes = 0;
  else if (!fresh) vDes = APPROACH_MIN_MMPS;
  else {
    vDes = (remaining > 0) ? sqrtf(2.0f * APPROACH_DECEL_MMPS2 * remaining) : 0;
    vDes = clampf(vDes, APPROACH_MIN_MMPS, CRUISE_MMPS);
  }
  if (vDes > apV) apV = min(vDes, apV + APPROACH_ACCEL_MMPS2 * dt);
  else            apV = max(vDes, apV - APPROACH_STOP_DECEL_MMPS2 * dt);

  if (apStopping && apV <= 0.5f) {                       // reference has stopped -> motors off, then verify
    stopMotors();
    Serial.print(F("[APPROACH] stopped, travel ")); Serial.print(travel, 0); Serial.println(F(" mm"));
    tofFilterFlush();
    enterState(S_FINAL_SETTLE);
    return;
  }
  apS += apV * dt;

  // ---- gyro heading hold: slowly shifts the left/right references ----
  if (USE_GYRO_HEADING_HOLD && gyroTrusted && gyroOnline) {
    float errMm = (holdHeading - headingDeg()) * (PI / 180.0f) * TRACK_MM * 0.5f;   // +err = needs more LEFT
    hOff = clampf(hOff + HEADING_K * errMm * dt, -HEADING_OFF_MAX_MM, HEADING_OFF_MAX_MM);
  }

  // ---- calibrated PI on both wheels ----
  long cl = getL(), cr = getR();
  float posL = (cl - apStartL) * MM_PER_COUNT_L;
  float posR = (cr - apStartR) * MM_PER_COUNT_R;
  float eL = clampf((apS - hOff) - posL, -APPROACH_MAX_ERR_MM, APPROACH_MAX_ERR_MM);
  float eR = clampf((apS + hOff) - posR, -APPROACH_MAX_ERR_MM, APPROACH_MAX_ERR_MM);
  apIL = clampf(apIL + eL * dt, -50, 50);
  apIR = clampf(apIR + eR * dt, -50, 50);
  float uL = KV_L * apV + KP * eL + KI * apIL;
  float uR = KV_R * apV + KP * eR + KI * apIR;
  if (fabsf(uL) > 1) uL += (uL > 0 ? DEAD_L : -DEAD_L);
  if (fabsf(uR) > 1) uR += (uR > 0 ? DEAD_R : -DEAD_R);
  uL = clampf(uL, 0, PWM_MAX);                           // straight approach never drives backwards
  uR = clampf(uR, 0, PWM_MAX);
  setMotors(uL, uR);

  // ---- stall guard ----
  if (cl != apLastCL) { apLastCL = cl; apMovL = now; }
  if (cr != apLastCR) { apLastCR = cr; apMovR = now; }
  if ((uL > 60 && now - apMovL > STALL_MS) || (uR > 60 && now - apMovR > STALL_MS)) {
    fault("APPROACH: wheel driven but not turning (encoder / motor / blocked wheel)");
  }
}

// ---- verify the final distance, nudge if needed ----
void updateFinalSettle(unsigned long now) {
  if (now - stateMs < SETTLE_MS) return;
  if (!tofFilteredFresh(now)) {
    if (now - stateMs > SETTLE_MS + 1500) fault("no valid ToF reading while verifying the final distance");
    return;
  }
  finalDistMm = tofFiltMm;
  float err = finalDistMm - WALL_TARGET_MM;
  Serial.print(F("[RESULT] ToF ")); Serial.print(finalDistMm, 0);
  Serial.print(F(" mm (target ")); Serial.print(WALL_TARGET_MM, 0);
  Serial.print(F(", error ")); Serial.print(err, 0); Serial.println(F(" mm)"));
  if (fabsf(err) <= WALL_TOL_MM) {
    Serial.println(F("[RESULT] WITHIN TOLERANCE - mission complete"));
    enterState(S_DONE);
    return;
  }
  if (err > 0 && nudgeCount < NUDGE_MAX_COUNT) {
    nudgeCount++;
    float d = err - 2.0f;
    Serial.print(F("[NUDGE] forward ")); Serial.print(d, 0); Serial.println(F(" mm"));
    startMove(d, d, NUDGE_SPEED_MMPS, APPROACH_ACCEL_MMPS2);
    if (mAct) { enterState(S_NUDGE); return; }
  } else if (err < 0 && ALLOW_REVERSE_NUDGE && nudgeCount < NUDGE_MAX_COUNT) {
    nudgeCount++;
    float d = err + 2.0f;                                 // negative = backwards
    Serial.print(F("[NUDGE] reverse ")); Serial.print(-d, 0); Serial.println(F(" mm"));
    startMove(d, d, NUDGE_SPEED_MMPS, APPROACH_ACCEL_MMPS2);
    if (mAct) { enterState(S_NUDGE); return; }
  }
  Serial.println(err < 0 ? F("[RESULT] stopped CLOSER than the window (overshoot) - not corrected")
                         : F("[RESULT] stopped FARTHER than the window - nudge limit reached"));
  enterState(S_DONE);
}
void updateNudge() {
  motionTick();
  if (moveAborted) { fault(String("NUDGE: ") + moveAbortReason); return; }
  if (!mAct) { tofFilterFlush(); enterState(S_FINAL_SETTLE); }
}

// ---- odometry ----
void odomReset() {
  odomX = odomY = odomTh = 0;
  odomLastL = getL(); odomLastR = getR();
}
void odomUpdate() {
  long cl = getL(), cr = getR();
  float dL = (cl - odomLastL) * MM_PER_COUNT_L;
  float dR = (cr - odomLastR) * MM_PER_COUNT_R;
  odomLastL = cl; odomLastR = cr;
  float ds = 0.5f * (dL + dR);
  float dth = (dR - dL) / TRACK_MM;
  odomX += ds * cosf(odomTh + 0.5f * dth);
  odomY += ds * sinf(odomTh + 0.5f * dth);
  odomTh += dth;
}
void printOdom() {
  Serial.print(F("odom x=")); Serial.print(odomX, 1);
  Serial.print(F(" y=")); Serial.print(odomY, 1);
  Serial.print(F(" th=")); Serial.print(odomTh * 180.0f / PI, 2); Serial.println(F("deg"));
  Serial.print(F("counts L=")); Serial.print(getL()); Serial.print(F(" R=")); Serial.println(getR());
}

// ---- fixed movement: forward SEQ_FWD1_MM, turn left SEQ_TURN_DEG, forward SEQ_FWD2_MM ----
void startSeqStep() {
  const SeqStep &st = SEQ[seqIdx];
  Serial.print(F("[SEQ] step ")); Serial.print(seqIdx + 1); Serial.print('/'); Serial.print(SEQ_N); Serial.print(F(": "));
  if (st.kind == SK_FWD) {
    Serial.print(F("forward ")); Serial.print(st.value, 0); Serial.print(F(" mm at ")); Serial.print(MOVE_SPEED_MMPS, 0);
    Serial.print(F(" mm/s, a ")); Serial.println(MOVE_ACCEL_MMPS2, 0);
    startMove(st.value, st.value, MOVE_SPEED_MMPS, MOVE_ACCEL_MMPS2);
  } else {
    float arc = (st.value * PI / 180.0f) * TRACK_MM * 0.5f;           // +deg = left: left wheel back, right wheel forward
    seqGyroStartRaw = rawHeadingDeg;
    Serial.print(F("turn LEFT ")); Serial.print(st.value, 0); Serial.print(F(" deg (wheel arc ")); Serial.print(arc, 1);
    Serial.print(F(" mm) at ")); Serial.print(TURN_SPEED_MMPS, 0); Serial.println(F(" mm/s"));
    startMove(-arc, arc, TURN_SPEED_MMPS, MOVE_ACCEL_MMPS2);
  }
  if (!mAct) { fault("sequence move did not start"); return; }
  enterState(S_SEQ_MOVE);
}
void startSequence() {
  noInterrupts(); cntL = 0; cntR = 0; interrupts();                    // odometry and counts start from the start pose
  odomReset();
  seqIdx = 0;
  Serial.println(F("[SEQ] MISSION START: forward 85 mm, turn left 90 deg, forward 380 mm"));
  startSeqStep();
}
void updateSeqMove(unsigned long now) {
  motionTick();
  if (moveAborted) { fault(String("SEQ step ") + String(seqIdx + 1) + ": " + moveAbortReason); return; }
  if (TOF_SAFETY_STOP_MM > 0 && SEQ[seqIdx].kind == SK_FWD && tofFilteredFresh(now) && (float)tofFiltMm < TOF_SAFETY_STOP_MM) {
    fault(String("ToF safety stop: ") + String((int)tofFiltMm) + " mm"); return;
  }
  if (mAct) return;
  odomUpdate();
  Serial.print(F("[SEQ] step ")); Serial.print(seqIdx + 1); Serial.println(F(" done"));
  printOdom();
  if (SEQ[seqIdx].kind == SK_TURN && gyroOnline) {                      // information only: the gyro is not used for control here
    Serial.print(F("[SEQ] gyro saw ")); Serial.print(fabsf(rawHeadingDeg - seqGyroStartRaw), 1);
    Serial.println(F(" deg for this turn (not used)"));
  }
  seqIdx++;
  enterState(S_SEQ_PAUSE);
}
void updateSeqPause(unsigned long now) {
  if (now - stateMs < SEQ_PAUSE_MS) return;
  if (seqIdx < SEQ_N) { startSeqStep(); return; }
  odomUpdate();
  Serial.println(F("[RESULT] sequence complete (final odometry after settling):"));
  printOdom();
  Serial.print(F("[RESULT] expected  x~")); Serial.print(SEQ_FWD1_MM, 0);
  Serial.print(F("  y~")); Serial.print(SEQ_FWD2_MM, 0);
  Serial.print(F("  th~")); Serial.print(SEQ_TURN_DEG, 0); Serial.println(F("deg"));
  if (tofFilteredFresh(now)) { Serial.print(F("[RESULT] ToF in front: ")); Serial.print(tofFiltMm); Serial.println(F(" mm")); }
  enterState(S_DONE);
}

// ---- countdown + readiness ----
const char *readinessCheck() {
  if (BATTERY_SENSE_PIN >= 0) {
    float vb = analogRead((uint8_t)BATTERY_SENSE_PIN) * (5.0f / 1023.0f) * BATTERY_DIV_RATIO;
    Serial.print(F("[READY] motor battery ")); Serial.print(vb, 1); Serial.println(F(" V"));
    if (vb < BATTERY_MIN_V) return "MOTOR BATTERY below BATTERY_MIN_V (switch off / unplugged / flat): the wheels cannot move";
  } else Serial.println(F("[READY] motor battery guard not fitted (BATTERY_SENSE_PIN = -1)"));
  float ffMax = max(KV_L, KV_R) * VMAX_MMPS + max(DEAD_L, DEAD_R);
  if (ffMax > (float)PWM_MAX) return "CALIBRATION: kv x vmax + dead exceeds PWM_MAX (kv in the wrong units? expected ~0.88 PWM per mm/s)";
  if (MOVE_SPEED_MMPS > VMAX_MMPS || TURN_SPEED_MMPS > VMAX_MMPS) return "CALIBRATION: move / turn speed is above vmax";
  if (!USE_WALL_MISSION && !(SEQ_FWD1_MM > 0 && SEQ_FWD2_MM > 0 && SEQ_TURN_DEG > 0)) return "SEQUENCE: step values must be > 0";
  if (USE_WALL_MISSION && !tofOnline) return "front ToF is offline (check SD3/SC3, VIN, GND)";
  if (!tofOnline) Serial.println(F("[READY] note: front ToF offline (not needed for the fixed movement)"));
  if (!gyroOnline) Serial.println(USE_WALL_MISSION ? F("[READY] WARNING: gyro offline -> no turn trim / heading hold, encoders only")
                                                   : F("[READY] note: gyro offline (not needed for the fixed movement)"));
  return nullptr;
}
void updateCountdown(unsigned long now) {
  unsigned long el = now - stateMs;
  if (!recalDone && el >= BOOT_DELAY_MS - GYRO_RECAL_LEAD_MS) {
    recalDone = true;
    if (gyroOnline) {
      stopMotors();
      if (!gyroCalibrate()) {
        gyroOnline = false;
        Serial.println(F("[GYRO] re-calibration failed (robot moved?) -> gyro disabled for this run"));
      }
      rawHeadingDeg = 0;
    }
  }
  if (el >= BOOT_DELAY_MS) { if (USE_WALL_MISSION) startTurn(); else startSequence(); }
}

void updateBench(unsigned long now);
void missionUpdate(unsigned long now) {
  switch (state) {
    case S_COUNTDOWN:    updateCountdown(now); break;
    case S_TURN:         updateTurn(now); break;
    case S_SETTLE:       updateSettle(now); break;
    case S_APPROACH:     updateApproach(now); break;
    case S_FINAL_SETTLE: updateFinalSettle(now); break;
    case S_NUDGE:        updateNudge(); break;
    case S_SEQ_MOVE:     updateSeqMove(now); break;
    case S_SEQ_PAUSE:    updateSeqPause(now); break;
    case S_BENCH:        updateBench(now); break;
    default: break;
  }
}

// ============================================================================
// 10. TELEMETRY, MATRIX, SERIAL
// ============================================================================
void printTelemetry(unsigned long now) {
  bool moving = (state == S_TURN || state == S_APPROACH || state == S_NUDGE || state == S_SEQ_MOVE);
  if (state == S_BENCH) return;
  if (now - lastTelemetryMs < (moving ? 250UL : 1000UL)) return;
  lastTelemetryMs = now;
  Serial.print('['); Serial.print(now / 1000.0f, 1); Serial.print(F("s] "));
  Serial.print(stateName(state));
  Serial.print(F(" tof=")); Serial.print(tofRawMm); Serial.print('/'); Serial.print(tofFiltMm);
  Serial.print(F(" hd=")); Serial.print(headingDeg(), 1);
  Serial.print(gyroTrusted ? F("(ok)") : F("(raw)"));
  Serial.print(F(" encL=")); Serial.print(getL()); Serial.print(F(" encR=")); Serial.print(getR());
  Serial.print(F(" pwm=")); Serial.print((int)lastUL); Serial.print('/'); Serial.print((int)lastUR);
  Serial.print(F(" odom=(")); Serial.print(odomX, 0); Serial.print(','); Serial.print(odomY, 0); Serial.print(',');
  Serial.print(odomTh * 180.0f / PI, 1); Serial.println(F(")"));
  if (state == S_FAULT) { Serial.print(F("  FAULT: ")); Serial.println(faultReason); }
}

void renderMatrix(unsigned long now) {
  if (now - lastMatrixMs < 100) return;
  lastMatrixMs = now;
  memset(ledFrame, 0, sizeof(ledFrame));
  if (state == S_FAULT) {
    for (int i = 0; i < 8; i++) { ledFrame[i][2 + i] = 1; ledFrame[i][9 - i] = 1; }
  } else if (state == S_DONE) {
    for (int i = 0; i < 8; i++) { ledFrame[0][2 + i] = 1; ledFrame[7][2 + i] = 1; ledFrame[i][2] = 1; ledFrame[i][9] = 1; }
  } else {
    if (tofHistN >= TOF_FILTER_MIN) {
      int cols = (int)clampf((float)tofFiltMm / 1200.0f * 12.0f, 1.0f, 12.0f);
      for (int c = 0; c < cols; c++) { ledFrame[3][c] = 1; ledFrame[4][c] = 1; }
    }
    if (gyroTrusted && (state == S_APPROACH)) {
      float err = holdHeading - headingDeg();
      int col = 6 + (int)clampf(err / 2.0f, -5.0f, 5.0f);
      ledFrame[6][col] = 1; ledFrame[6][col + 1 > 11 ? 11 : col + 1] = 1;
    }
    if (state == S_COUNTDOWN) {
      int cols = (int)((now - stateMs) * 12UL / BOOT_DELAY_MS);
      if (cols > 12) cols = 12;
      for (int c = 0; c < cols; c++) ledFrame[7][c] = 1;
    }
  }
  matrix.renderBitmap(ledFrame, 8, 12);
}

// ============================================================================
// MOTOR MAP (EEPROM) + BENCH TOOLS
// ============================================================================
constexpr uint8_t MAP_MAGIC = 0xA7;
static bool isMotorPin(uint8_t p) { return p == 5 || p == 6 || p == 9 || p == 10; }   // the PWM-capable pins the MDD10A can be wired to
void saveMotorMap() {
  uint8_t b[7] = { pinLPwm, pinLDir, pinRPwm, pinRDir, (uint8_t)(motorLSign > 0 ? 1 : 0), (uint8_t)(motorRSign > 0 ? 1 : 0), 0 };
  b[6] = (uint8_t)(b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5] ^ MAP_MAGIC);
  EEPROM.write(0, MAP_MAGIC);
  for (uint8_t i = 0; i < 7; i++) EEPROM.write(1 + i, b[i]);
}
bool loadMotorMap() {
  if (EEPROM.read(0) != MAP_MAGIC) return false;
  uint8_t b[7];
  for (uint8_t i = 0; i < 7; i++) b[i] = EEPROM.read(1 + i);
  if (b[6] != (uint8_t)(b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5] ^ MAP_MAGIC)) return false;
  for (uint8_t i = 0; i < 4; i++) if (!isMotorPin(b[i])) return false;
  if (b[0] == b[1] || b[0] == b[2] || b[0] == b[3] || b[1] == b[2] || b[1] == b[3] || b[2] == b[3]) return false;
  pinLPwm = b[0]; pinLDir = b[1]; pinRPwm = b[2]; pinRDir = b[3];
  motorLSign = b[4] ? 1 : -1; motorRSign = b[5] ? 1 : -1;
  return true;
}
void printMotorMap() {
  Serial.print(F("[MOTORS] map: LEFT PWM=D")); Serial.print(pinLPwm); Serial.print(F(" DIR=D")); Serial.print(pinLDir);
  Serial.print(F(" sign ")); Serial.print((int)motorLSign);
  Serial.print(F(" | RIGHT PWM=D")); Serial.print(pinRPwm); Serial.print(F(" DIR=D")); Serial.print(pinRDir);
  Serial.print(F(" sign ")); Serial.print((int)motorRSign);
  Serial.print(F("   source=")); Serial.println(motorMapSource);
}

enum BenchMode : uint8_t { BM_NONE, BM_PROBE, BM_ENC };
BenchMode benchMode = BM_NONE;
unsigned long benchT0 = 0, benchLastPrint = 0;
uint16_t probeStep = 0;                                  // 0 = start delay, then 24 runs of (run, pause): 12 pin pairs x DIR low/high
long probeCnt[12][2][2];                                 // [pair][DIR level 0/1][wheel 0=L 1=R] = encoder counts during the run
uint8_t probeA[12], probeB[12];                          // pair k: PWM on pin A, level on pin B
long probeStartL = 0, probeStartR = 0, benchEnc0L = 0, benchEnc0R = 0;
constexpr unsigned long PROBE_START_DELAY_MS = 4000, PROBE_RUN_MS = 220, PROBE_PAUSE_MS = 280;
constexpr int  PROBE_PWM = 200;
constexpr long PROBE_MIN_COUNTS = 25;                    // a wheel counts as "moved" above this many counts

static void pinLowOut(uint8_t pin) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }   // pinMode also releases a pin that was used for PWM
void benchFinish(const char *why) {
  stopMotors();
  const uint8_t P[4] = {5, 6, 9, 10};
  for (uint8_t i = 0; i < 4; i++) pinLowOut(P[i]);
  benchMode = BM_NONE;
  guardL = DirGuard(); guardR = DirGuard();
  Serial.print(F("[BENCH] ")); Serial.print(why);
  Serial.println(F(". Motors off. Press RESET to run the mission (it loads the saved motor map). 'p' / 'e' repeat a tool."));
}

void benchStart(BenchMode m) {
  stopMotors();
  benchMode = m; benchT0 = millis(); benchLastPrint = benchT0; probeStep = 0;
  state = S_BENCH; stateMs = benchT0;
  if (m == BM_PROBE) {
    uint8_t n = 0; const uint8_t P[4] = {5, 6, 9, 10};
    for (uint8_t a = 0; a < 4; a++) for (uint8_t b = 0; b < 4; b++) if (a != b) { probeA[n] = P[a]; probeB[n] = P[b]; n++; }
    for (uint8_t k = 0; k < 12; k++) probeCnt[k][0][0] = probeCnt[k][0][1] = probeCnt[k][1][0] = probeCnt[k][1][1] = 0;
    Serial.println(F("\n[PROBE] WIRING PROBE starts in 4 s. ROBOT MUST BE ON BLOCKS (wheels free): every PWM/DIR pin combination of D5 D6 D9 D10"));
    Serial.println(F("[PROBE] is pulsed for 0.2 s while the encoders are watched. Takes ~15 s. Any key aborts."));
  } else {
    benchEnc0L = getL(); benchEnc0R = getR();
    Serial.println(F("\n[ENC] ENCODER MONITOR (motors off): turn each wheel by hand, forward and backward. Any key stops it."));
  }
}

void probeAnalyse() {
  Serial.println(F("[PROBE] result table  (counts while pulsed: DIR low / DIR high)"));
  int bestL = -1, bestR = -1; long bestLs = 0, bestRs = 0, maxAbs = 0;
  for (uint8_t k = 0; k < 12; k++) {
    Serial.print(F("  PWM=D")); Serial.print(probeA[k]); Serial.print(F(" DIR=D")); Serial.print(probeB[k]);
    Serial.print(F("   LEFT enc ")); Serial.print(probeCnt[k][0][0]); Serial.print('/'); Serial.print(probeCnt[k][1][0]);
    Serial.print(F("   RIGHT enc ")); Serial.print(probeCnt[k][0][1]); Serial.print('/'); Serial.println(probeCnt[k][1][1]);
    for (uint8_t w = 0; w < 2; w++) {
      long lo = probeCnt[k][0][w], hi = probeCnt[k][1][w];
      if (labs(lo) > maxAbs) maxAbs = labs(lo);
      if (labs(hi) > maxAbs) maxAbs = labs(hi);
      long strength = min(labs(lo), labs(hi));
      bool good = (labs(lo) >= PROBE_MIN_COUNTS && labs(hi) >= PROBE_MIN_COUNTS && ((lo < 0) != (hi < 0)));   // moves AND reverses with DIR
      if (good && w == 0 && strength > bestLs) { bestLs = strength; bestL = k; }
      if (good && w == 1 && strength > bestRs) { bestRs = strength; bestR = k; }
    }
  }
  if (maxAbs < 10) {
    Serial.println(F("[PROBE] NO ENCODER COUNT AT ALL for ANY pin combination."));
    Serial.println(F("        If the wheels did NOT turn: the motors get no power -> motor battery connected + switched ON? MDD10A power LED lit?"));
    Serial.println(F("        MDD10A B+/B- on the battery, Arduino GND joined to the MDD10A signal GND, motor leads tight in M1A/M1B/M2A/M2B."));
    Serial.println(F("        If the wheels DID turn: the encoders are dead -> encoder VCC/GND, A wires on D3 (left) and D2 (right)."));
    return;
  }
  if (bestL >= 0 && bestR >= 0 && probeA[bestL] != probeA[bestR] && probeA[bestL] != probeB[bestR] && probeB[bestL] != probeA[bestR] && probeB[bestL] != probeB[bestR]) {
    pinLPwm = probeA[bestL]; pinLDir = probeB[bestL]; motorLSign = (probeCnt[bestL][1][0] > 0) ? 1 : -1;
    pinRPwm = probeA[bestR]; pinRDir = probeB[bestR]; motorRSign = (probeCnt[bestR][1][1] > 0) ? 1 : -1;
    motorMapSource = "probe";
    saveMotorMap();
    Serial.println(F("[PROBE] FOUND both wheels. Adopted and SAVED to EEPROM:"));
    printMotorMap();
    Serial.println(F("        (sign -1 = DIR HIGH moves that wheel backwards, so the sketch drives it with the opposite level)"));
    Serial.println(F("        Put these four numbers into DEF_L_PWM / DEF_L_DIR / DEF_R_PWM / DEF_R_DIR if you want them in the source too."));
    Serial.println(F("        The encoder sign convention (positive counts = forward) is taken from the calibration: on the first run WATCH that the robot drives FORWARD."));
    return;
  }
  Serial.print(F("[PROBE] only partly found: LEFT ")); Serial.print(bestL >= 0 ? F("yes") : F("NO")); Serial.print(F(", RIGHT ")); Serial.println(bestR >= 0 ? F("yes") : F("NO"));
  Serial.println(F("        The wheel marked NO produced no clean 'moves and reverses with DIR' result: that motor / its driver channel / its DIR wire / its encoder is faulty."));
  Serial.println(F("        Not saved. Use the table above (a wheel that moves in one DIR column only has a dead DIR wire)."));
}

void updateBench(unsigned long now) {
  if (benchMode == BM_NONE) { setMotors(0, 0); return; }
  if (benchMode == BM_ENC) {
    if (now - benchLastPrint >= 250) {
      benchLastPrint = now;
      Serial.print(F("[ENC] L: A=")); Serial.print(digitalRead(PIN_ENC_L_A)); Serial.print(F(" B=")); Serial.print(digitalRead(PIN_ENC_L_B));
      Serial.print(F(" counts=")); Serial.print(getL() - benchEnc0L);
      Serial.print(F(" | R: A=")); Serial.print(digitalRead(PIN_ENC_R_A)); Serial.print(F(" B=")); Serial.print(digitalRead(PIN_ENC_R_B));
      Serial.print(F(" counts=")); Serial.println(getR() - benchEnc0R);
    }
    return;
  }
  // ---- wiring probe ----
  unsigned long el = now - benchT0;
  if (probeStep == 0) {
    if (el < PROBE_START_DELAY_MS) return;
    probeStep = 1; benchT0 = now;
    const uint8_t P[4] = {5, 6, 9, 10};
    for (uint8_t i = 0; i < 4; i++) pinLowOut(P[i]);
    el = 0;
  }
  uint16_t run = (probeStep - 1) / 2; bool running = (probeStep % 2) == 1;
  uint8_t k = run / 2, level = run % 2;
  if (run >= 24) { probeAnalyse(); benchFinish("probe finished"); return; }
  if (running) {
    static uint16_t startedStep = 0xFFFF;
    if (startedStep != probeStep) {                     // first call of this run
      startedStep = probeStep;
      probeStartL = getL(); probeStartR = getR();
      const uint8_t P[4] = {5, 6, 9, 10};
      for (uint8_t i = 0; i < 4; i++) pinLowOut(P[i]);
      digitalWrite(probeB[k], level ? HIGH : LOW);
      analogWrite(probeA[k], PROBE_PWM);
    }
    if (el >= PROBE_RUN_MS) {
      analogWrite(probeA[k], 0);
      pinLowOut(probeA[k]); pinLowOut(probeB[k]);
      probeCnt[k][level][0] = getL() - probeStartL;
      probeCnt[k][level][1] = getR() - probeStartR;
      probeStep++; benchT0 = now;
    }
  } else if (el >= PROBE_PAUSE_MS) { probeStep++; benchT0 = now; }
}

void serialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    if (state == S_BENCH && benchMode != BM_NONE) { benchFinish("aborted by serial input"); continue; }
    if (c == 'p' || c == 'P' || c == 'e' || c == 'E') {
      if (state == S_COUNTDOWN || state == S_FAULT || state == S_DONE || state == S_BENCH) benchStart((c == 'p' || c == 'P') ? BM_PROBE : BM_ENC);
      else Serial.println(F("[CMD] refused: send p / e only during the countdown, after a fault, or when done (never while moving)"));
      continue;
    }
    if (c == 'z' || c == 'Z') {
      EEPROM.write(0, 0);
      Serial.println(F("[MOTORS] saved motor map erased - reset to use the sketch defaults"));
      continue;
    }
    if ((c == 'x' || c == 'X' || c == 's' || c == 'S') && state != S_FAULT && state != S_DONE) fault("emergency stop from serial");
  }
}

// ============================================================================
// 11. SETUP / LOOP
// ============================================================================
void setup() {
  // motors OFF first (this also claims the PWM timers before the LED matrix starts)
  { const uint8_t P[4] = {5, 6, 9, 10}; for (uint8_t i = 0; i < 4; i++) pinLowOut(P[i]); }   // every candidate motor pin LOW first
  if (USE_SAVED_MOTOR_MAP && loadMotorMap()) motorMapSource = "saved (EEPROM)";
  pinMode(pinLPwm, OUTPUT); pinMode(pinLDir, OUTPUT);
  pinMode(pinRPwm, OUTPUT); pinMode(pinRDir, OUTPUT);
  stopMotors();

  Serial.begin(115200);
  { unsigned long tw = millis(); while (!Serial && millis() - tw < 4000UL) delay(10); }
  matrix.begin();
  showStage(1);
  Serial.println(USE_WALL_MISSION ? F("\n[BOOT] Robot A final mission: left turn + straight to the ToF target")
                                  : F("\n[BOOT] Robot A final mission: forward 85, turn left 90, forward 380"));

  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrR, CHANGE);
  printMotorMap();
  Serial.println(F("[INIT] encoders attached (L: D3/D7, R: D2/D4)"));
  showStage(3);

  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  if (checkI2CPresence(TCA_I2C_ADDR)) Serial.println(F("[INIT] TCA9548A found at 0x70"));
  else Serial.println(F("[INIT] WARNING: TCA9548A not answering at 0x70 (check SDA/SCL, power)"));
  showStage(5);

  Serial.println(F("[BOOT] starting ToF bring-up (mux ch3)..."));
  showStage(6);
  tofOnline = tofBegin();
  Serial.println(tofOnline ? F("[BOOT] front VL53L1X online") : F("[BOOT] front VL53L1X OFFLINE"));
  showStage(8);

  gyroOnline = gyroBegin();
  tcaDisableAll();
  Serial.println(gyroOnline ? F("[BOOT] gyro online (mux ch0)") : F("[BOOT] gyro OFFLINE"));
  showStage(10);
  rawHeadingDeg = 0;
  lastGyroUs = micros();
  showStage(12);

  Serial.println(F("[CAL] cpr 1462.25/1457.55  corr 1.87264/1.87264  wheel 70  track 187.01  dead 14.4/14.4  kv 0.87669/0.87769  vmax 223.4  kp 3  ki 0.5  v 150  a 300  tv 100"));
  Serial.print(F("[CAL] counts per mm L/R = ")); Serial.print(COUNTS_PER_MM_L, 3); Serial.print('/'); Serial.print(COUNTS_PER_MM_R, 3);
  Serial.print(F("   mm per count = ")); Serial.print(MM_PER_COUNT_L, 5); Serial.print('/'); Serial.println(MM_PER_COUNT_R, 5);
  if (USE_WALL_MISSION) {
    Serial.print(F("[MISSION] turn LEFT ")); Serial.print(LEFT_TURN_DEG, 0);
    Serial.print(F(" deg slowly, then straight until ToF = ")); Serial.print(WALL_TARGET_MM, 0); Serial.println(F(" mm"));
  } else {
    Serial.print(F("[MISSION] forward ")); Serial.print(SEQ_FWD1_MM, 0); Serial.print(F(" mm, turn LEFT ")); Serial.print(SEQ_TURN_DEG, 0);
    Serial.print(F(" deg, forward ")); Serial.print(SEQ_FWD2_MM, 0); Serial.println(F(" mm (encoders only, gyro not used)"));
  }

  enterState(S_READY_CHECK);
  const char *why = readinessCheck();
  if (why) { fault(why); return; }
  Serial.println(F("[READY] OK - robot must stay still. Countdown starts. Send 'x' to stop."));
  recalDone = false;
  enterState(S_COUNTDOWN);
}

void loop() {
  unsigned long now = millis();
  gyroUpdate(now);
  tofUpdate(now);
  odomUpdate();
  serialCommands();
  missionUpdate(now);
  printTelemetry(now);
  renderMatrix(now);
}
