/* ============================================================================
   ROBOT A - QUARANTINE-ZONE NAVIGATION, SIMPLE VERSION (NO GYRO)  (Arduino UNO R4 WiFi)

   The gyro / MPU is NOT used any more. Every movement is plain and uses only the wheel encoders and the front ToF:

     power on -> motors 0 -> init -> readiness check -> 5 s stationary countdown
     -> (optional) INITIAL_FORWARD  : straight, left/right wheel distances kept equal
     -> LEFT_TURN                   : both wheels pivot at a fixed slow PWM until the encoder arc
                                      (TURN_TRACK_MM x angle) is reached, tapering at the end, then stop
     -> TURN_SETTLE                 : stand still, refill the ToF filter
     -> WALL_APPROACH / FINE        : straight, wheel distances kept equal, speed tiers by filtered ToF
     -> QUARANTINE_STOP -> QUARANTINE_READY (stopped for good until reset)

   ACCURACY NOTE: with no gyro the turn angle is only as good as TURN_TRACK_MM (and wheel grip). Calibrate it once:
   run the turn, measure the real angle, set TURN_TRACK_MM = old x (90 / real angle).

   Kept unchanged: MDD10A safety layer (ramp to 0 -> neutral -> reverse), encoders, VL53L1X driver + channel discovery,
   median ToF filter, speed-adaptive wall stop, faults, bench tools ('m' motor test, 'e' encoder monitor, 'd' DIR pin test).
   NOT implemented on purpose: search, discs, pickup, arm, servos, delivery.
   ============================================================================ */

#include <Wire.h>
#include <WiFiS3.h>
#include <Adafruit_VL53L1X.h>
#include "Arduino_LED_Matrix.h"
#include <math.h>
#include <stdio.h>

// ============================================================================
// 0. TYPES (kept at the top for the Arduino preprocessor)
// ============================================================================
enum MissionState : uint8_t {
  S_BOOT,
  S_INIT,
  S_READY_CHECK,
  S_START_COUNTDOWN,
  S_INITIAL_FORWARD,
  S_PRETURN_DECEL,
  S_LEFT_TURN,
  S_TURN_SETTLE,
  S_WALL_APPROACH,
  S_WALL_FINE_APPROACH,
  S_QUARANTINE_STOP,
  S_QUARANTINE_READY,
  S_FAULT,
  S_BENCH_TEST           // serial 'm' / 'g' bench tools; never entered while the mission is moving
};

struct MotorAxis { int16_t current = 0; int8_t sign = 1; unsigned long zeroMs = 0; };

struct LogRecord {
  uint32_t t;
  int16_t  tofRaw;
  int16_t  tofFilt;
  int16_t  head10;   // heading x10 (deg)
  int16_t  tgt10;    // target heading x10 (deg)
  int16_t  lpwm;
  int16_t  rpwm;
  int16_t  encl;
  int16_t  encr;
  int16_t  travel;   // mm since the last drive reset
  uint8_t  state;
};

// ============================================================================
// 1. HARDWARE CONFIGURATION  (unchanged from the working sketch)
// ============================================================================
// Cytron MDD10A
constexpr uint8_t PIN_PWM_LEFT    = 5;
constexpr uint8_t PIN_DIR_LEFT    = 6;
constexpr uint8_t PIN_PWM_RIGHT   = 9;
constexpr uint8_t PIN_DIR_RIGHT   = 10;
// Motor direction flags. FLIPPED after the motor wiring was redone (both motors were spinning backwards).
// Check on the first run: the robot must pivot LEFT (counter-clockwise seen from above). If it pivots RIGHT, flip BOTH.
bool invertLeftDir  = true;
bool invertRightDir = false;

// Encoders (A = interrupt pin, counted on RISING). "LEFT"/"RIGHT" = the MOTOR CHANNEL (see the
// working sketch: the D3/D7 encoder sits on the D5/D6 motor, the D2/D4 encoder on the D9/D10 motor).
constexpr uint8_t PIN_ENC_L_A = 3;
constexpr uint8_t PIN_ENC_L_B = 7;
constexpr uint8_t PIN_ENC_R_A = 2;
constexpr uint8_t PIN_ENC_R_B = 4;
constexpr int8_t  LEFT_FORWARD_SIGN  = +1;
constexpr int8_t  RIGHT_FORWARD_SIGN = -1;
// false = count A edges only; direction comes from the motor command (immune to a bad B wire).
constexpr bool ENCODER_USE_B_DIRECTION = false;

constexpr uint8_t PIN_XSHUT_TOF = A0;

// TCRT5000 modules: wired and configured as inputs, but IGNORED by this version (next task).
constexpr uint8_t PIN_TCRT_LEFT   = A1;
constexpr uint8_t PIN_TCRT_CENTRE = A2;
constexpr uint8_t PIN_TCRT_RIGHT  = A3;

// D8 (old start button) is REMOVED and must stay unused. D12 (old digital IR) is no longer used either.

// TCA9548A mux (only the ToF sits behind it now)
constexpr uint8_t TCA_I2C_ADDR = 0x70;
constexpr uint8_t TCA_CH_TOF   = 1;          // configured channel; tofBegin() searches the others and reports the real one
constexpr uint8_t TOF_ADDR     = 0x29;

// EXISTING ESTIMATES (not newly verified). Confirm with a measured 5-turn wheel rotation.
constexpr float WHEEL_DIAMETER_MM = 65.0f;
constexpr float ENCODER_TICKS_REV = 660.0f;
constexpr float MM_PER_TICK = (3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_TICKS_REV;

// ============================================================================
// 2. CALIBRATION BLOCK  (EVERYTHING THAT NEEDS A PHYSICAL FIELD TEST)
// ============================================================================
// ---- Route geometry (PLACEHOLDERS - not measured, not rulebook values) ----
constexpr float INITIAL_FORWARD_MM         = 0.0f;    // 0 = turn left immediately at the start pose (no forward leg). >=50 = drive this far first
constexpr float QUARANTINE_LEFT_TURN_DEG   = 90.0f;   // PLACEHOLDER: ~90 for first test; left = positive
constexpr float QUARANTINE_WALL_TARGET_MM  = 280.0f;  // OUR navigation strategy, NOT a rulebook value. ToF reading (sensor to wall) at the stop
constexpr float MAX_WALL_APPROACH_MM       = 800.0f;  // PLACEHOLDER: max encoder travel after the turn before "WALL NOT FOUND"

// ---- Speeds (PWM 0..255; below ~80 the wheels stall) ----
constexpr int16_t INITIAL_CRUISE_PWM = 140;   // PLACEHOLDER
constexpr int16_t WALL_FAST_PWM      = 150;   // PLACEHOLDER
constexpr int16_t WALL_MEDIUM_PWM    = 130;   // PLACEHOLDER
constexpr int16_t WALL_SLOW_PWM      = 115;   // PLACEHOLDER
constexpr int16_t WALL_CRAWL_PWM     = 105;   // PLACEHOLDER
constexpr int16_t WALL_MIN_PWM       = 100;   // lowest forward PWM ever commanded during the wall approach

// ---- Wall-approach speed zones (distance ABOVE QUARANTINE_WALL_TARGET_MM; PLACEHOLDERS) ----
constexpr float WALL_MEDIUM_ZONE_MM = 450.0f;  // above this: FAST
constexpr float WALL_SLOW_ZONE_MM   = 250.0f;  // above this: MEDIUM, below: SLOW
constexpr float WALL_FINE_ZONE_MM   = 100.0f;  // inside this: CRAWL (state WALL_FINE_APPROACH)

// ---- Wall confirmation ----
constexpr float   WALL_STOP_LEAD_MM         = 5.0f;   // fixed part of the stop lead
constexpr float   WALL_STOP_LATENCY_S       = 0.32f;  // speed-dependent part: lead += closing speed x this (ToF filter + confirm + brake ramp).
                                                      // The stop trigger is  target + LEAD + speed x LATENCY, so a heavier / faster robot stops earlier.
                                                      // TUNE from the [STOP] line: error < 0 (too close) -> raise LATENCY_S; error > 0 (short) -> lower it.
constexpr uint8_t WALL_CONFIRM_COUNT        = 3;      // consecutive NEW filtered samples at/below the trigger
constexpr float   WALL_DISTANCE_TOLERANCE_MM = 10.0f; // accepted final window: target +/- this

// ---- Timing ----
constexpr unsigned long BOOT_DELAY_MS      = 5000;   // stationary countdown after readiness PASS
constexpr unsigned long TURN_SETTLE_MS     = 400;    // stationary time after the left turn
constexpr unsigned long QUARANTINE_STOP_MS = 500;    // stationary time after the wall stop
constexpr unsigned long MATCH_DURATION_MS  = 120000; // 2-minute match
// Set true ONLY for bench work. Competition timing must stay enforced.
constexpr bool BENCH_TEST_DISABLE_MATCH_TIMER = false;

// ---- Pre-turn deceleration ----
constexpr float PRETURN_DECEL_MM       = 80.0f;   // PLACEHOLDER: speed ramps down over the last this-much of INITIAL_FORWARD_MM
constexpr float PRETURN_STOP_EARLY_MM  = 6.0f;    // PLACEHOLDER: command zero this early (coast)
constexpr unsigned long PRETURN_QUIET_MS = 150;   // encoders must be quiet this long before the turn starts

// ---- Left turn (encoders only) ----
constexpr float   TURN_TRACK_MM      = 300.0f;    // effective pivot track used to turn degrees into wheel arc. ~300 from your earlier logs
                                                  // (gyro said 104 deg for ~277 mm of wheel travel). CALIBRATE: new = old x (90 / real angle)
constexpr int16_t TURN_KICK_PWM      = 190;       // breakaway kick at the start of the turn
constexpr unsigned long TURN_KICK_MS = 200;       // kick duration, counted from when the outputs have really reached TURN_KICK_PWM
constexpr int16_t TURN_PWM           = 140;       // slow turning PWM
constexpr int16_t TURN_MIN_PWM       = 115;       // PWM at the very end of the taper (must still move the robot)
constexpr float   TURN_TAPER_FRACTION = 0.30f;    // taper over the last 30 % of the arc
constexpr float   TURN_STOP_EARLY_MM = 12.0f;     // command zero this much arc early (coasting). Tune: turned too far -> raise, too little -> lower
constexpr unsigned long TURN_TIMEOUT_MS = 9000;
constexpr unsigned long TURN_WATCHDOG_MS = 1500;  // within this time both wheels must have counted TURN_MIN_TICKS
constexpr long          TURN_MIN_TICKS   = 100;

// ---- Bench tools (serial 'm' motor test, 'e' encoder monitor, 'd' DIR pin test) ----
constexpr int16_t BENCH_MOTOR_PWM = 180;
constexpr unsigned long BENCH_START_DELAY_MS = 2000;
constexpr unsigned long BENCH_RUN_MS   = 1000;
constexpr unsigned long BENCH_PAUSE_MS = 600;

// ---- Straight driving ----
constexpr int16_t MAX_PWM       = 200;     // raised from 165 for the heavy (~1.5 kg) robot; 200/255 = 78 % duty
constexpr int16_t MIN_DRIVE_PWM = 100;
constexpr int16_t DRIVE_KICK_PWM = 190;           // breakaway kick when a straight drive starts from standstill
constexpr unsigned long DRIVE_KICK_MS = 200;      // 0 = no kick (previous behaviour)
constexpr float   DRIVE_ACCEL_PWM_PER_S = 160.0f; // speed-command ramp up
constexpr float   DRIVE_DECEL_PWM_PER_S = 400.0f; // speed-command ramp down
constexpr float   STRAIGHT_KP       = 0.25f;          // PWM per mm of (left - right) wheel-distance difference
constexpr float   STRAIGHT_MAX_CORR = 25.0f;          // max PWM correction between the wheels

// ---- Timeouts / watchdogs ----
constexpr unsigned long INITIAL_FORWARD_TIMEOUT_MS = 8000;   // covers INITIAL_FORWARD + PRETURN_DECEL
constexpr unsigned long WALL_APPROACH_TIMEOUT_MS   = 8000;    // base; plus WALL_TIMEOUT_MS_PER_MM x distance still to travel
constexpr unsigned long WALL_TIMEOUT_MS_PER_MM     = 30;      // = the robot must average >= ~33 mm/s on the wall approach
constexpr unsigned long ENCODER_STALL_MS = 500;
constexpr bool          ENCODER_SINGLE_FAIL_IS_FAULT = true;  // one silent encoder while the other counts -> FAULT
constexpr float         IMBALANCE_FAULT_MM = 80.0f;   // wheels disagree by this much (slip / one wheel blocked) for IMBALANCE_FAULT_MS -> FAULT
constexpr unsigned long IMBALANCE_FAULT_MS = 400;
constexpr float         WALL_PROGRESS_AFTER_MM = 150.0f;      // after this travel the ToF must have closed...
constexpr float         WALL_PROGRESS_MIN      = 0.30f;       // ...at least this fraction of it
constexpr unsigned long LOOP_WATCHDOG_MS = 300;
constexpr uint8_t       LOOP_STALL_MAX   = 4;

// ---- ToF (existing driver + settings) ----
constexpr uint16_t TOF_MIN_MM = 30;
constexpr uint16_t TOF_MAX_MM = 4000;
constexpr uint16_t TOF_TIMING_BUDGET_MS = 33;
constexpr uint16_t TOF_INTERMEASURE_MS  = 35;
constexpr bool     TOF_FAST_MODE        = true;
constexpr unsigned long TOF_POLL_MS = 8;
// The narrow disc ROI is OFF here: the full field of view is the right choice for a wall.
constexpr bool    TOF_NARROW_ROI = false;
constexpr uint8_t TOF_ROI_X      = 16;
constexpr uint8_t TOF_ROI_Y      = 8;
// Filter: rolling median of the last TOF_FILTER_N VALID samples (small, little lag).
constexpr uint8_t  TOF_FILTER_N   = 5;
constexpr uint8_t  TOF_FILTER_MIN = 3;       // samples needed before the filtered value is trusted
constexpr unsigned long TOF_STALE_MS = 300;  // filtered value older than this = not trusted
constexpr unsigned long TOF_INVALID_FAULT_MS = 800;   // no trusted reading for this long during the approach -> FAULT
constexpr unsigned long TOF_PRIME_TIMEOUT_MS = 2000;

// ---- Loop intervals ----
constexpr unsigned long CONTROL_INTERVAL_MS   = 10;
constexpr unsigned long LOG_INTERVAL_MS       = 150;
constexpr unsigned long TELEMETRY_INTERVAL_MS = 250;
constexpr unsigned long MATRIX_INTERVAL_MS    = 60;

// ---- Wi-Fi (read-only dashboard before / after the mission; not serviced while moving) ----
constexpr bool ENABLE_WIFI = true;
const char WIFI_SSID[] = "RobotA_Telemetry";
const char WIFI_PASS[] = "12345678";

// ============================================================================
// 3. GLOBAL STATE
// ============================================================================
ArduinoLEDMatrix matrix;
Adafruit_VL53L1X tof;
WiFiServer server(80);
uint8_t ledFrame[8][12];

volatile long encLeftTicks  = 0;
volatile long encRightTicks = 0;
volatile int8_t  encDirL = 1, encDirR = 1;
volatile uint32_t encAgreeL = 0, encDisL = 0, encAgreeR = 0, encDisR = 0;
volatile long     encBSumL = 0, encBSumR = 0;       // sum of the B level (+1/-1) at every A edge (bench test only)
volatile uint32_t encEdgesL = 0, encEdgesR = 0;     // raw A-edge counts (bench test only)

uint8_t tcaCurrent = 0xFF;
uint8_t loopStalls = 0;
unsigned long durTof = 0, durMission = 0, durWifi = 0, durMatrix = 0, durTelem = 0;
uint8_t i2cRecoveries = 0;
unsigned long lastRecoverMs = 0;
uint8_t tofChannel = TCA_CH_TOF;

// ToF
bool     tofOnline   = false;
bool     tofValid    = false;          // last sample was in range
int16_t  tofRawMm    = -1;             // TOF_RAW: last raw sample (-1 = library reported invalid)
uint16_t tofFiltMm   = 0;              // TOF_FILTERED: rolling median of the last valid samples
uint16_t tofHist[TOF_FILTER_N];
uint8_t  tofHistN    = 0;
uint8_t  tofHistPos  = 0;
uint32_t tofSeq      = 0;              // every sample
uint32_t tofFiltSeq  = 0;              // every VALID sample (filtered value updated)
uint32_t tofValidCount = 0, tofInvalidCount = 0, tofConsecInvalid = 0;
unsigned long tofMs  = 0;              // time of last sample (valid or not)
unsigned long tofLastValidMs = 0;
unsigned long lastTofPollMs = 0;
uint8_t  tofFailCount = 0;

// Motors
int16_t liveLeftPwm  = 0;
int16_t liveRightPwm = 0;

// Mission
MissionState state = S_BOOT;
const char *faultReason = "none";
unsigned long stateStartMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastLoopMs = 0;
unsigned long missionStartMs = 0, missionStopMs = 0;
bool missionStarted = false, missionClockRunning = false;
int8_t lastCountdownShown = 6;

float initialTravelMm = 0.0f, wallApproachTravelMm = 0.0f;
float finalWallMm = 0.0f, turnResultDeg = 0.0f;

unsigned long moveStartMs = 0;        // movement timeout reference (INITIAL_FORWARD + PRETURN_DECEL)
unsigned long wallStartMs = 0;
unsigned long wallTimeoutMs = WALL_APPROACH_TIMEOUT_MS;   // set per run from the distance still to travel        // wall-approach timeout reference
bool  brakePhase = false;             // PRETURN_DECEL: false = profiled drive, true = ramp-to-zero wait
unsigned long brakeStartMs = 0;

// turn
float turnArcTargetMm = 0.0f;         // wheel arc that gives QUARANTINE_LEFT_TURN_DEG
long  turnEncL0 = 0, turnEncR0 = 0;
bool  turnBrake = false;
unsigned long turnBrakeMs = 0;
bool    turnKickActive = false;
unsigned long turnKickBegin = 0, turnKickT0 = 0;
uint32_t settleFiltSeq0 = 0;
uint32_t turnAgreeL0 = 0, turnDisL0 = 0, turnAgreeR0 = 0, turnDisR0 = 0;   // encoder-B counters at turn start
unsigned long imbalanceSinceMs = 0;

// bench tools
enum BenchMode : uint8_t { BM_NONE, BM_MOTOR, BM_ENC, BM_DIR };
BenchMode benchMode = BM_NONE;
uint8_t benchStep = 0;
unsigned long benchT0 = 0, benchLastPrintMs = 0;
long benchTicks0 = 0, benchB0 = 0;
uint32_t benchEdges0 = 0;
const char *benchResult[4] = {"-", "-", "-", "-"};
int8_t benchBDir[4] = {0, 0, 0, 0};      // direction encoder B reported per run (after FORWARD_SIGN)
long   benchA[4] = {0, 0, 0, 0};         // A edges per run
uint8_t benchBSeen[4] = {0, 0, 0, 0};    // bit0 = B pin read LOW at least once during the run, bit1 = read HIGH (both = B is alive)
uint32_t benchLastEdges[2] = {0, 0};     // edge counters at the end of the previous run (stray-edge detection)
uint32_t benchStray = 0;
bool benchMidPrinted = false;

// drive health
long  driveL0 = 0, driveR0 = 0, prevL = 0, prevR = 0;
unsigned long lastLEdgeMs = 0, lastREdgeMs = 0, driveStartMs = 0;
bool  encWarned = false;
long  quietL = 0, quietR = 0;
unsigned long lastEncMoveMs = 0;

// speed command ramp
float speedCmd = 0.0f;
unsigned long lastSpeedMs = 0;
bool driveKickEnabled = true, driveKickActive = false;
unsigned long driveKickBegin = 0, driveKickT0 = 0;

// wall approach
float wallStartFiltMm = 0.0f;
float wallSpeedMmS = 0.0f, wallPrevTrav = 0.0f;   // closing speed from the encoders (smoothed)
unsigned long wallPrevMs = 0;
uint8_t wallConfirmCnt = 0;
uint32_t wallConfirmSeq = 0;
unsigned long wallInvalidSince = 0;

// quarantine stop
enum StopPhase : uint8_t { QS_RAMP, QS_HOLD };
StopPhase stopPhase = QS_RAMP;
unsigned long stopHoldStartMs = 0;
uint32_t stopFiltSeq0 = 0;

unsigned long lastLogMs = 0, lastTelemetryMs = 0, lastMatrixMs = 0, lastWifiMs = 0;

constexpr int LOG_CAPACITY = 300;
LogRecord logBuffer[LOG_CAPACITY];
int logHead = 0, logCount = 0;

// ============================================================================
// 4. FORWARD DECLARATIONS
// ============================================================================
void motorUpdate(unsigned long now);
void stopMotors();
void setMotors(int16_t l, int16_t r);
void stopWithFault(const char *reason, bool hard = false);
void enterState(MissionState s);
bool tofBegin();
void enterQuarantineStop();
void enterQuarantineReady();
bool tofFilteredFresh(unsigned long now);
void updateBench(unsigned long now);
void printEncDiag();
float estYawDeg();

// ============================================================================
// 5. SMALL HELPERS
// ============================================================================
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

const char *stateName(MissionState s) {
  switch (s) {
    case S_BOOT:              return "BOOT";
    case S_INIT:              return "INIT";
    case S_READY_CHECK:       return "READY_CHECK";
    case S_START_COUNTDOWN:   return "START_COUNTDOWN";
    case S_INITIAL_FORWARD:   return "INITIAL_FORWARD";
    case S_PRETURN_DECEL:     return "PRETURN_DECEL";
    case S_LEFT_TURN:         return "LEFT_TURN";
    case S_TURN_SETTLE:       return "TURN_SETTLE";
    case S_WALL_APPROACH:     return "WALL_APPROACH";
    case S_WALL_FINE_APPROACH:return "WALL_FINE_APPROACH";
    case S_QUARANTINE_STOP:   return "QUARANTINE_STOP";
    case S_QUARANTINE_READY:  return "QUARANTINE_READY";
    case S_FAULT:             return "FAULT";
    case S_BENCH_TEST:        return "BENCH_TEST";
  }
  return "?";
}

// true from the start of the countdown until READY / FAULT
bool missionRunning() { return state >= S_START_COUNTDOWN && state <= S_QUARANTINE_STOP; }
// states in which the 2-minute match clock is enforced
bool matchClockState() { return state >= S_INITIAL_FORWARD && state <= S_QUARANTINE_STOP; }

unsigned long missionTimeMs(unsigned long now) {
  if (!missionStarted) return 0;
  return (missionClockRunning ? now : missionStopMs) - missionStartMs;
}

// ============================================================================
// 6. I2C: TCA9548A, ToF   (working code, gyro removed)
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

// ---- I2C bus recovery (unchanged) ----
constexpr uint8_t I2C_MAX_RECOVERIES = 12;
bool i2cRecover(const char *who) {
  stopMotors();
  if (i2cRecoveries >= I2C_MAX_RECOVERIES) { Serial.println(F("[I2C] too many bus recoveries, giving up")); return false; }
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
    if (ok && tofOnline) {
      ok = tcaSelect(tofChannel);
      if (ok) tof.startRanging();
    }
    lastTofPollMs = millis();
    if (ok) {
      tofFailCount = 0;
      Serial.println(F("[I2C] bus recovered"));
      lastRecoverMs = millis();
      return true;
    }
    delay(20);
  }
  Serial.println(F("[I2C] soft recovery failed - full re-init of the ToF (robot must stay still)"));
  tcaCurrent = 0xFF;
  bool all = true;
  if (tofOnline)  { tofOnline  = tofBegin();  all = all && tofOnline; }
  lastTofPollMs = millis();
  if (all) {
    tofFailCount = 0; lastRecoverMs = millis();
    Serial.println(F("[I2C] recovered by full re-init"));
    return true;
  }
  Serial.println(F("[I2C] recovery FAILED"));
  return false;
}

// ---- ToF sample handling: raw + rolling-median filter + valid/invalid counters ----
bool tofFilteredFresh(unsigned long now) {
  return tofOnline && tofHistN >= TOF_FILTER_MIN && (now - tofLastValidMs) <= TOF_STALE_MS;
}

void tofFilterFlush() { tofHistN = 0; tofHistPos = 0; }

static uint16_t tofMedianOfHistory() {
  uint16_t t[TOF_FILTER_N];
  for (uint8_t i = 0; i < tofHistN; i++) t[i] = tofHist[i];
  for (uint8_t i = 1; i < tofHistN; i++) {            // insertion sort (n <= 5)
    uint16_t v = t[i]; int8_t j = (int8_t)i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return t[tofHistN / 2];
}

void tofUpdate(unsigned long now) {
  if (!tofOnline || now - lastTofPollMs < TOF_POLL_MS) return;
  lastTofPollMs = now;
  if (!tcaSelect(tofChannel)) {
    if (++tofFailCount >= 5) {
      tofFailCount = 0;
      if (!i2cRecover("ToF")) { tofOnline = false; tofValid = false; Serial.println(F("TOF LOST: mux channel, bus could not be recovered")); }
    }
    return;
  }
  tofFailCount = 0;
  if (!tof.dataReady()) return;
  int16_t d = tof.distance();
  tof.clearInterrupt();
  tofSeq++;
  tofMs = now;
  tofRawMm = d;
  if (d >= (int16_t)TOF_MIN_MM && d <= (int16_t)TOF_MAX_MM) {
    tofValid = true;
    tofValidCount++;
    tofConsecInvalid = 0;
    if (tofHistN > 0 && (now - tofLastValidMs) > TOF_STALE_MS) tofFilterFlush();   // do not mix in old history after a gap
    if (tofHistN < TOF_FILTER_N) { tofHist[tofHistN++] = (uint16_t)d; tofHistPos = tofHistN % TOF_FILTER_N; }
    else { tofHist[tofHistPos] = (uint16_t)d; tofHistPos = (tofHistPos + 1) % TOF_FILTER_N; }
    tofFiltMm = tofMedianOfHistory();
    tofLastValidMs = now;
    tofFiltSeq++;
  } else {
    tofValid = false;
    tofInvalidCount++;
    tofConsecInvalid++;
  }
}

// ---- ToF fast mode / ROI register writes (unchanged) ----
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
static bool tofTuneEnabled = TOF_FAST_MODE;
static bool tofRoiEnabled  = TOF_NARROW_ROI;

static bool tofSetRoi(uint8_t x, uint8_t y) {
  if (x < 4) x = 4;
  if (x > 16) x = 16;
  if (y < 4) y = 4;
  if (y > 16) y = 16;
  Wire.beginTransmission(TOF_ADDR);
  Wire.write((uint8_t)0x00); Wire.write((uint8_t)0x7F);
  Wire.write((uint8_t)199);
  Wire.write((uint8_t)(((y - 1) << 4) | (x - 1)));
  return Wire.endTransmission() == 0;
}

void i2cScanReport() {
  Serial.println(F("[I2C] scan (expect 0x29 = VL53L1X):"));
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaCurrent = 0xFF;
    if (!tcaSelect(ch)) { Serial.print(F("  ch ")); Serial.print(ch); Serial.println(F(": mux select FAILED")); continue; }
    delay(5);
    Serial.print(F("  ch ")); Serial.print(ch); Serial.print(F(":"));
    bool any = false;
    for (uint8_t a = 1; a < 127; a++) {
      if (a == TCA_I2C_ADDR) continue;
      if (checkI2CPresence(a)) { Serial.print(F(" 0x")); Serial.print(a, HEX); any = true; }
    }
    Serial.println(any ? F("") : F(" nothing"));
  }
  tcaDisableAll();
}

// Reset the sensor, select its mux channel and start ranging. If the sensor is not on TCA_CH_TOF, every
// other mux channel is searched (existing discovery behaviour) and the channel that answers is used.
bool tofBegin() {
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    bool useReset = (attempt < 3);
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
    if (present && tof.begin(TOF_ADDR, &Wire)) {
      Wire.setClock(100000);
      tcaCurrent = 0xFF; tcaSelect(tofChannel);
      tof.setTimingBudget(TOF_TIMING_BUDGET_MS);
      bool tuned = tofTuneEnabled && tofSetInterMeasurementMs(TOF_INTERMEASURE_MS);
      if (tofTuneEnabled && !tuned) Serial.println(F("[BOOT] ToF fast-mode register write FAILED: running at the default ~10 Hz"));
      bool roiSet = tofRoiEnabled && tofSetRoi(TOF_ROI_X, TOF_ROI_Y);
      if (tofRoiEnabled && !roiSet) Serial.println(F("[BOOT] ToF ROI register write FAILED: using the full field of view"));
      if (tof.startRanging()) {
        if (tuned || roiSet) {
          uint8_t need = tuned ? 11 : 3;
          uint8_t n = 0;
          unsigned long t0 = millis();
          while (millis() - t0 < 600) {
            if (tof.dataReady()) { tof.distance(); tof.clearInterrupt(); n++; }
            delay(2);
          }
          Serial.print(tuned ? F("[BOOT] ToF fast mode: ") : F("[BOOT] ToF default mode: "));
          Serial.print(n); Serial.println(F(" samples in 0.6 s"));
          if (n < need) {
            Serial.println(F("[BOOT] fast mode / ROI did not work, re-initialising with the default settings"));
            tofTuneEnabled = false;
            tofRoiEnabled  = false;
            delay(100);
            continue;
          }
        }
        tofValid = false; tofFilterFlush(); tofFailCount = 0;
        Serial.print(F("[BOOT] ToF ACTUAL mux channel = ")); Serial.print(tofChannel);
        Serial.print(F(" (configured TCA_CH_TOF = ")); Serial.print(TCA_CH_TOF);
        Serial.println(tofChannel == TCA_CH_TOF ? F(", match)") : F(", DIFFERENT - update TCA_CH_TOF)"));
        return true;
      }
    }
    Serial.print(F("[BOOT] ToF init attempt ")); Serial.print(attempt);
    if (!muxOk) Serial.println(F(" failed (mux did not answer)"));
    else if (!present) Serial.println(F(" failed (no VL53L1X at 0x29 on ANY mux channel)"));
    else Serial.println(F(" failed (sensor answered but library init/start failed)"));
    delay(150);
  }
  i2cScanReport();
  Serial.println(F("[BOOT] ToF checklist: sensor VIN/GND, SDA/SCL on the mux SDx/SCx pair, XSHUT, loose Dupont wires"));
  return false;
}

// ============================================================================
// 7. MOTORS & ENCODERS  (MDD10A safety layer unchanged)
// ============================================================================
// Slew PWM, ramp to zero before reversing, then wait neutral. stopMotors() is the immediate zero.
MotorAxis motorL, motorR;
int16_t goalLeftPwm = 0, goalRightPwm = 0;
unsigned long lastMotorMs = 0;
constexpr unsigned long MOTOR_NEUTRAL_MS = 120;
constexpr int16_t MOTOR_RAMP_STEP = 8;

void stopMotors() {
  unsigned long now = millis();
  if (motorL.current != 0) motorL.zeroMs = now;
  if (motorR.current != 0) motorR.zeroMs = now;
  motorL.current = motorR.current = 0;
  goalLeftPwm = goalRightPwm = liveLeftPwm = liveRightPwm = 0;
  analogWrite(PIN_PWM_LEFT, 0);
  analogWrite(PIN_PWM_RIGHT, 0);
}

void setMotors(int16_t l, int16_t r) {
  goalLeftPwm = (int16_t)clampf(l, -MAX_PWM, MAX_PWM);
  goalRightPwm = (int16_t)clampf(r, -MAX_PWM, MAX_PWM);
}

static int16_t updateAxis(MotorAxis &m, int16_t goal, uint8_t pwmPin,
                          uint8_t dirPin, bool invert, unsigned long now) {
  int8_t wantedSign = goal < 0 ? -1 : 1;
  int16_t target = goal < 0 ? -goal : goal;
  if (goal != 0 && wantedSign != m.sign) {
    if (m.current > 0) target = 0;
    else if (now - m.zeroMs < MOTOR_NEUTRAL_MS) target = 0;
    else { m.sign = wantedSign; }
  }
  int16_t old = m.current;
  if (m.current < target) m.current += (target - m.current < MOTOR_RAMP_STEP ? target - m.current : MOTOR_RAMP_STEP);
  else if (m.current > target) m.current -= (m.current - target < MOTOR_RAMP_STEP ? m.current - target : MOTOR_RAMP_STEP);
  if (old > 0 && m.current == 0) m.zeroMs = now;
  if (m.current > 0) digitalWrite(dirPin, ((m.sign > 0) != invert) ? HIGH : LOW);
  analogWrite(pwmPin, m.current);
  return m.current * m.sign;
}

void motorUpdate(unsigned long now) {
  if (now - lastMotorMs < 10) return;
  lastMotorMs = now;
  liveLeftPwm = updateAxis(motorL, goalLeftPwm, PIN_PWM_LEFT, PIN_DIR_LEFT, invertLeftDir, now);
  liveRightPwm = updateAxis(motorR, goalRightPwm, PIN_PWM_RIGHT, PIN_DIR_RIGHT, invertRightDir, now);
  if (liveLeftPwm) encDirL = liveLeftPwm > 0 ? 1 : -1;
  if (liveRightPwm) encDirR = liveRightPwm > 0 ? 1 : -1;
}

// Ramp both outputs to zero through the normal slew limiter (blocks <= maxMs, used on the fault path).
void rampMotorsToZero(unsigned long maxMs) {
  setMotors(0, 0);
  unsigned long t0 = millis();
  while ((liveLeftPwm != 0 || liveRightPwm != 0) && millis() - t0 < maxMs) {
    motorUpdate(millis());
    delay(2);
  }
  stopMotors();
}

// positive = turn LEFT / counter-clockwise (heading increases)
void setTurnPwm(int16_t p) { setMotors(-p, p); }

// Straight drive without a gyro: keep the left and right WHEEL DISTANCES equal (encoders). If the right wheel has
// travelled more, slow the right wheel and speed up the left one. Forward only.
float wheelDistLeftMm();
float wheelDistRightMm();
void driveStraight(int16_t base) {
  float diff = wheelDistRightMm() - wheelDistLeftMm();           // + = right travelled more = drifting left
  float corr = clampf(STRAIGHT_KP * diff, -STRAIGHT_MAX_CORR, STRAIGHT_MAX_CORR);
  setMotors((int16_t)(base + corr), (int16_t)(base - corr));
}

// Speed-command ramp: limits how fast the commanded forward PWM may rise / fall.
void speedReset(bool kick = true) { speedCmd = 0.0f; lastSpeedMs = millis(); driveKickEnabled = kick; driveKickActive = false; }
int16_t speedRamp(float target, unsigned long now) {
  float dt = (now - lastSpeedMs) * 0.001f;
  lastSpeedMs = now;
  if (dt > 0.1f) dt = 0.1f;
  if (dt < 0.0f) dt = 0.0f;
  // Breakaway kick: a start from standstill first drives DRIVE_KICK_PWM for DRIVE_KICK_MS (counted once the outputs are
  // really there), then the speed command ramps DOWN to the requested band. Wheels that need more than the band PWM to
  // start would otherwise never move and trip the encoder-stall check.
  if (speedCmd == 0.0f && !driveKickActive && driveKickEnabled && DRIVE_KICK_MS > 0 && target >= (float)MIN_DRIVE_PWM) {
    driveKickActive = true; driveKickBegin = now; driveKickT0 = 0;
  }
  if (driveKickActive) {
    speedCmd = (float)DRIVE_KICK_PWM > target ? (float)DRIVE_KICK_PWM : target;
    int16_t la = liveLeftPwm < 0 ? -liveLeftPwm : liveLeftPwm;
    int16_t ra = liveRightPwm < 0 ? -liveRightPwm : liveRightPwm;
    int16_t hi = la > ra ? la : ra;
    if (driveKickT0 == 0 && hi >= DRIVE_KICK_PWM - 10) driveKickT0 = now;
    if ((driveKickT0 != 0 && now - driveKickT0 >= DRIVE_KICK_MS) || now - driveKickBegin > 600) driveKickActive = false;
    return (int16_t)speedCmd;
  }
  if (speedCmd < (float)MIN_DRIVE_PWM && target >= (float)MIN_DRIVE_PWM) speedCmd = (float)MIN_DRIVE_PWM;   // skip the stall band
  if (target > speedCmd) { speedCmd += DRIVE_ACCEL_PWM_PER_S * dt; if (speedCmd > target) speedCmd = target; }
  else                   { speedCmd -= DRIVE_DECEL_PWM_PER_S * dt; if (speedCmd < target) speedCmd = target; }
  return (int16_t)speedCmd;
}

// Raw ticks are stored so that (ticks * *_FORWARD_SIGN) is positive when the wheel goes forward.
void isrEncoderLeft() {
  int8_t q = (digitalRead(PIN_ENC_L_B) == HIGH) ? 1 : -1;
  int8_t d = encDirL;
  if ((int8_t)(q * LEFT_FORWARD_SIGN) == d) encAgreeL++; else encDisL++;
  encBSumL += q; encEdgesL++;
  if (ENCODER_USE_B_DIRECTION) encLeftTicks += q;
  else                         encLeftTicks += (int8_t)(d * LEFT_FORWARD_SIGN);
}
void isrEncoderRight() {
  int8_t q = (digitalRead(PIN_ENC_R_B) == HIGH) ? 1 : -1;
  int8_t d = encDirR;
  if ((int8_t)(q * RIGHT_FORWARD_SIGN) == d) encAgreeR++; else encDisR++;
  encBSumR += q; encEdgesR++;
  if (ENCODER_USE_B_DIRECTION) encRightTicks += q;
  else                         encRightTicks += (int8_t)(d * RIGHT_FORWARD_SIGN);
}

static const char *encVerdict(uint32_t agree, uint32_t dis) {
  uint32_t n = agree + dis;
  if (n < 20) return "no data";
  uint32_t pct = (agree * 100UL) / n;
  if (pct >= 90) return "B OK";
  if (pct <= 10) return "B reversed (flip FORWARD_SIGN, or this motor spins backward: flip its invert flag)";
  return "B dead/noisy (stuck level or loose wire)";
}
void printEncDiag() {
  uint32_t aL, dL, aR, dR;
  noInterrupts(); aL = encAgreeL; dL = encDisL; aR = encAgreeR; dR = encDisR; interrupts();
  Serial.print(F("[ENC] L agree/dis=")); Serial.print(aL); Serial.print('/'); Serial.print(dL);
  Serial.print(F(" ")); Serial.print(encVerdict(aL, dL));
  Serial.print(F(" | R agree/dis=")); Serial.print(aR); Serial.print('/'); Serial.print(dR);
  Serial.print(F(" ")); Serial.println(encVerdict(aR, dR));
}

// Distance driven since driveHealthReset(). A failing encoder only ever UNDER-counts, so when one wheel
// reads far less than the other, trust the larger one (existing behaviour).
// Signed forward distance of each wheel since driveHealthReset().
float wheelDistLeftMm() {
  long l; noInterrupts(); l = encLeftTicks; interrupts();
  return (float)((l - driveL0) * LEFT_FORWARD_SIGN) * MM_PER_TICK;
}
float wheelDistRightMm() {
  long r; noInterrupts(); r = encRightTicks; interrupts();
  return (float)((r - driveR0) * RIGHT_FORWARD_SIGN) * MM_PER_TICK;
}

float driveTravelMm() {
  long l, r;
  noInterrupts(); l = encLeftTicks; r = encRightTicks; interrupts();
  float lmm = (float)((l - driveL0) * LEFT_FORWARD_SIGN)  * MM_PER_TICK;
  float rmm = (float)((r - driveR0) * RIGHT_FORWARD_SIGN) * MM_PER_TICK;
  float hi = lmm > rmm ? lmm : rmm;
  float lo = lmm > rmm ? rmm : lmm;
  if (hi > 40.0f && lo < 0.7f * hi) return hi;
  return 0.5f * (lmm + rmm);
}

void driveHealthReset(unsigned long now) {
  noInterrupts(); driveL0 = encLeftTicks; driveR0 = encRightTicks; interrupts();
  prevL = driveL0; prevR = driveR0;
  lastLEdgeMs = lastREdgeMs = driveStartMs = now;
  imbalanceSinceMs = 0;
  encWarned = false;
}

// True once both encoders have been still for quietMs (used to confirm the robot really stopped).
bool encodersQuiet(unsigned long now, unsigned long quietMs) {
  long l, r;
  noInterrupts(); l = encLeftTicks; r = encRightTicks; interrupts();
  if (l != quietL || r != quietR) { quietL = l; quietR = r; lastEncMoveMs = now; }
  return (now - lastEncMoveMs) >= quietMs;
}
void encodersQuietReset(unsigned long now) {
  noInterrupts(); quietL = encLeftTicks; quietR = encRightTicks; interrupts();
  lastEncMoveMs = now;
}

// Call while driving. Returns false (after raising a fault) on a stall, a failed encoder or wheels out of step.
bool driveHealthy(unsigned long now, bool pivot = false) {
  long l, r;
  noInterrupts(); l = encLeftTicks; r = encRightTicks; interrupts();
  if (l != prevL) { lastLEdgeMs = now; prevL = l; }
  if (r != prevR) { lastREdgeMs = now; prevR = r; }
  bool lSilent = (now - driveStartMs > ENCODER_STALL_MS) && (now - lastLEdgeMs > ENCODER_STALL_MS);
  bool rSilent = (now - driveStartMs > ENCODER_STALL_MS) && (now - lastREdgeMs > ENCODER_STALL_MS);
  if (lSilent && rSilent) {
    stopWithFault("ENCODER STALL: both encoders silent while driving (robot blocked/stuck, or both encoder wires lost)");
    return false;
  }
  if (lSilent || rSilent) {
    if (ENCODER_SINGLE_FAIL_IS_FAULT) {
      if (lSilent) stopWithFault("ENCODER LEFT FAILURE: left encoder (D3/D7) not counting while the right one is");
      else         stopWithFault("ENCODER RIGHT FAILURE: right encoder (D2/D4) not counting while the left one is");
      return false;
    }
    if (!encWarned) {
      encWarned = true;
      Serial.print(F("[ENC] WARNING: ")); Serial.print(lSilent ? F("LEFT") : F("RIGHT"));
      Serial.println(F(" encoder not counting while the other one is - ignoring it"));
    }
  }
  float lmm = (float)((l - driveL0) * LEFT_FORWARD_SIGN)  * MM_PER_TICK;
  float rmm = (float)((r - driveR0) * RIGHT_FORWARD_SIGN) * MM_PER_TICK;
  if (ENCODER_USE_B_DIRECTION) {
    if (lmm < -10.0f && liveLeftPwm > 0)  { stopWithFault("ENCODER LEFT FAILURE: counts backwards, flip LEFT_FORWARD_SIGN"); return false; }
    if (rmm < -10.0f && liveRightPwm > 0) { stopWithFault("ENCODER RIGHT FAILURE: counts backwards, flip RIGHT_FORWARD_SIGN"); return false; }
  }
  // Wheels must keep pace with each other (straight: equal distance; pivot: equal and opposite distance).
  float imb = pivot ? fabsf(lmm + rmm) : fabsf(lmm - rmm);
  if (imb > IMBALANCE_FAULT_MM) {
    if (imbalanceSinceMs == 0) imbalanceSinceMs = now;
    else if (now - imbalanceSinceMs > IMBALANCE_FAULT_MS) {
      stopWithFault("MOVEMENT: wheels disagree by more than IMBALANCE_FAULT_MM (one wheel slipping, blocked or not driven)");
      return false;
    }
  } else imbalanceSinceMs = 0;
  return true;
}

// ============================================================================
// 8. FAULTS, STATE ENTRY
// ============================================================================
// Latches FAULT. Soft (default): ramp the motors to zero through the slew limiter first.
// hard = true: immediate PWM 0 (lost sensors / bus errors).
void stopWithFault(const char *reason, bool hard) {
  if (state == S_FAULT) { stopMotors(); return; }
  if (hard) stopMotors(); else setMotors(0, 0);
  state = S_FAULT;
  stateStartMs = millis();
  faultReason = reason;
  if (missionClockRunning) { missionClockRunning = false; missionStopMs = millis(); }
  Serial.println();
  Serial.print(F("FAULT: ")); Serial.println(reason);
  printEncDiag();
  if (!hard) rampMotorsToZero(400);
  stopMotors();
  Serial.println(F("[FAULT] PWM L=0 R=0, latched. Reset / power-cycle to restart."));
}

// NOTE: does NOT touch the motors; callers decide between ramp and immediate stop.
void enterState(MissionState s) {
  state = s;
  stateStartMs = millis();
  Serial.print(F("[STATE] ")); Serial.print(stateName(s));
  Serial.print(F("  t=")); Serial.print(missionTimeMs(millis()) / 1000.0f, 1);
  Serial.print(F("s  yaw~")); Serial.print(estYawDeg(), 1);
  Serial.print(F("  tofRaw=")); Serial.print(tofRawMm);
  Serial.print(F("  tofFilt=")); Serial.println(tofHistN >= TOF_FILTER_MIN ? (int)tofFiltMm : -1);
}

// ============================================================================
// 9. MISSION LOGIC
// ============================================================================
// ---- INITIAL_FORWARD ----
void enterInitialForward() {
  noInterrupts(); encLeftTicks = 0; encRightTicks = 0; interrupts();   // encoder movement reference = 0 at mission start
  driveHealthReset(millis());
  speedReset();
  moveStartMs = millis();
  brakePhase = false;
  enterState(S_INITIAL_FORWARD);
}

void updateInitialForward(unsigned long now) {
  if (now - moveStartMs > INITIAL_FORWARD_TIMEOUT_MS) { stopWithFault("MOVEMENT TIMEOUT: INITIAL_FORWARD did not finish in time"); return; }
  float trav = driveTravelMm();
  if (trav >= INITIAL_FORWARD_MM - PRETURN_DECEL_MM) {
    enterState(S_PRETURN_DECEL);                       // motion continues, no stop here
    return;
  }
  if (!driveHealthy(now)) return;
  driveStraight(speedRamp((float)INITIAL_CRUISE_PWM, now));
}

// ---- PRETURN_DECEL: profiled slow-down, then ramp to zero, then wait until the robot is really still ----
void startLeftTurn();

void updatePreturnDecel(unsigned long now) {
  if (now - moveStartMs > INITIAL_FORWARD_TIMEOUT_MS) { stopWithFault("MOVEMENT TIMEOUT: PRETURN_DECEL did not finish in time"); return; }
  float trav = driveTravelMm();

  if (!brakePhase) {
    float remain = (INITIAL_FORWARD_MM - PRETURN_STOP_EARLY_MM) - trav;
    if (remain <= 0.0f) {
      brakePhase = true; brakeStartMs = now;
      speedCmd = 0.0f;
      setMotors(0, 0);                                 // ramps through the slew limiter
      encodersQuietReset(now);
      return;
    }
    if (!driveHealthy(now)) return;
    float span = PRETURN_DECEL_MM - PRETURN_STOP_EARLY_MM;
    float f = span > 1.0f ? clampf(remain / span, 0.0f, 1.0f) : 0.0f;
    float pwm = (float)MIN_DRIVE_PWM + ((float)INITIAL_CRUISE_PWM - (float)MIN_DRIVE_PWM) * f;
    driveStraight(speedRamp(pwm, now));
    return;
  }

  setMotors(0, 0);
  bool stopped = (liveLeftPwm == 0 && liveRightPwm == 0);
  if (stopped && encodersQuiet(now, PRETURN_QUIET_MS)) {
    initialTravelMm = driveTravelMm();
    Serial.print(F("[ROUTE] initial forward travel = ")); Serial.print(initialTravelMm, 0);
    Serial.print(F(" mm (target ")); Serial.print(INITIAL_FORWARD_MM, 0); Serial.println(F(" mm)"));
    startLeftTurn();
  } else if (now - brakeStartMs > 1500) {
    stopMotors();                                      // never wait forever for a coast-down
  }
}

// ---- LEFT_TURN: encoder-arc pivot, no gyro ----
// Both wheels pivot (left backward, right forward). The turn is finished when the average wheel arc reaches
// TURN_TRACK_MM x angle / 2. Kick, then TURN_PWM, tapering to TURN_MIN_PWM over the last TURN_TAPER_FRACTION of the arc.
void startLeftTurn() {
  stopMotors();
  turnArcTargetMm = TURN_TRACK_MM * (3.14159265f / 180.0f) * QUARANTINE_LEFT_TURN_DEG * 0.5f;
  turnBrake = false;
  turnKickActive = true; turnKickBegin = millis(); turnKickT0 = 0;
  driveHealthReset(millis());                          // wheel distances are measured from here
  noInterrupts(); turnEncL0 = encLeftTicks; turnEncR0 = encRightTicks; turnAgreeL0 = encAgreeL; turnDisL0 = encDisL; turnAgreeR0 = encAgreeR; turnDisR0 = encDisR; interrupts();
  Serial.print(F("[TURN] left turn ")); Serial.print(QUARANTINE_LEFT_TURN_DEG, 1);
  Serial.print(F(" deg = wheel arc ")); Serial.print(turnArcTargetMm, 0);
  Serial.print(F(" mm (TURN_TRACK_MM=")); Serial.print(TURN_TRACK_MM, 0); Serial.println(F(")"));
  Serial.println(F("[TURN] WATCH THE ROBOT: it must rotate LEFT (counter-clockwise seen from above)."));
  enterState(S_LEFT_TURN);
}

static int8_t wheelBVerdict(uint32_t agree, uint32_t dis) {
  uint32_t n = agree + dis;
  if (n < 30) return 0;
  uint32_t pct = (agree * 100UL) / n;
  return pct >= 90 ? 1 : (pct <= 10 ? -1 : 2);
}
// Per-wheel encoder-B check (diagnostic only; it never stops the robot by itself).
void turnWheelDiagnosis() {
  uint32_t aL, dL, aR, dR;
  noInterrupts(); aL = encAgreeL - turnAgreeL0; dL = encDisL - turnDisL0; aR = encAgreeR - turnAgreeR0; dR = encDisR - turnDisR0; interrupts();
  int8_t vl = wheelBVerdict(aL, dL), vr = wheelBVerdict(aR, dR);
  Serial.print(F("[TURN] wheel check (encoder B vs command): LEFT ")); Serial.print(aL); Serial.print('/'); Serial.print(dL);
  Serial.print(vl == 1 ? F(" OK") : vl == -1 ? F(" REVERSED") : vl == 2 ? F(" NOISY") : F(" no data"));
  Serial.print(F(" | RIGHT ")); Serial.print(aR); Serial.print('/'); Serial.print(dR);
  Serial.println(vr == 1 ? F(" OK") : vr == -1 ? F(" REVERSED") : vr == 2 ? F(" NOISY") : F(" no data"));
  if (vl == -1 || vr == -1)
    Serial.println(F("[TURN] a wheel reads REVERSED: its motor flag, its DIR wire, or its encoder B wire is wrong. Run 'm' (summary) and 'd'."));
}

void finishTurn(unsigned long now) {
  stopMotors();
  float arc = 0.5f * (fabsf(wheelDistLeftMm()) + fabsf(wheelDistRightMm()));
  turnResultDeg = arc * 2.0f / TURN_TRACK_MM * (180.0f / 3.14159265f);
  tofFilterFlush();                                    // refill the ToF filter with post-turn samples only
  settleFiltSeq0 = tofFiltSeq;
  Serial.print(F("[TURN] done: wheel arc=")); Serial.print(arc, 0);
  Serial.print(F(" mm = ")); Serial.print(turnResultDeg, 1);
  Serial.print(F(" deg (encoder estimate)  time=")); Serial.print((now - stateStartMs) / 1000.0f, 1); Serial.println(F(" s"));
  printEncDiag();
  turnWheelDiagnosis();
  enterState(S_TURN_SETTLE);
}

void updateTurn(unsigned long now) {
  if (now - stateStartMs > TURN_TIMEOUT_MS) { turnWheelDiagnosis(); stopWithFault("MOVEMENT TIMEOUT: left turn did not reach the target arc in time"); return; }

  float arc = 0.5f * (fabsf(wheelDistLeftMm()) + fabsf(wheelDistRightMm()));
  float lo = fabsf(wheelDistLeftMm()) < fabsf(wheelDistRightMm()) ? fabsf(wheelDistLeftMm()) : fabsf(wheelDistRightMm());
  float hi = fabsf(wheelDistLeftMm()) < fabsf(wheelDistRightMm()) ? fabsf(wheelDistRightMm()) : fabsf(wheelDistLeftMm());
  if (hi > 40.0f && lo < 0.7f * hi) arc = hi;          // a failing encoder only under-counts: trust the larger one (existing rule)

  // Both wheels must really turn.
  if (now - stateStartMs > TURN_WATCHDOG_MS) {
    long l, r;
    noInterrupts(); l = encLeftTicks; r = encRightTicks; interrupts();
    long dl = labs(l - turnEncL0), dr = labs(r - turnEncR0);
    if (dl < TURN_MIN_TICKS || dr < TURN_MIN_TICKS) {
      Serial.print(F("[TURN] watchdog: ticks L=")); Serial.print(dl); Serial.print(F(" R=")); Serial.println(dr);
      turnWheelDiagnosis();
      stopWithFault("ENCODER STALL: wheels not moving during the turn (stall, no power or blocked) - check MDD10A battery / GND, run 'm'");
      return;
    }
  }

  if (turnBrake) {                                     // arc reached: ramp to zero, wait until the wheels are really still
    setMotors(0, 0);
    bool stopped = (liveLeftPwm == 0 && liveRightPwm == 0);
    if (stopped && encodersQuiet(now, PRETURN_QUIET_MS)) finishTurn(now);
    else if (now - turnBrakeMs > 1500) stopMotors();
    return;
  }
  if (!driveHealthy(now, true)) return;                // silent encoder / wheels out of step -> FAULT

  float remain = (turnArcTargetMm - TURN_STOP_EARLY_MM) - arc;
  if (remain <= 0.0f) { turnBrake = true; turnBrakeMs = now; encodersQuietReset(now); setMotors(0, 0); return; }

  float mag;
  if (turnKickActive) {                                // breakaway kick (heavy robot)
    mag = (float)TURN_KICK_PWM;
    int16_t la = liveLeftPwm < 0 ? -liveLeftPwm : liveLeftPwm;
    int16_t ra = liveRightPwm < 0 ? -liveRightPwm : liveRightPwm;
    if (turnKickT0 == 0 && la >= TURN_KICK_PWM && ra >= TURN_KICK_PWM) turnKickT0 = now;
    if ((turnKickT0 != 0 && now - turnKickT0 >= TURN_KICK_MS) || now - turnKickBegin > 600 || remain < 0.15f * turnArcTargetMm)
      turnKickActive = false;
  } else {
    float span = TURN_TAPER_FRACTION * turnArcTargetMm;
    float f = span > 1.0f ? clampf(remain / span, 0.0f, 1.0f) : 1.0f;     // 1 = full speed, 0 = end of taper
    mag = (float)TURN_MIN_PWM + ((float)TURN_PWM - (float)TURN_MIN_PWM) * f;
  }
  setTurnPwm((int16_t)mag);
}

// ---- TURN_SETTLE ----
void updateTurnSettle(unsigned long now) {
  setMotors(0, 0);
  unsigned long el = now - stateStartMs;
  bool filterReady = tofFilteredFresh(now);
  if (el < TURN_SETTLE_MS || !filterReady || !encodersQuiet(now, PRETURN_QUIET_MS)) {
    if (el > TURN_SETTLE_MS + TOF_PRIME_TIMEOUT_MS) {
      if (!filterReady) stopWithFault("TOF INVALID TOO LONG: no valid ToF readings after the turn (wall out of range or sensor blocked)");
      else stopWithFault("ENCODER STALL: wheels keep moving after the turn (coasting / noise on the encoder wires)");
    }
    return;
  }
  float need = (float)tofFiltMm - QUARANTINE_WALL_TARGET_MM;
  Serial.print(F("[ROUTE] turn ended at ")); Serial.print(turnResultDeg, 1);
  Serial.print(F(" deg (encoder estimate), wall at ")); Serial.print(tofFiltMm);
  Serial.print(F(" mm, to travel ~")); Serial.print(need, 0); Serial.println(F(" mm"));
  if (need < -WALL_DISTANCE_TOLERANCE_MM) {
    stopWithFault("WALL OVERSHOOT: wall is already closer than QUARANTINE_WALL_TARGET_MM before the approach starts (check TURN_TRACK_MM / turn angle / initial travel; a ToF that reads the same while the robot rotates is looking at part of the robot)");
    return;
  }
  if (need > MAX_WALL_APPROACH_MM) {
    stopWithFault("WALL NOT FOUND: wall farther than MAX_WALL_APPROACH_MM from the target (turned the wrong amount?)");
    return;
  }
  driveHealthReset(now);
  speedReset();
  wallStartMs = now;
  wallTimeoutMs = WALL_APPROACH_TIMEOUT_MS + (unsigned long)(need > 0.0f ? need : 0.0f) * WALL_TIMEOUT_MS_PER_MM;
  wallStartFiltMm = (float)tofFiltMm;
  wallConfirmCnt = 0;
  wallSpeedMmS = 0.0f; wallPrevTrav = 0.0f; wallPrevMs = now;
  wallConfirmSeq = tofFiltSeq;
  wallInvalidSince = 0;
  enterState(S_WALL_APPROACH);
}

// ---- WALL_APPROACH / WALL_FINE_APPROACH ----
int16_t wallBandPwm(float remain) {
  int16_t p;
  if (remain > WALL_MEDIUM_ZONE_MM)      p = WALL_FAST_PWM;
  else if (remain > WALL_SLOW_ZONE_MM)   p = WALL_MEDIUM_PWM;
  else if (remain > WALL_FINE_ZONE_MM)   p = WALL_SLOW_PWM;
  else                                   p = WALL_CRAWL_PWM;
  return p < WALL_MIN_PWM ? WALL_MIN_PWM : p;
}

void updateWall(unsigned long now) {
  if (now - wallStartMs > wallTimeoutMs) { stopWithFault("WALL APPROACH TIMEOUT: target not reached in time"); return; }
  float trav = driveTravelMm();
  if (trav > MAX_WALL_APPROACH_MM) {
    stopWithFault("WALL NOT FOUND: MAX_WALL_APPROACH_MM travelled without reaching the wall target");
    return;
  }

  // No trusted ToF reading: do not drive blind. Ramp down and wait; fault if it lasts.
  if (!tofFilteredFresh(now)) {
    setMotors(0, 0); speedCmd = 0.0f;
    if (wallInvalidSince == 0) wallInvalidSince = now;
    if (now - wallInvalidSince > TOF_INVALID_FAULT_MS) stopWithFault("TOF INVALID TOO LONG: no valid ToF reading during the wall approach");
    return;
  }
  wallInvalidSince = 0;
  if (!driveHealthy(now)) return;
  if (now - wallPrevMs >= 40) {                        // closing speed from the encoders, smoothed
    float v = (trav - wallPrevTrav) * 1000.0f / (float)(now - wallPrevMs);
    wallSpeedMmS += 0.3f * (v - wallSpeedMmS);
    wallPrevTrav = trav; wallPrevMs = now;
  }

  // The ToF must close on the wall while we drive (guards against a sensor looking at the robot / a side wall).
  if (trav > WALL_PROGRESS_AFTER_MM && (wallStartFiltMm - (float)tofFiltMm) < WALL_PROGRESS_MIN * trav) {
    Serial.print(F("[WALL] drove ")); Serial.print(trav, 0); Serial.print(F(" mm but ToF went "));
    Serial.print(wallStartFiltMm, 0); Serial.print(F(" -> ")); Serial.println(tofFiltMm);
    stopWithFault("WALL NOT FOUND: ToF is not closing while driving (wrong heading, sensor blocked, or wheel slip)");
    return;
  }

  // Confirm the stop condition on NEW filtered samples only.
  if (tofFiltSeq != wallConfirmSeq) {
    wallConfirmSeq = tofFiltSeq;
    float trigger = QUARANTINE_WALL_TARGET_MM + WALL_STOP_LEAD_MM + (wallSpeedMmS > 0.0f ? wallSpeedMmS : 0.0f) * WALL_STOP_LATENCY_S;
    if ((float)tofFiltMm <= trigger) wallConfirmCnt++;
    else wallConfirmCnt = 0;
  }
  if (wallConfirmCnt >= WALL_CONFIRM_COUNT) {
    Serial.print(F("[WALL] stop confirmed: filtered=")); Serial.print(tofFiltMm);
    Serial.print(F(" mm  closing speed=")); Serial.print(wallSpeedMmS, 0);
    Serial.print(F(" mm/s  trigger was ")); Serial.print(QUARANTINE_WALL_TARGET_MM + WALL_STOP_LEAD_MM + wallSpeedMmS * WALL_STOP_LATENCY_S, 0);
    Serial.println(F(" mm"));
    enterQuarantineStop(); return;
  }

  float remain = (float)tofFiltMm - QUARANTINE_WALL_TARGET_MM;
  if (state == S_WALL_APPROACH && remain <= WALL_FINE_ZONE_MM) enterState(S_WALL_FINE_APPROACH);
  driveStraight(speedRamp((float)wallBandPwm(remain), now));
}

// ---- QUARANTINE_STOP ----
void enterQuarantineStop() {
  setMotors(0, 0);                                     // ramp through the slew limiter
  speedCmd = 0.0f;
  stopPhase = QS_RAMP;
  encodersQuietReset(millis());
  enterState(S_QUARANTINE_STOP);
}

void updateQuarantineStop(unsigned long now) {
  setMotors(0, 0);
  if (stopPhase == QS_RAMP) {
    bool zero = (liveLeftPwm == 0 && liveRightPwm == 0);
    if (zero && encodersQuiet(now, PRETURN_QUIET_MS)) {
      stopPhase = QS_HOLD;
      stopHoldStartMs = now;
      tofFilterFlush();                                // measure the final distance from post-stop samples only
      stopFiltSeq0 = tofFiltSeq;
    } else if (now - stateStartMs > 1500) {
      stopMotors();
    }
    return;
  }
  bool filterReady = tofFilteredFresh(now);
  if (now - stopHoldStartMs < QUARANTINE_STOP_MS || !filterReady) {
    if (now - stopHoldStartMs > QUARANTINE_STOP_MS + TOF_PRIME_TIMEOUT_MS)
      stopWithFault("TOF INVALID TOO LONG: no valid ToF reading after stopping at the wall");
    return;
  }

  finalWallMm = (float)tofFiltMm;
  wallApproachTravelMm = driveTravelMm();
  float err = finalWallMm - QUARANTINE_WALL_TARGET_MM;
  Serial.print(F("[STOP] wall=")); Serial.print(finalWallMm, 0);
  Serial.print(F(" mm  target=")); Serial.print(QUARANTINE_WALL_TARGET_MM, 0);
  Serial.print(F(" mm  error=")); Serial.print(err, 1); Serial.println(F(" mm"));

  if (err < -WALL_DISTANCE_TOLERANCE_MM) {
    Serial.print(F("[OVERSHOOT] robot is ")); Serial.print(-err, 0); Serial.println(F(" mm closer to the wall than the target"));
    stopWithFault("WALL OVERSHOOT: stopped closer to the wall than QUARANTINE_WALL_TARGET_MM - tolerance (robot left stopped, no reverse)");
    return;
  }
  if (err > WALL_DISTANCE_TOLERANCE_MM) {
    Serial.print(F("[WARNING] stopped ")); Serial.print(err, 0);
    Serial.println(F(" mm SHORT of the target (lower WALL_STOP_LEAD_MM or WALL_CRAWL_PWM)"));
  }
  enterQuarantineReady();
}

// ---- QUARANTINE_READY (terminal) ----
void enterQuarantineReady() {
  stopMotors();
  missionClockRunning = false;
  missionStopMs = millis();
  enterState(S_QUARANTINE_READY);
  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("QUARANTINE ENTRY POSITION REACHED"));
  Serial.println(F("Robot stopped."));
  Serial.print(F("Wall distance: "));        Serial.print(finalWallMm, 0);          Serial.println(F(" mm"));
  Serial.print(F("Left turn (encoder estimate): ")); Serial.print(turnResultDeg, 1);     Serial.println(F(" deg"));
  Serial.print(F("Initial travel: "));       Serial.print(initialTravelMm, 0);      Serial.println(F(" mm"));
  Serial.print(F("Wall approach travel: ")); Serial.print(wallApproachTravelMm, 0); Serial.println(F(" mm"));
  Serial.println(F("READY FOR NEXT DEVELOPMENT STAGE"));
  Serial.println(F("================================"));
  Serial.println(F("(distances from encoders use the ESTIMATED wheel/ticks values; wall distance is the filtered front ToF)"));
  printEncDiag();
}

// ---- START_COUNTDOWN ----
void enterCountdown() {
  lastCountdownShown = (int8_t)(BOOT_DELAY_MS / 1000) + 1;
  stopMotors();
  enterState(S_START_COUNTDOWN);
  Serial.println(F("READY"));
}

void updateCountdown(unsigned long now) {
  stopMotors();
  unsigned long el = now - stateStartMs;
  int8_t left = (int8_t)((BOOT_DELAY_MS - (el < BOOT_DELAY_MS ? el : BOOT_DELAY_MS) + 999) / 1000);   // 5..0
  while (lastCountdownShown > left && lastCountdownShown > 1) {   // prints 5,4,3,2,1 (catches up after the blocking recalibration)
    lastCountdownShown--;
    Serial.println(lastCountdownShown);
  }
  if (el >= BOOT_DELAY_MS) {
    noInterrupts(); encLeftTicks = 0; encRightTicks = 0; interrupts();   // movement reference = start pose
    missionStarted = true;
    missionClockRunning = true;
    missionStartMs = millis();
    Serial.println(F("MISSION START"));
    if (INITIAL_FORWARD_MM <= 0.0f) {                  // no forward leg: turn left right away
      initialTravelMm = 0.0f;
      Serial.println(F("[ROUTE] no initial forward leg - turning left immediately"));
      startLeftTurn();
    } else {
      enterInitialForward();
    }
  }
}

// ---- state dispatcher ----
void missionUpdate(unsigned long now) {
  if (state == S_BENCH_TEST) { updateBench(now); return; }
  if (state == S_QUARANTINE_READY || state == S_FAULT) { setMotors(0, 0); return; }
  if (!missionRunning()) return;

  if (missionClockRunning && !BENCH_TEST_DISABLE_MATCH_TIMER && matchClockState()
      && now - missionStartMs >= MATCH_DURATION_MS) {
    stopWithFault("MISSION TIMEOUT: 120 s match time reached, motors stopped, no automatic resume");
    return;
  }

  if (state == S_START_COUNTDOWN) { updateCountdown(now); return; }

  if (now - lastControlMs < CONTROL_INTERVAL_MS) return;
  lastControlMs = now;

  switch (state) {
    case S_INITIAL_FORWARD:    updateInitialForward(now); break;
    case S_PRETURN_DECEL:      updatePreturnDecel(now);   break;
    case S_LEFT_TURN:          updateTurn(now);           break;
    case S_TURN_SETTLE:        updateTurnSettle(now);     break;
    case S_WALL_APPROACH:
    case S_WALL_FINE_APPROACH: updateWall(now);           break;
    case S_QUARANTINE_STOP:    updateQuarantineStop(now); break;
    default: break;
  }
}

// ============================================================================
// 10. READINESS CHECK
// ============================================================================
// Returns nullptr when the route configuration is valid, else the exact reason.
const char *routeConfigError() {
  if (!(INITIAL_FORWARD_MM == 0.0f || (INITIAL_FORWARD_MM >= 50.0f && INITIAL_FORWARD_MM <= 1500.0f))) return "ROUTE CONFIG INVALID: INITIAL_FORWARD_MM must be 0 (no leg) or 50..1500";
  if (INITIAL_FORWARD_MM > 0.0f && !(PRETURN_DECEL_MM > PRETURN_STOP_EARLY_MM && PRETURN_DECEL_MM < INITIAL_FORWARD_MM)) return "ROUTE CONFIG INVALID: need PRETURN_STOP_EARLY_MM < PRETURN_DECEL_MM < INITIAL_FORWARD_MM";
  if (!(QUARANTINE_LEFT_TURN_DEG >= 5.0f && QUARANTINE_LEFT_TURN_DEG <= 175.0f)) return "ROUTE CONFIG INVALID: QUARANTINE_LEFT_TURN_DEG outside 5..175";
  if (!(QUARANTINE_WALL_TARGET_MM >= 50.0f && QUARANTINE_WALL_TARGET_MM <= 1000.0f)) return "ROUTE CONFIG INVALID: QUARANTINE_WALL_TARGET_MM outside 50..1000";
  if (!(MAX_WALL_APPROACH_MM >= 50.0f && MAX_WALL_APPROACH_MM <= 2000.0f)) return "ROUTE CONFIG INVALID: MAX_WALL_APPROACH_MM outside 50..2000";
  if (!(WALL_FINE_ZONE_MM > 0.0f && WALL_FINE_ZONE_MM < WALL_SLOW_ZONE_MM && WALL_SLOW_ZONE_MM < WALL_MEDIUM_ZONE_MM)) return "ROUTE CONFIG INVALID: need 0 < WALL_FINE_ZONE_MM < WALL_SLOW_ZONE_MM < WALL_MEDIUM_ZONE_MM";
  if (!(MIN_DRIVE_PWM <= WALL_MIN_PWM && WALL_MIN_PWM <= WALL_CRAWL_PWM && WALL_CRAWL_PWM <= WALL_SLOW_PWM
        && WALL_SLOW_PWM <= WALL_MEDIUM_PWM && WALL_MEDIUM_PWM <= WALL_FAST_PWM && WALL_FAST_PWM <= MAX_PWM)) return "ROUTE CONFIG INVALID: need MIN_DRIVE_PWM <= WALL_MIN_PWM <= CRAWL <= SLOW <= MEDIUM <= FAST <= MAX_PWM";
  if (!(INITIAL_CRUISE_PWM >= MIN_DRIVE_PWM && INITIAL_CRUISE_PWM <= MAX_PWM)) return "ROUTE CONFIG INVALID: INITIAL_CRUISE_PWM outside MIN_DRIVE_PWM..MAX_PWM";
  if (!(TURN_MIN_PWM <= TURN_PWM && TURN_PWM <= TURN_KICK_PWM && TURN_KICK_PWM <= MAX_PWM && TURN_MIN_PWM > 0)) return "ROUTE CONFIG INVALID: need 0 < TURN_MIN_PWM <= TURN_PWM <= TURN_KICK_PWM <= MAX_PWM";
  if (!(TURN_TRACK_MM >= 80.0f && TURN_TRACK_MM <= 800.0f && TURN_KICK_MS > 0 && TURN_TAPER_FRACTION > 0.0f && TURN_TAPER_FRACTION <= 1.0f)) return "ROUTE CONFIG INVALID: TURN_TRACK_MM / TURN_KICK_MS / TURN_TAPER_FRACTION";
  if (!(WALL_CONFIRM_COUNT >= 1 && WALL_DISTANCE_TOLERANCE_MM > 0.0f && WALL_STOP_LEAD_MM >= 0.0f)) return "ROUTE CONFIG INVALID: WALL_CONFIRM_COUNT / WALL_DISTANCE_TOLERANCE_MM / WALL_STOP_LEAD_MM";
  if (!(TOF_FILTER_MIN >= 1 && TOF_FILTER_MIN <= TOF_FILTER_N)) return "ROUTE CONFIG INVALID: TOF_FILTER_MIN must be 1..TOF_FILTER_N";
  return nullptr;
}

// Runs with the robot stationary and PWM 0. Returns nullptr on PASS, else the exact failure.
const char *readinessCheck() {
  Serial.println(F("[READY_CHECK] starting"));

  const char *cfg = routeConfigError();
  if (cfg) return cfg;
  Serial.println(F("[READY_CHECK] route configuration: PASS"));

  if (!tofOnline) return "TOF NOT FOUND: VL53L1X did not initialise on any TCA channel";
  uint32_t seq0 = tofSeq;
  unsigned long t0 = millis();
  while (millis() - t0 < 1500 && (tofSeq - seq0) < 4) { tofUpdate(millis()); delay(2); }
  if ((tofSeq - seq0) < 4) return "TOF NOT FOUND: VL53L1X initialised but produced no measurements in 1.5 s";
  // The sensor must also return VALID ranges from the start pose (the field wall is within range there).
  t0 = millis();
  while (millis() - t0 < 1000 && !tofFilteredFresh(millis())) { tofUpdate(millis()); delay(2); }
  if (!tofFilteredFresh(millis())) return "TOF INVALID TOO LONG: VL53L1X is ranging but returns no valid distance at the start pose (blocked / out of range / wrong direction)";
  Serial.print(F("[READY_CHECK] ToF: PASS  samples=")); Serial.print(tofSeq - seq0);
  Serial.print(F(" raw=")); Serial.print(tofRawMm);
  Serial.print(F(" filtered=")); Serial.print(tofHistN >= 1 ? (int)tofFiltMm : -1);
  Serial.print(F(" valid/invalid=")); Serial.print(tofValidCount); Serial.print('/'); Serial.println(tofInvalidCount);

  // Encoders: they cannot be proven alive without moving. At standstill they must at least be quiet.
  // The real liveness test (each encoder must count while driving) runs in the first 0.5 s of INITIAL_FORWARD.
  long l0, r0;
  noInterrupts(); l0 = encLeftTicks; r0 = encRightTicks; interrupts();
  t0 = millis();
  while (millis() - t0 < 300) { tofUpdate(millis()); delay(2); }
  long l1, r1;
  noInterrupts(); l1 = encLeftTicks; r1 = encRightTicks; interrupts();
  if (labs(l1 - l0) > 3) return "ENCODER LEFT FAILURE: left encoder counts while the robot is stationary (noise / floating A wire on D3)";
  if (labs(r1 - r0) > 3) return "ENCODER RIGHT FAILURE: right encoder counts while the robot is stationary (noise / floating A wire on D2)";
  Serial.println(F("[READY_CHECK] encoders: quiet at standstill (liveness proven once driving starts)"));
  return nullptr;
}

// ============================================================================
// 11. LOGGING, TELEMETRY, LED MATRIX
// ============================================================================
// Yaw estimated from the wheels (left turn = +): (right distance - left distance) / track. Cumulative since MISSION START.
float estYawDeg() {
  long l, r; noInterrupts(); l = encLeftTicks; r = encRightTicks; interrupts();
  float lmm = (float)(l * LEFT_FORWARD_SIGN) * MM_PER_TICK, rmm = (float)(r * RIGHT_FORWARD_SIGN) * MM_PER_TICK;
  return (rmm - lmm) / TURN_TRACK_MM * (180.0f / 3.14159265f);
}
float targetYawNow() { return state >= S_LEFT_TURN ? QUARANTINE_LEFT_TURN_DEG : 0.0f; }

float travelNowMm() {
  if (state == S_LEFT_TURN || state == S_TURN_SETTLE) return 0.0f;      // pivot: wheels move opposite ways, 'travel' is meaningless
  return (state >= S_INITIAL_FORWARD && state <= S_QUARANTINE_STOP) ? driveTravelMm() : 0.0f;
}

float targetMmNow() {
  if (state <= S_PRETURN_DECEL) return INITIAL_FORWARD_MM;
  if (state >= S_TURN_SETTLE && state <= S_QUARANTINE_STOP) return QUARANTINE_WALL_TARGET_MM;
  return 0.0f;
}

void recordTelemetry(unsigned long now) {
  LogRecord &r = logBuffer[logHead];
  r.t       = now;
  r.state   = (uint8_t)state;
  r.tofRaw  = tofRawMm;
  r.tofFilt = tofHistN >= TOF_FILTER_MIN ? (int16_t)tofFiltMm : 0;
  r.head10  = (int16_t)lroundf(estYawDeg() * 10.0f);
  r.tgt10   = (int16_t)lroundf(targetYawNow() * 10.0f);
  r.lpwm    = liveLeftPwm;
  r.rpwm    = liveRightPwm;
  long l, rr;
  noInterrupts(); l = encLeftTicks; rr = encRightTicks; interrupts();
  r.encl = (int16_t)l;
  r.encr = (int16_t)rr;
  r.travel = (int16_t)travelNowMm();
  logHead = (logHead + 1) % LOG_CAPACITY;
  if (logCount < LOG_CAPACITY) logCount++;
}

void printTelemetry(unsigned long now) {
  long l, r;
  noInterrupts(); l = encLeftTicks; r = encRightTicks; interrupts();
  Serial.print(F("[T] ")); Serial.print(stateName(state));
  Serial.print(F(" t=")); Serial.print(missionTimeMs(now) / 1000.0f, 1);
  Serial.print(F("s enc=")); Serial.print(l); Serial.print('/'); Serial.print(r);
  Serial.print(F(" trav=")); Serial.print(travelNowMm(), 0);
  Serial.print(F("mm tgt=")); Serial.print(targetMmNow(), 0);
  Serial.print(F("mm yaw~")); Serial.print(estYawDeg(), 1);
  Serial.print(F(" tgtYaw=")); Serial.print(targetYawNow(), 1);
  Serial.print(F(" TOF_RAW="));
  if (!tofOnline) Serial.print(F("OFFLINE")); else Serial.print(tofRawMm);
  Serial.print(F(" TOF_FILTERED="));
  if (tofHistN >= TOF_FILTER_MIN) Serial.print(tofFiltMm); else Serial.print(F("--"));
  Serial.print(F(" ok/bad=")); Serial.print(tofValidCount); Serial.print('/'); Serial.print(tofInvalidCount);
  Serial.print(F(" pwm=")); Serial.print(liveLeftPwm); Serial.print('/'); Serial.print(liveRightPwm);
  Serial.print(F(" fault=")); Serial.println(faultReason);
}

static void drawSquare() {
  for (int i = 0; i < 8; i++) {
    ledFrame[0][2 + i] = 1; ledFrame[7][2 + i] = 1;
    ledFrame[i][2] = 1;     ledFrame[i][9] = 1;
  }
}

void renderLiveMatrix(unsigned long now) {
  memset(ledFrame, 0, sizeof(ledFrame));
  if (state == S_FAULT) {
    for (int i = 0; i < 8; i++) { ledFrame[i][2 + i] = 1; ledFrame[i][9 - i] = 1; }   // X
    matrix.renderBitmap(ledFrame, 8, 12);
    return;
  }
  if (state == S_QUARANTINE_READY) { drawSquare(); matrix.renderBitmap(ledFrame, 8, 12); return; }
  // row 3/4: filtered ToF bar (0..1200 mm)
  if (tofHistN >= TOF_FILTER_MIN) {
    int cols = (int)clampf((float)tofFiltMm / 1200.0f * 12.0f, 1.0f, 12.0f);
    for (int c = 0; c < cols; c++) { ledFrame[3][c] = 1; ledFrame[4][c] = 1; }
  }
  // row 7: countdown bar
  if (state == S_START_COUNTDOWN) {
    int cols = (int)((now - stateStartMs) * 12UL / BOOT_DELAY_MS);
    if (cols > 12) cols = 12;
    for (int c = 0; c < cols; c++) ledFrame[7][c] = 1;
  }
  matrix.renderBitmap(ledFrame, 8, 12);
}

// ============================================================================
// 12. WI-FI DASHBOARD (read-only; only serviced while the robot is NOT moving)
// ============================================================================
void printTenths(WiFiClient &c, int16_t v) {
  if (v < 0) { c.print('-'); v = -v; }
  c.print(v / 10); c.print('.'); c.print(v % 10);
}

void handleWirelessClients() {
  WiFiClient client = server.available();
  if (!client) return;

  String reqLine = "";
  unsigned long timeout = millis() + 10;
  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      char c = client.read();
      if (c == '\r' || c == '\n') { if (reqLine.length() > 0) break; }
      else if (reqLine.length() < 64) reqLine += c;
    }
  }
  while (client.available()) client.read();

  if (reqLine.indexOf("/download.csv") >= 0) {
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/csv\r\nContent-Disposition: attachment; filename=\"robot_log.csv\"\r\nConnection: close\r\n\r\n"));
    client.println(F("t_ms,state,tof_raw,tof_filtered,yaw_est_deg,target_yaw_deg,l_pwm,r_pwm,enc_l,enc_r,travel_mm"));
    int start = (logCount < LOG_CAPACITY) ? 0 : logHead;
    for (int i = 0; i < logCount; i++) {
      const LogRecord &r = logBuffer[(start + i) % LOG_CAPACITY];
      client.print(r.t); client.print(',');
      client.print(stateName((MissionState)r.state)); client.print(',');
      client.print(r.tofRaw); client.print(',');
      client.print(r.tofFilt); client.print(',');
      printTenths(client, r.head10); client.print(',');
      printTenths(client, r.tgt10); client.print(',');
      client.print(r.lpwm); client.print(',');
      client.print(r.rpwm); client.print(',');
      client.print(r.encl); client.print(',');
      client.print(r.encr); client.print(',');
      client.println(r.travel);
    }
    delay(2);
    client.stop();
    return;
  }

  if (reqLine.indexOf("/data") >= 0) {
    long l, rr;
    noInterrupts(); l = encLeftTicks; rr = encRightTicks; interrupts();
    char buf[420];
    int n = snprintf(buf, sizeof(buf),
      "{\"state\":\"%s\",\"t\":%lu,\"tofRaw\":%d,\"tofFilt\":%d,\"h10\":%ld,\"tgt10\":%ld,"
      "\"lpwm\":%d,\"rpwm\":%d,\"encl\":%ld,\"encr\":%ld,\"travel\":%ld,\"fault\":\"%s\"}",
      stateName(state), (unsigned long)(missionTimeMs(millis()) / 1000UL),
      (int)tofRawMm, tofHistN >= TOF_FILTER_MIN ? (int)tofFiltMm : 0,
      lroundf(estYawDeg() * 10.0f), lroundf(targetYawNow() * 10.0f),
      liveLeftPwm, liveRightPwm, l, rr, lroundf(travelNowMm()), faultReason);
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n"));
    if (n > 0) client.write((const uint8_t *)buf, (size_t)n);
  }
  else {
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"));
    client.print(F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Robot A</title>"));
    client.print(F("<style>body{font-family:system-ui,sans-serif;background:#0d1117;color:#f0f6fc;text-align:center;padding:12px}"));
    client.print(F(".c{background:#161b22;border-radius:10px;padding:12px;margin:8px auto;max-width:440px}.v{font-size:20px;font-weight:700;color:#3fb950}"));
    client.print(F("a{color:#58a6ff}</style></head><body><h3>ROBOT A - quarantine navigation</h3>"));
    client.print(F("<div class='c'><div id='st' class='v'>--</div><div id='ft'></div></div>"));
    client.print(F("<div class='c'>ToF raw / filtered: <span id='tf' class='v'>--</span></div>"));
    client.print(F("<div class='c'>Heading / target: <span id='hd' class='v'>--</span></div>"));
    client.print(F("<div class='c'>PWM L/R: <span id='pw' class='v'>--</span> Enc L/R: <span id='en' class='v'>--</span> Travel: <span id='tr' class='v'>--</span></div>"));
    client.print(F("<div class='c'><a href='/download.csv'>Download CSV log</a></div>"));
    client.print(F("<script>function $(i){return document.getElementById(i)}function poll(){fetch('/data').then(r=>r.json()).then(d=>{"));
    client.print(F("$('st').innerText=d.state+' t='+d.t+'s';$('ft').innerText='fault: '+d.fault;$('tf').innerText=d.tofRaw+' / '+d.tofFilt+' mm';"));
    client.print(F("$('hd').innerText=(d.h10/10).toFixed(1)+' / '+(d.tgt10/10).toFixed(1);$('pw').innerText=d.lpwm+'/'+d.rpwm;"));
    client.print(F("$('en').innerText=d.encl+'/'+d.encr;$('tr').innerText=d.travel+' mm'}).catch(e=>{})}setInterval(poll,500);poll();</script></body></html>"));
  }

  delay(1);
  client.stop();
}

// ============================================================================
// 12b. BENCH TOOLS (serial commands; only before MISSION START or when stopped)
// ============================================================================
//  'm' = motor test: each wheel alone, forward then backward, BENCH_RUN_MS at BENCH_MOTOR_PWM. Prints, per run, the
//        commanded direction, the A-edge count and what encoder B says the physical direction was. PUT THE ROBOT ON BLOCKS.
//  'e' = raw encoder monitor (MOTORS STAY OFF): prints the A/B pin levels and edge counts 4x per second; turn each wheel
//        by hand to prove the encoder wiring independently of the motors / MDD10A power.
//  'd' = DIR pin test (MOTORS STAY OFF, PWM 0): D6 and D10 toggle LOW/HIGH every 2 s; measure the MDD10A DIR1 / DIR2 terminals
//        with a multimeter (they must follow, 0 V <-> 5 V). Proves the DIR wires and pins independent of the motors.
//  any key while a bench tool runs aborts it. After a bench tool the robot stays stopped: reset to run the mission.
const char *dirName(int8_t d) { return d > 0 ? "FORWARD" : (d < 0 ? "BACKWARD" : "none"); }

void benchSnapshot(uint8_t wheel) {
  noInterrupts();
  if (wheel == 0) { benchTicks0 = (long)encEdgesL; benchB0 = encBSumL; }
  else            { benchTicks0 = (long)encEdgesR; benchB0 = encBSumR; }
  interrupts();
  benchStray = (uint32_t)benchTicks0 - benchLastEdges[wheel];   // edges since the last run = hand-turning or noise
  benchMidPrinted = false;
}

void benchFinish(const char *why) {
  stopMotors();
  benchMode = BM_NONE;
  Serial.print(F("[BENCH] ")); Serial.print(why);
  Serial.println(F(". Motors off. Reset to run the mission ('m' / 'e' / 'd' repeats a tool)."));
}

void benchStart(BenchMode m) {
  stopMotors();
  benchMode = m;
  benchStep = 0;
  benchT0 = millis();
  benchLastPrintMs = benchT0;
  for (uint8_t i = 0; i < 4; i++) benchResult[i] = "-";
  noInterrupts(); benchLastEdges[0] = encEdgesL; benchLastEdges[1] = encEdgesR; interrupts();
  state = S_BENCH_TEST;
  stateStartMs = benchT0;
  if (m == BM_MOTOR) {
    Serial.println(F("\n[BENCH] MOTOR TEST in 2 s: each wheel spins alone, FORWARD then BACKWARD, 1 s at 180 PWM."));
    Serial.println(F("[BENCH] ROBOT MUST BE ON BLOCKS. Send any key to abort. Watch which way each wheel really turns."));
  } else if (m == BM_DIR) {
    Serial.println(F("\n[BENCH] DIR PIN TEST (motors off, PWM 0): D6 and D10 go LOW, HIGH, LOW, HIGH, 2 s each."));
    Serial.println(F("[BENCH] Put a multimeter (DC volts) between MDD10A GND and the DIR1 terminal, then DIR2. Each must follow D6 / D10."));
  } else if (m == BM_ENC) {
    noInterrupts(); benchTicks0 = (long)encEdgesL; benchB0 = (long)encEdgesR; interrupts();   // reused as start counts
    Serial.println(F("\n[BENCH] ENCODER MONITOR (motors stay OFF). Turn each wheel by hand, forward and backward."));
    Serial.println(F("[BENCH] Expect A/B levels to toggle and the edge count of THAT wheel to rise. Send any key to stop."));
  }
}

void benchReport(uint8_t seg) {
  uint8_t wheel = seg / 2;
  int8_t cmd = (seg % 2 == 0) ? 1 : -1;
  long ticks, b;
  noInterrupts();
  ticks = (long)(wheel == 0 ? encEdgesL : encEdgesR);
  b = (wheel == 0 ? encBSumL : encBSumR);
  interrupts();
  long dA = ticks - benchTicks0, dB = b - benchB0;
  int8_t fs = (wheel == 0) ? LEFT_FORWARD_SIGN : RIGHT_FORWARD_SIGN;
  int8_t bDir = (dB > 0 ? 1 : (dB < 0 ? -1 : 0)) * fs;       // direction that encoder B reports, after FORWARD_SIGN
  const char *verdict;
  if (dA < 20) verdict = "NO TICKS: wheel not turning, or encoder A dead";
  else if (labs(dB) * 10 < dA * 6) verdict = "B DEAD/NOISY (B does not follow A)";
  else if (bDir == cmd) verdict = "B OK";
  else verdict = "B REVERSED";
  benchResult[seg] = verdict;
  benchBDir[seg] = bDir; benchA[seg] = dA;
  noInterrupts(); benchLastEdges[0] = encEdgesL; benchLastEdges[1] = encEdgesR; interrupts();
  Serial.print(F("[MOTOR TEST] ")); Serial.print(wheel == 0 ? F("LEFT  (D5/D6, invertLeftDir=") : F("RIGHT (D9/D10, invertRightDir="));
  Serial.print((wheel == 0 ? invertLeftDir : invertRightDir) ? 1 : 0);
  Serial.print(F(") commanded ")); Serial.print(dirName(cmd));
  Serial.print(F(": A edges=")); Serial.print(dA);
  Serial.print(F("  B sum=")); Serial.print(dB);
  Serial.print(F("  stray edges before run=")); Serial.print(benchStray);
  Serial.print(F("  B says ")); Serial.print(dirName(bDir));
  Serial.print(F(" (expected ")); Serial.print(dirName(cmd));
  Serial.print(F(")  -> ")); Serial.println(verdict);
}

// Per wheel: compare what encoder B reported for FORWARD and for BACKWARD. A healthy wheel reports opposite directions.
void benchSummary() {
  Serial.println(F("[MOTOR TEST] SUMMARY (per wheel, from the FORWARD run and the BACKWARD run):"));
  for (uint8_t w = 0; w < 2; w++) {
    int8_t bf = benchBDir[2 * w], bb = benchBDir[2 * w + 1];
    long af = benchA[2 * w], ab = benchA[2 * w + 1];
    Serial.print(F("  ")); Serial.print(w == 0 ? F("LEFT : ") : F("RIGHT: "));
    if (af < 20 || ab < 20) Serial.println(F("NO TICKS in at least one run -> wheel not turning (power / driver / motor lead) or encoder A dead."));
    else if (bf == 0 || bb == 0) Serial.println(F("B signal unreadable -> check the B wire."));
    else if (bf == bb) {
      bool bAlive = (benchBSeen[2 * w] == 3) || (benchBSeen[2 * w + 1] == 3);
      if (!bAlive) {
        Serial.print(w == 0 ? F("B PIN STUCK at one level (never toggled while the wheel spun) -> encoder B wire D7 dead / floating / shorted, or the encoder's B output is dead. ")
                            : F("B PIN STUCK at one level (never toggled while the wheel spun) -> encoder B wire D4 dead / floating / shorted, or the encoder's B output is dead. "));
        Serial.println(F("The motor itself may be fine: B is only a diagnostic here (counting uses A)."));
      } else {
        Serial.println(w == 0 ? F("DOES NOT REVERSE: B toggles, but the wheel spins the SAME way for FORWARD and BACKWARD -> DIR input not working: check D6 -> MDD10A DIR1, the connector, the MDD10A channel (use 'd' + a multimeter).")
                              : F("DOES NOT REVERSE: B toggles, but the wheel spins the SAME way for FORWARD and BACKWARD -> DIR input not working: check D10 -> MDD10A DIR2, the connector, the MDD10A channel (use 'd' + a multimeter)."));
      }
    }
    else if (bf == 1 && bb == -1) Serial.println(F("OK: reverses correctly and B agrees with the command."));
    else Serial.println(F("reverses, but B says the OPPOSITE of the command in BOTH runs -> if the wheel visibly turned the wrong way flip its invert flag, otherwise flip its *_FORWARD_SIGN."));
  }
}

void updateBench(unsigned long now) {
  if (benchMode == BM_NONE) { setMotors(0, 0); return; }

  if (benchMode == BM_ENC) {
    if (now - benchLastPrintMs >= 250) {
      benchLastPrintMs = now;
      uint32_t eL, eR;
      noInterrupts(); eL = encEdgesL; eR = encEdgesR; interrupts();
      Serial.print(F("[ENC RAW] L: A=")); Serial.print(digitalRead(PIN_ENC_L_A)); Serial.print(F(" B=")); Serial.print(digitalRead(PIN_ENC_L_B));
      Serial.print(F(" edges=")); Serial.print(eL - (uint32_t)benchTicks0);
      Serial.print(F(" | R: A=")); Serial.print(digitalRead(PIN_ENC_R_A)); Serial.print(F(" B=")); Serial.print(digitalRead(PIN_ENC_R_B));
      Serial.print(F(" edges=")); Serial.println(eR - (uint32_t)benchB0);
    }
    setMotors(0, 0);
    return;
  }

  if (benchMode == BM_DIR) {                           // PWM stays 0; only the DIR pins move
    setMotors(0, 0);
    unsigned long el = now - benchT0;
    uint8_t phase = (uint8_t)(el / 2000);              // 0 LOW, 1 HIGH, 2 LOW, 3 HIGH, then done
    if (phase >= 4) { benchFinish("DIR pin test complete"); return; }
    int lvl = (phase % 2) ? HIGH : LOW;
    if (benchStep != phase + 1) {
      benchStep = phase + 1;
      digitalWrite(PIN_DIR_LEFT, lvl); digitalWrite(PIN_DIR_RIGHT, lvl);
      Serial.print(F("[DIR TEST] D6 (-> MDD10A DIR1) = ")); Serial.print(digitalRead(PIN_DIR_LEFT) ? "HIGH" : "LOW");
      Serial.print(F("   D10 (-> MDD10A DIR2) = ")); Serial.print(digitalRead(PIN_DIR_RIGHT) ? "HIGH" : "LOW");
      Serial.println(F("   <- measure the MDD10A DIR1 / DIR2 terminals now (expect ~0 V / ~5 V)"));
    }
    return;
  }

  // BM_MOTOR: step 0 = start delay, then (run, pause) x 4 = L fwd, L back, R fwd, R back
  unsigned long el = now - benchT0;
  if (benchStep == 0) {
    setMotors(0, 0);
    if (el >= BENCH_START_DELAY_MS) {
      Serial.print(F("[MOTOR TEST] encoder pins now: L A=")); Serial.print(digitalRead(PIN_ENC_L_A)); Serial.print(F(" B=")); Serial.print(digitalRead(PIN_ENC_L_B));
      Serial.print(F(" | R A=")); Serial.print(digitalRead(PIN_ENC_R_A)); Serial.print(F(" B=")); Serial.println(digitalRead(PIN_ENC_R_B));
      for (uint8_t i = 0; i < 4; i++) benchBSeen[i] = 0;
      benchStep = 1; benchT0 = now; benchSnapshot(0);
    }
    return;
  }
  uint8_t seg = (benchStep - 1) / 2;
  bool running = (benchStep % 2) == 1;
  if (running) {
    int8_t cmd = (seg % 2 == 0) ? 1 : -1;
    int16_t p = (int16_t)(cmd * BENCH_MOTOR_PWM);
    if (seg / 2 == 0) setMotors(p, 0); else setMotors(0, p);
    benchBSeen[seg] |= (digitalRead(seg / 2 == 0 ? PIN_ENC_L_B : PIN_ENC_R_B) == HIGH) ? 2 : 1;   // B must toggle while the wheel spins
    if (!benchMidPrinted && el >= BENCH_RUN_MS / 2) {            // proves what the sketch is actually commanding
      benchMidPrinted = true;
      Serial.print(F("[MOTOR TEST]   mid-run live PWM L=")); Serial.print(liveLeftPwm);
      Serial.print(F(" R=")); Serial.println(liveRightPwm);
    }
    if (el >= BENCH_RUN_MS) { setMotors(0, 0); benchReport(seg); benchStep++; benchT0 = now; }
  } else {
    setMotors(0, 0);                                   // pause long enough for the slew layer's neutral delay
    if (el >= BENCH_PAUSE_MS) {
      if (seg >= 3) {
        printEncDiag();
        benchSummary();
        Serial.println(F("[MOTOR TEST] HOW TO READ THIS:"));
        Serial.println(F("  - Wheel physically turned the WRONG way for 'FORWARD'  -> flip that motor's invert flag (invertLeftDir / invertRightDir)."));
        Serial.println(F("  - Wheel turned the right way but B says REVERSED on BOTH its runs -> flip that wheel's *_FORWARD_SIGN."));
        Serial.println(F("  - B REVERSED on one run only, or B DEAD/NOISY -> check the B wire / connector of that encoder."));
        Serial.println(F("  - NO TICKS on ALL runs -> almost always no motor power: MDD10A B+/B- battery + switch, common GND Arduino<->MDD10A, PWM/DIR wires."));
        Serial.println(F("  - NO TICKS on one wheel only -> that motor lead / driver channel, or that encoder (VCC, GND, A). Use 'e' to test the encoder by hand."));
        Serial.println(F("  - stray edges > 0 while the motor was off -> wheel moved by hand, or electrical noise on the A wire."));
        benchFinish("motor test complete");
      } else { benchStep++; benchT0 = now; benchSnapshot((seg + 1) / 2); }
    }
  }
}

void serialCommands() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    if (state == S_BENCH_TEST && benchMode != BM_NONE) { benchFinish("aborted by serial input"); continue; }
    if (c == 'm' || c == 'e' || c == 'd') {
      bool ok = (state == S_START_COUNTDOWN || state == S_QUARANTINE_READY || state == S_FAULT || state == S_BENCH_TEST);
      if (!ok) { Serial.println(F("[CMD] refused: bench tools run only before MISSION START or when the robot is stopped")); continue; }
      benchStart(c == 'm' ? BM_MOTOR : (c == 'e' ? BM_ENC : BM_DIR));
    }
  }
}

// ============================================================================
// 13. SETUP
// ============================================================================
void printConfigSummary() {
  Serial.println(F("[CONFIG] values below are PLACEHOLDERS until calibrated on the real field:"));
  Serial.print(F("  INITIAL_FORWARD_MM=")); Serial.print(INITIAL_FORWARD_MM, 0);
  Serial.print(F("  QUARANTINE_LEFT_TURN_DEG=")); Serial.print(QUARANTINE_LEFT_TURN_DEG, 1);
  Serial.print(F("  QUARANTINE_WALL_TARGET_MM=")); Serial.print(QUARANTINE_WALL_TARGET_MM, 0);
  Serial.print(F("  MAX_WALL_APPROACH_MM=")); Serial.println(MAX_WALL_APPROACH_MM, 0);
  Serial.print(F("  PWM: cruise=")); Serial.print(INITIAL_CRUISE_PWM);
  Serial.print(F(" wall fast/med/slow/crawl=")); Serial.print(WALL_FAST_PWM); Serial.print('/'); Serial.print(WALL_MEDIUM_PWM);
  Serial.print('/'); Serial.print(WALL_SLOW_PWM); Serial.print('/'); Serial.print(WALL_CRAWL_PWM);
  Serial.print(F("  turn: TURN_TRACK_MM=")); Serial.print(TURN_TRACK_MM, 0);
  Serial.print(F("  PWM kick/cruise/min=")); Serial.print(TURN_KICK_PWM); Serial.print('/'); Serial.print(TURN_PWM); Serial.print('/'); Serial.print(TURN_MIN_PWM);
  Serial.print(F("  stop-early=")); Serial.print(TURN_STOP_EARLY_MM, 0); Serial.print(F(" mm  timeout=")); Serial.print(TURN_TIMEOUT_MS); Serial.println(F(" ms"));
  Serial.print(F("  invertLeftDir=")); Serial.print(invertLeftDir ? 1 : 0);
  Serial.print(F(" invertRightDir=")); Serial.print(invertRightDir ? 1 : 0);
  Serial.print(F(" LEFT_FORWARD_SIGN=")); Serial.print((int)LEFT_FORWARD_SIGN);
  Serial.print(F(" RIGHT_FORWARD_SIGN=")); Serial.print((int)RIGHT_FORWARD_SIGN);
  Serial.print(F(" ENCODER_USE_B_DIRECTION=")); Serial.println(ENCODER_USE_B_DIRECTION ? 1 : 0);
  Serial.println(F("  NO GYRO: turn angle comes from the wheel encoders only"));
  Serial.print(F("  wheel dia=")); Serial.print(WHEEL_DIAMETER_MM, 0);
  Serial.print(F(" mm, ticks/rev=")); Serial.print(ENCODER_TICKS_REV, 0);
  Serial.println(F("  (existing ESTIMATES, not verified)"));
  if (BENCH_TEST_DISABLE_MATCH_TIMER) Serial.println(F("  *** BENCH_TEST: 120 s MATCH TIMER DISABLED ***"));
  else Serial.println(F("  match timer: 120 s enforced"));
}

void setup() {
  // 1-2. Motor outputs to zero before anything else.
  pinMode(PIN_PWM_LEFT, OUTPUT);  pinMode(PIN_DIR_LEFT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT); pinMode(PIN_DIR_RIGHT, OUTPUT);
  digitalWrite(PIN_PWM_LEFT, LOW); digitalWrite(PIN_PWM_RIGHT, LOW);
  stopMotors();

  // 3. Serial (no wait for a PC: the robot must also boot on battery)
  Serial.begin(115200);
  state = S_BOOT;
  Serial.println(F("\n[BOOT] Robot A - quarantine navigation build. PWM L=0 R=0"));
  state = S_INIT;

  // 4. Motor control (DIR pins defined above; PWM already 0)
  digitalWrite(PIN_DIR_LEFT, LOW); digitalWrite(PIN_DIR_RIGHT, LOW);
  Serial.println(F("[INIT] motor control ready (MDD10A slew + neutral-delay safety active)"));

  // 5. Encoders
  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, RISING);
  Serial.println(F("[INIT] encoders attached (L: D3/D7, R: D2/D4)"));

  // TCRT5000 inputs stay configured but are not used by this mission.
  pinMode(PIN_TCRT_LEFT, INPUT); pinMode(PIN_TCRT_CENTRE, INPUT); pinMode(PIN_TCRT_RIGHT, INPUT);
  Serial.println(F("[INIT] TCRT5000 A1/A2/A3 configured as inputs - IGNORED in this version"));

  // 6. I2C / TCA
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(18000, true);
#endif
  delay(300);                                          // let the mux and sensor modules finish powering up
  if (checkI2CPresence(TCA_I2C_ADDR)) Serial.println(F("[INIT] TCA9548A found at 0x70"));
  else Serial.println(F("[INIT] WARNING: TCA9548A not answering at 0x70"));

  // 7. VL53L1X
  tofOnline = tofBegin();
  for (uint8_t extra = 1; extra <= 3 && !tofOnline; extra++) {
    Serial.print(F("[BOOT] ToF still offline - waiting and retrying (")); Serial.print(extra); Serial.println(F("/3)"));
    delay(700);
    tofOnline = tofBegin();
  }
  Serial.println(tofOnline ? F("[BOOT] front VL53L1X online") : F("[BOOT] front VL53L1X OFFLINE - check TCA_CH_TOF and wiring"));

  // 9. LED matrix / logging / WiFi (as in the existing sketch)
  matrix.begin();
  if (ENABLE_WIFI) {
    Serial.println(F("[WIFI] Starting hotspot..."));
    WiFi.beginAP(WIFI_SSID, WIFI_PASS);
    server.begin();
    Serial.print(F("[WIFI] http://")); Serial.println(WiFi.localIP());
  }
  printConfigSummary();
  Serial.println(F("[BOOT] serial commands before MISSION START: 'm' = motor/encoder-B test (robot on blocks), 'e' = raw encoder monitor (motors off), 'd' = DIR pin test (motors off)"));

  // 10-11. Readiness; any failure latches FAULT without moving.
  state = S_READY_CHECK;
  const char *why = readinessCheck();
  lastLoopMs = millis();
  if (why) { stopWithFault(why, true); return; }
  Serial.println(F("[READY_CHECK] ALL PASS"));

  // 5-second stationary countdown starts now, mission follows automatically (no button; D8 unused).
  enterCountdown();
}

// ============================================================================
// 14. MAIN LOOP
// ============================================================================
static inline void noteDur(unsigned long &mx, unsigned long t0) { unsigned long d = millis() - t0; if (d > mx) mx = d; }

void loop() {
  unsigned long now = millis();

  // loop-stall watchdog (unchanged): if the loop was blocked while the motors were on, kill them, recover the bus, carry on.
  unsigned long gap = now - lastLoopMs;
  if (gap > LOOP_WATCHDOG_MS && (liveLeftPwm != 0 || liveRightPwm != 0) && missionRunning()) {
    stopMotors();
    Serial.print(F("[STALL] loop blocked ")); Serial.print(gap);
    Serial.print(F(" ms. slowest calls (ms): tof=")); Serial.print(durTof);
    Serial.print(F(" mission=")); Serial.print(durMission);
    Serial.print(F(" wifi=")); Serial.print(durWifi);
    Serial.print(F(" matrix=")); Serial.print(durMatrix);
    Serial.print(F(" serial=")); Serial.println(durTelem);
    durTof = durMission = durWifi = durMatrix = durTelem = 0;
    unsigned long tPause = millis();
    if (++loopStalls > LOOP_STALL_MAX) stopWithFault("MOVEMENT TIMEOUT: control loop stalled repeatedly", true);
    else if (!i2cRecover("loop stall")) stopWithFault("TOF NOT FOUND: control loop stalled and the I2C bus could not be recovered", true);
    if (state != S_FAULT) {
      unsigned long t = millis();
      unsigned long paused = gap + (t - tPause);
      // stage timeouts do not count the pause; the 120 s MATCH clock deliberately does
      stateStartMs += paused; moveStartMs += paused; wallStartMs += paused; wallPrevMs += paused; turnBrakeMs += paused; turnKickBegin += paused;
      driveStartMs = lastLEdgeMs = lastREdgeMs = t;
      imbalanceSinceMs = 0;
      lastControlMs = t;
      speedCmd = 0.0f; lastSpeedMs = t;
      Serial.println(F("[STALL] recovered, resuming"));
    }
    now = millis();
  }
  lastLoopMs = now;

  unsigned long t0 = millis();
  tofUpdate(now);       noteDur(durTof, t0);
  now = millis();

  if (missionRunning() && !tofOnline) stopWithFault("TOF NOT FOUND: ToF lost during mission", true);

  t0 = millis();
  serialCommands();
  t0 = millis();
  missionUpdate(now);   noteDur(durMission, t0);
  motorUpdate(millis());

  if (state > S_INIT && now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    recordTelemetry(now);
  }

  // server.available() is slow on the R4 WiFi: only service it while the robot is not running.
  if (ENABLE_WIFI && !missionRunning() && now - lastWifiMs >= 40) {
    lastWifiMs = now;
    t0 = millis(); handleWirelessClients(); noteDur(durWifi, t0);
  }

  if (now - lastMatrixMs >= MATRIX_INTERVAL_MS) { lastMatrixMs = now; t0 = millis(); renderLiveMatrix(now); noteDur(durMatrix, t0); }
  unsigned long telemInterval = missionRunning() ? TELEMETRY_INTERVAL_MS : 2000UL;
  if (state != S_BENCH_TEST && now - lastTelemetryMs >= telemInterval) { lastTelemetryMs = now; t0 = millis(); printTelemetry(now); noteDur(durTelem, t0); }
}
