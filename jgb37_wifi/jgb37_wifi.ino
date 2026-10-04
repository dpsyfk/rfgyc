/*
 * JGB37-520 differential robot - WIRELESS calibration + precise motion
 * Arduino UNO R4 WiFi + Cytron MDD10A
 * ---------------------------------------------------------------------
 * HOW TO USE
 *  1. Upload this sketch, power the robot from its battery, put it on the floor.
 *  2. On your phone join Wi-Fi  "JGB37-Robot"  (password 12345678).
 *     (Turn mobile data OFF if the phone keeps dropping the network.)
 *  3. Open a browser:  http://192.168.4.1
 *  4. Use the page: calibration steps, manual moves, path runner, STOP button.
 *
 * WIRING (matches the pin numbers below)
 *   MDD10A  LEFT : PWM1 <- D10  DIR1 <- D9      RIGHT: PWM2 <- D5  DIR2 <- D6      common GND (Arduino GND <-> MDD10A GND)
 *   Encoder LEFT : A -> D3  B -> D7             Encoder RIGHT: A -> D2  B -> D4      VCC -> 5V, GND
 *   Motor battery -> MDD10A B+ / B-  (must be ON, otherwise the wheels never move)
 *
 * PATH RUNNER TOKENS (comma separated, up to MAX_STEPS steps):
 *   F<mm> forward   B<mm> back   T<deg> turn (+left)   Z1 = SCAN the area (see below)
 *   e.g.  F85,T90,F380,Z1   = 85 mm forward, 90 deg left, 380 mm forward, then scan
 *
 * SCAN (Z1)  - coverage sweep of a W x H area (default 280 x 280 mm) with the robot CENTRE, in the robot's own frame at
 *   the moment Z1 starts: x = forward (depth H), y = left (width W, centred on the robot + offset).
 *   The robot first moves sideways to the edge that is closest to its centre line, then drives lanes along its forward
 *   axis (forward H, shift one lane pitch sideways, back H, ...). The last lane is placed exactly on the far edge
 *   (the pitch is shrunk to fit). Optionally it returns to the pose where the scan started, so later tokens continue
 *   from a known place. All scan numbers are editable on the page ("Scan area (Z)") and saved with SAVE.
 *
 * Settings saved earlier in EEPROM (cpr, motor limits) are kept.
 *
 * SAFETY
 *  - Motion is non-blocking: the STOP button works at any time.
 *  - Stall guard: a wheel that is driven but not turning aborts the move.
 *  - Every move is bounded in length and time.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <WiFiS3.h>
#include <math.h>

// Types used in function signatures MUST be declared before any function: the Arduino IDE inserts its automatic
// prototypes at the top of the file, so a struct defined further down is 'not declared' there.
struct DirGuard { int8_t dir = 0; unsigned long zeroSince = 0; };
struct ScanInfo { int steps; int lanes; float dist; float timeS; float wc; float lc; float pitch; };

// ============================ USER CONFIG ===================================
#define AP_SSID "JGB37-Robot"
#define AP_PASS "12345678"          // at least 8 characters

#define ENC_L_A 3      // D2 / D3 are the only external-interrupt pins on UNO R4
#define ENC_L_B 7
#define ENC_R_A 2
#define ENC_R_B 4

#define M_L_PWM 10
#define M_L_DIR 9
#define M_R_PWM 5
#define M_R_DIR 6

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define MOTOR_L_SIGN  1
#define MOTOR_R_SIGN  1
#define ENC_L_SIGN   -1
#define ENC_R_SIGN    1

#define DEFAULT_WHEEL_DIAM_MM   65.0f
#define DEFAULT_TRACK_MM       150.0f
#define MOTOR_GEAR_RATIO        30.0f
#define DEFAULT_CPR            (11.0f * 2.0f * MOTOR_GEAR_RATIO)

#define PWM_MAX 255
#define CONTROL_DT_MS 10
#define MAX_STEPS 128          // a scan with a small lane pitch needs many steps

// ============================ SETTINGS ======================================
struct Settings {                     // layout unchanged -> old EEPROM data still valid
  uint32_t magic;
  float wheelDiam, track;
  float cprL, cprR;
  float corrL, corrR;
  float deadL, deadR;
  float kvL, kvR;
  float vmax;
  float kp, ki;
};
static const uint32_t MAGIC = 0x4A423337;
Settings S, Sprev;
bool haveBackup = false;

// ---- scan area (Z) settings, stored separately in EEPROM (the Settings layout above is unchanged) ----
struct ScanCfg { uint32_t magic; float w, h, off, lane, inset, v; uint8_t ret; uint8_t pad[3]; };
static const uint32_t SCAN_MAGIC = 0x5A534331;      // "ZSC1"
#define SCAN_EEPROM_ADDR 128
ScanCfg Z, Zprev;
void scanDefaults() { Z.magic = SCAN_MAGIC; Z.w = 280; Z.h = 280; Z.off = 0; Z.lane = 70; Z.inset = 0; Z.v = 120; Z.ret = 1; Z.pad[0] = Z.pad[1] = Z.pad[2] = 0; }
void scanClamp() {
  Z.w = constrain(Z.w, 20.0f, 1500.0f);   Z.h = constrain(Z.h, 20.0f, 1500.0f);
  Z.off = constrain(Z.off, -1000.0f, 1000.0f);
  Z.lane = constrain(Z.lane, 15.0f, 1000.0f);
  Z.inset = constrain(Z.inset, 0.0f, 200.0f);
  Z.v = constrain(Z.v, 20.0f, 400.0f);
  Z.ret = Z.ret ? 1 : 0;
}
void scanSave() { EEPROM.put(SCAN_EEPROM_ADDR, Z); }
void scanLoad() {
  EEPROM.get(SCAN_EEPROM_ADDR, Z);
  if (Z.magic != SCAN_MAGIC || !isfinite(Z.w) || !isfinite(Z.h) || !isfinite(Z.off) || !isfinite(Z.lane) || !isfinite(Z.inset) || !isfinite(Z.v)) scanDefaults();
  scanClamp();
}
float vCruise = 150, accel = 300, tVel = 100;   // mm/s, mm/s^2, mm/s (turns)

void defaults() {
  S.magic = MAGIC;
  S.wheelDiam = DEFAULT_WHEEL_DIAM_MM;
  S.track = DEFAULT_TRACK_MM;
  S.cprL = S.cprR = DEFAULT_CPR;
  S.corrL = S.corrR = 1.0f;
  S.deadL = S.deadR = 40;
  S.kvL = S.kvR = 0.5f;
  S.vmax = 200;
  S.kp = 3.0f;
  S.ki = 0.5f;
}
void saveSettings() { EEPROM.put(0, S); }
void loadSettings() {
  EEPROM.get(0, S);
  if (S.magic != MAGIC || !isfinite(S.track) || !isfinite(S.cprL)) defaults();
}
void backup() { Sprev = S; Zprev = Z; haveBackup = true; }

String lastMsg = "Ready";

// ============================ ENCODERS ======================================
volatile long cntL = 0, cntR = 0;
void IRAM_ATTR isrL() {
  bool a = digitalRead(ENC_L_A), b = digitalRead(ENC_L_B);
  cntL += ((a == b) ? 1 : -1) * ENC_L_SIGN;
}
void IRAM_ATTR isrR() {
  bool a = digitalRead(ENC_R_A), b = digitalRead(ENC_R_B);
  cntR += ((a == b) ? 1 : -1) * ENC_R_SIGN;
}
long getL() { noInterrupts(); long v = cntL; interrupts(); return v; }
long getR() { noInterrupts(); long v = cntR; interrupts(); return v; }
void zeroEnc() { noInterrupts(); cntL = cntR = 0; interrupts(); }
float mmPerCountL() { return (PI * S.wheelDiam / S.cprL) * S.corrL; }
float mmPerCountR() { return (PI * S.wheelDiam / S.cprR) * S.corrR; }

// ============================ MOTORS ========================================
// A wheel never changes direction while it is driven: it must pass through zero output for REVERSE_NEUTRAL_MS first,
// and DIR is only written when PWM is about to be non-zero (MDD10A protection).
#define REVERSE_NEUTRAL_MS 60
DirGuard guardL, guardR;
void driveMotor(int pwmPin, int dirPin, int sign, float cmd, DirGuard &g) {
  int p = (int)constrain(cmd * sign, -PWM_MAX, PWM_MAX);
  int8_t want = p > 0 ? 1 : (p < 0 ? -1 : 0);
  unsigned long now = millis();
  if (want != 0 && g.dir != 0 && want != g.dir) {
    if (g.zeroSince == 0) g.zeroSince = now;
    if (now - g.zeroSince >= REVERSE_NEUTRAL_MS) { g.dir = want; g.zeroSince = 0; }
    else { p = 0; want = 0; }
  }
  if (want == 0) {
    analogWrite(pwmPin, 0);
    if (g.zeroSince == 0) g.zeroSince = now;
    return;
  }
  g.dir = want; g.zeroSince = 0;
  digitalWrite(dirPin, p > 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(p));
}
void setMotors(float l, float r) {
  driveMotor(M_L_PWM, M_L_DIR, MOTOR_L_SIGN, l, guardL);
  driveMotor(M_R_PWM, M_R_DIR, MOTOR_R_SIGN, r, guardR);
}
void stopMotors() { setMotors(0, 0); }

// ============================ ODOMETRY ======================================
float ox = 0, oy = 0, oth = 0;
long lastOL = 0, lastOR = 0;
void odomReset() { ox = oy = oth = 0; lastOL = getL(); lastOR = getR(); }
void odomUpdate() {
  long l = getL(), r = getR();
  float dL = (l - lastOL) * mmPerCountL();
  float dR = (r - lastOR) * mmPerCountR();
  lastOL = l; lastOR = r;
  float d = (dL + dR) * 0.5f;
  float dth = (dR - dL) / S.track;
  ox += d * cosf(oth + dth * 0.5f);
  oy += d * sinf(oth + dth * 0.5f);
  oth += dth;
}

// ============================ MOTION (non-blocking) =========================
float pD, pV, pA, pTa, pTc, pT;
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

bool mAct = false;
float mFL, mFR, mIL, mIR;
long mSL, mSR, mLastCL, mLastCR;
unsigned long mT0, mTick, mMovL, mMovR;

// step queue
float qL[MAX_STEPS], qR[MAX_STEPS], qV[MAX_STEPS];
uint8_t qLane[MAX_STEPS];                 // >0 on the lane drives of a scan (progress display)
int qN = 0, qI = 0;
bool qOver = false;                       // a step did not fit in the queue
uint8_t curLane = 0, scanLanesTotal = 0;
bool qWait = false;
unsigned long qNextAt = 0;

void startMove(float dL, float dR, float v) {
  float D = max(fabsf(dL), fabsf(dR));
  if (D < 0.01f) return;
  mFL = dL / D; mFR = dR / D;
  profileInit(D, min(v, S.vmax), accel);
  mSL = getL(); mSR = getR();
  mLastCL = mSL; mLastCR = mSR;
  mIL = mIR = 0;
  mT0 = mTick = mMovL = mMovR = millis();
  mAct = true;
}

void abortAll(const char *why) {
  mAct = false; qWait = false; qN = qI = 0; curLane = 0; scanLanesTotal = 0;
  stopMotors();
  lastMsg = why;
}

void finishStep() {
  stopMotors();
  mAct = false;
  odomUpdate();
  if (qI < qN) { qWait = true; qNextAt = millis() + 250; }
  else { qN = qI = 0; curLane = 0; lastMsg = scanLanesTotal ? "Path + scan finished" : "Move finished"; scanLanesTotal = 0; }
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
  float posL = (cl - mSL) * mmPerCountL();
  float posR = (cr - mSR) * mmPerCountR();
  float eL = s * mFL - posL, eR = s * mFR - posR;
  mIL = constrain(mIL + eL * dt, -50, 50);
  mIR = constrain(mIR + eR * dt, -50, 50);

  float uL = S.kvL * v * mFL + S.kp * eL + S.ki * mIL;
  float uR = S.kvR * v * mFR + S.kp * eR + S.ki * mIR;
  if (fabsf(uL) > 1) uL += (uL > 0 ? S.deadL : -S.deadL);
  if (fabsf(uR) > 1) uR += (uR > 0 ? S.deadR : -S.deadR);
  setMotors(uL, uR);

  // stall guard
  if (cl != mLastCL) { mLastCL = cl; mMovL = now; }
  if (cr != mLastCR) { mLastCR = cr; mMovR = now; }
  if ((fabsf(uL) > 60 && now - mMovL > 800) || (fabsf(uR) > 60 && now - mMovR > 800)) {
    abortAll("ABORT: wheel driven but not turning (check encoder / motor / blocked wheel)");
    return;
  }

  if (t > pT && fabsf(eL) < 1.0f && fabsf(eR) < 1.0f) finishStep();   // 1 mm (0.3 mm made every step creep for ~2 s)
  else if (t > pT + 2.0f) finishStep();
}

bool queueStepL(float dL, float dR, float v, uint8_t lane) {
  if (qN < MAX_STEPS) { qL[qN] = dL; qR[qN] = dR; qV[qN] = v; qLane[qN] = lane; qN++; return true; }
  qOver = true;
  return false;
}
void queueStep(float dL, float dR, float v) { queueStepL(dL, dR, v, 0); }
void queueStraight(float mm) { queueStep(mm, mm, vCruise); }
void queueTurn(float deg) {
  float arc = (deg * PI / 180.0f) * S.track * 0.5f;
  queueStep(-arc, arc, tVel);
}
void queueTick() {
  if (mAct || !qWait || millis() < qNextAt) return;
  qWait = false;
  if (qI < qN) {
    curLane = qLane[qI];
    if (curLane) lastMsg = "Scan lane " + String(curLane) + "/" + String(scanLanesTotal);
    startMove(qL[qI], qR[qI], qV[qI]);
    qI++;
    if (!mAct) {                          // zero-length step: go straight to the next one
      if (qI < qN) { qWait = true; qNextAt = millis(); }
      else { qN = qI = 0; curLane = 0; lastMsg = scanLanesTotal ? "Path + scan finished" : "Move finished"; scanLanesTotal = 0; }
    }
  } else { qN = qI = 0; curLane = 0; }
}
void beginQueue() { qI = 0; qWait = (qN > 0); qNextAt = millis(); }

// ============================ SCAN PLANNER (Z) ==============================
// Plans in the robot's frame at the moment Z starts: x forward, y LEFT, heading 0 = forward, +deg = left.
// Every move is axis-aligned, so the planner only needs headings of 0 / +-90 / 180.
static float plX, plY, plTh;
static bool plCommit;
static const char *scanErr = nullptr;

static float moveTimeS(float D, float v) {
  float a = accel;
  if (D >= v * v / a) return D / v + v / a;
  return 2.0f * sqrtf(D / a);
}
static float normDeg(float a) { while (a > 180.0f) a -= 360.0f; while (a <= -180.0f) a += 360.0f; return a; }
static void plTurn(ScanInfo &inf, float deg) {                       // + = left
  float arc = fabsf(deg * PI / 180.0f) * S.track * 0.5f;
  inf.steps++; inf.timeS += moveTimeS(arc, tVel) + 0.6f;      // + pause and settle per step
  if (plCommit) queueTurn(deg);
}
static void plFwd(ScanInfo &inf, float mm, float v, uint8_t lane) {
  inf.steps++; inf.dist += mm; inf.timeS += moveTimeS(mm, v) + 0.6f;
  if (plCommit) queueStepL(mm, mm, v, lane);
}
static void plTurnTo(ScanInfo &inf, float target) {
  float d = normDeg(target - plTh);
  if (fabsf(d) > 0.5f) plTurn(inf, d);
  plTh = normDeg(target);
}
static void plGoX(ScanInfo &inf, float x, float v, uint8_t lane) {
  float dx = x - plX;
  if (fabsf(dx) < 0.5f) return;
  plTurnTo(inf, dx > 0 ? 0.0f : 180.0f);
  plFwd(inf, fabsf(dx), v, lane);
  plX = x;
}
static void plGoY(ScanInfo &inf, float y, float v) {
  float dy = y - plY;
  if (fabsf(dy) < 0.5f) return;
  plTurnTo(inf, dy > 0 ? 90.0f : -90.0f);
  plFwd(inf, fabsf(dy), v, 0);
  plY = y;
}

// Plans (commit = false) or plans AND queues (commit = true) the serpentine scan. Returns false and sets scanErr on a problem.
bool buildScan(bool commit, ScanInfo &inf) {
  scanErr = nullptr;
  memset(&inf, 0, sizeof(inf));
  scanClamp();
  float v = constrain(Z.v, 20.0f, S.vmax);
  float Wc = Z.w - 2.0f * Z.inset;                 // width / length of the path of the robot centre
  float Lc = Z.h - Z.inset;
  if (Wc < 0.0f || Lc < 20.0f) { scanErr = "Scan area too small after the inset"; return false; }
  int gaps = (Wc < 1.0f) ? 0 : (int)ceilf(Wc / Z.lane - 0.001f);
  if (gaps < 0) gaps = 0;
  float pitch = gaps ? Wc / gaps : 0.0f;
  float yL = Z.off + Wc * 0.5f, yR = Z.off - Wc * 0.5f;
  bool startLeft = fabsf(yL) <= fabsf(yR);        // start at the edge closest to the robot centre line
  float yStart = startLeft ? yL : yR;
  float shift = startLeft ? -pitch : pitch;       // lanes then step towards the far edge
  inf.wc = Wc; inf.lc = Lc; inf.pitch = pitch;

  plX = plY = plTh = 0.0f;
  plCommit = commit;
  qOver = false;
  int qStart = qN;

  plGoY(inf, yStart, v);
  plTurnTo(inf, 0.0f);                            // face along the lanes
  for (int i = 0; i <= gaps; i++) {
    plGoX(inf, (i % 2 == 0) ? Lc : 0.0f, v, (uint8_t)(i + 1));
    inf.lanes++;
    if (i < gaps) plGoY(inf, plY + shift, v);
  }
  if (Z.ret) { plGoX(inf, 0.0f, v, 0); plGoY(inf, 0.0f, v); plTurnTo(inf, 0.0f); }

  if (inf.steps > MAX_STEPS - qStart || qOver) { scanErr = "Too many steps for the queue: raise the lane pitch or shorten the path"; if (commit) qN = qStart; return false; }
  if (commit) scanLanesTotal = (uint8_t)inf.lanes;
  return true;
}
String scanSummary(const ScanInfo &inf) {
  return "Scan " + String(Z.w, 0) + "x" + String(Z.h, 0) + " mm (centre path " + String(inf.wc, 0) + "x" + String(inf.lc, 0) + "): " +
         String(inf.lanes) + " lanes, pitch " + String(inf.pitch, 1) + " mm, " + String(inf.steps) + " steps, " +
         String(inf.dist / 1000.0f, 2) + " m, about " + String(inf.timeS, 0) + " s" + (Z.ret ? ", returns to start" : ", stays at the end");
}

// ============================ CALIBRATIONS ==================================
// Motor deadband / feed-forward (blocking ~1 min, wheels must be lifted!)
float measureSpeed(int pwm, bool left) {
  zeroEnc();
  if (left) setMotors(pwm, 0); else setMotors(0, pwm);
  delay(400);
  long a = left ? getL() : getR();
  unsigned long t0 = millis();
  delay(800);
  long b = left ? getL() : getR();
  float dt = (millis() - t0) / 1000.0f;
  stopMotors(); delay(300);
  return (b - a) * (left ? mmPerCountL() : mmPerCountR()) / dt;
}
void calMotor() {
  backup();
  String out = "Motor cal: ";
  float vmin = 1e9;
  for (int side = 0; side < 2; side++) {
    bool left = (side == 0);
    int dead = 0;
    for (int pwm = 10; pwm <= 120; pwm += 2) {
      zeroEnc();
      if (left) setMotors(pwm, 0); else setMotors(0, pwm);
      delay(300);
      long c = left ? getL() : getR();
      stopMotors(); delay(150);
      if (abs(c) > 5) { dead = pwm; break; }
    }
    float v1 = measureSpeed(120, left);
    float v2 = measureSpeed(220, left);
    float kv = 100.0f / max(v2 - v1, 1.0f);
    float vm = measureSpeed(PWM_MAX, left);
    if (dead == 0) dead = 40;
    if (left) { S.deadL = dead * 0.9f; S.kvL = kv; } else { S.deadR = dead * 0.9f; S.kvR = kv; }
    vmin = min(vmin, vm);
    out += String(left ? "L " : "R ") + "dead=" + String(dead) + " kv=" + String(kv, 3) + " v=" + String(vm, 0) + "; ";
  }
  S.vmax = vmin * 0.7f;
  lastMsg = out + "vmax=" + String(S.vmax, 0);
}

String umbCompute(float side, float xcw, float xccw, bool apply) {
  float alpha = (xcw + xccw) / (-4.0f * side) * (180.0f / PI);
  float beta  = (xcw - xccw) / (-4.0f * side) * (180.0f / PI);
  float Ed = 1.0f;
  if (fabsf(beta) > 0.005f) {
    float R = (side / 2.0f) / sinf((beta * PI / 180.0f) / 2.0f);
    Ed = (R + S.track / 2.0f) / (R - S.track / 2.0f);
  }
  float Eb = 90.0f / (90.0f - alpha);
  String m = "alpha=" + String(alpha, 3) + " beta=" + String(beta, 3) + " Ed=" + String(Ed, 5) + " Eb=" + String(Eb, 5);
  if (!isfinite(Ed) || !isfinite(Eb) || Ed < 0.9f || Ed > 1.1f || Eb < 0.9f || Eb > 1.1f)
    return m + " -> OUT OF RANGE, recheck your measurements. Nothing applied.";
  if (!apply) return m + " (preview, not applied)";
  backup();
  S.corrL *= 2.0f / (Ed + 1.0f);
  S.corrR *= 2.0f / ((1.0f / Ed) + 1.0f);
  S.track *= Eb;
  return m + " -> APPLIED. corrL=" + String(S.corrL, 5) + " corrR=" + String(S.corrR, 5) + " track=" + String(S.track, 3);
}

// ============================ WEB UI ========================================
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta name=viewport content="width=device-width,initial-scale=1"><title>JGB37</title>
<style>body{font-family:sans-serif;margin:8px;background:#111;color:#eee}h3{margin:16px 0 4px;color:#7cf}
input{width:28%;padding:8px;font-size:16px;margin:2px;background:#222;color:#fff;border:1px solid #444;border-radius:5px}
input.w{width:92%}button{padding:10px 12px;font-size:16px;margin:3px;border:0;border-radius:6px;background:#2a6;color:#fff}
button.o{background:#555}.r{background:#d22;width:100%;font-size:24px;padding:18px;margin:0}
pre{background:#000;padding:6px;font-size:12px;white-space:pre-wrap}#m{color:#fd5;min-height:2.4em;margin:6px 0}small{color:#999}</style></head><body>
<button class=r onclick="fetch('/stop').then(st)">STOP</button>
<div id=m>...</div><pre id=s></pre>
<h3>Manual</h3>
<input id=mm value=200 type=number>mm <button onclick="go('/mv?mm='+v('mm'))">Move</button><br>
<input id=dg value=90 type=number>deg <button onclick="go('/tn?deg='+v('dg'))">Turn (+left)</button><br>
<button class=o onclick="go('/z')">Zero counters + odom</button>
<h3>1. Counts per rev</h3><small>Zero, then turn BOTH wheels forward by hand exactly N revolutions (use a mark on the wheel).</small><br>
<button class=o onclick="go('/z')">Zero</button><input id=rv value=10 type=number>revs <button onclick="go('/cpr?revs='+v('rv'))">Compute</button>
<h3>2. Distance + drift</h3><small>Robot on floor, mark centre of axle at start.</small><br>
<input id=dc value=1000 type=number>mm <button onclick="go('/mv?mm='+v('dc'))">Drive</button><br>
<input id=da placeholder="actual mm" type=number step=any><input id=dh placeholder="heading deg (left +)" type=number step=any>
<button onclick="go('/dfix?cmd='+v('dc')+'&act='+v('da')+'&dth='+v('dh'))">Apply</button><br>
<small>heading ~ atan((right wheel fwd pos - left wheel fwd pos)/150) ; or 2*sideDrift/1000 rad</small>
<h3>3. Rotation (track width)</h3>
<input id=tt value=5 type=number>turns <button onclick="go('/tn?deg='+(360*v('tt')))">Spin left</button><br>
<input id=tr placeholder="extra deg past mark (+over/-under)" class=w type=number step=any>
<button onclick="go('/tfix?cmd='+(360*v('tt'))+'&res='+v('tr'))">Apply</button>
<h3>4. Square test (UMBmark)</h3>
<input id=ss value=1000 type=number>mm side <button onclick="go('/sq?side='+v('ss')+'&dir=cw')">CW square</button><button onclick="go('/sq?side='+v('ss')+'&dir=ccw')">CCW square</button><br>
<small>Final position error vs start: x fwd, y left (mm)</small><br>
<input id=x1 placeholder="CW x" type=number step=any><input id=y1 placeholder="CW y" type=number step=any><br>
<input id=x2 placeholder="CCW x" type=number step=any><input id=y2 placeholder="CCW y" type=number step=any><br>
<button class=o onclick="go('/ufix?side='+v('ss')+'&xcw='+v('x1')+'&xccw='+v('x2')+'&apply=0')">Preview</button>
<button onclick="go('/ufix?side='+v('ss')+'&xcw='+v('x1')+'&xccw='+v('x2')+'&apply=1')">Apply</button>
<h3>Path runner</h3><small>F=forward mm, B=back mm, T=turn deg (+left), Z1=scan the area below. Up to 128 steps.</small><br>
<input id=pp class=w value="F85,T90,F380,Z1"><br><button onclick="go('/p?s='+v('pp'))">Run path</button>
<h3>Scan area (Z)</h3><small>Z1 in the path runner scans this area with the robot CENTRE: forward = depth, left = width, relative to where the robot is when Z1 starts.
Lanes run forward/back. Inset keeps the body off the walls (centre path = area minus inset). Pitch should be about the width of what the sensor sees.</small><br>
<input id=zw type=number step=any>W mm <input id=zh type=number step=any>H mm <input id=zo type=number step=any>offset mm (+left)<br>
<input id=zl type=number step=any>lane pitch <input id=zi type=number step=any>inset <input id=zv type=number step=any>speed mm/s<br>
<label><input type=checkbox id=zr style="width:auto"> return to start pose</label><br>
<button onclick="go('/zcfg?'+zq())">Apply</button><button class=o onclick="go('/zplan?'+zq())">Preview plan</button><button onclick="go('/zrun?'+zq())">Run scan now</button><br>
<button class=o onclick="$('pp').value=($('pp').value?$('pp').value+',':'')+'Z1'">+ Z1 in path</button><button class=o onclick="$('pp').value='F85,T90,F380,Z1'">Load F85,T90,F380,Z1</button>
<h3>Settings</h3><small>names: scanW scanH scanOff scanLane scanInset scanV scanRet wheel track kp ki vmax v a tv deadL deadR kvL kvR cprL cprR corrL corrR</small><br>
<input id=k placeholder=name><input id=kv placeholder=value type=number step=any><button onclick="go('/set?k='+v('k')+'&v='+v('kv'))">Set</button><br>
<button onclick="go('/save')">SAVE</button><button class=o onclick="go('/undo')">Undo last change</button>
<button class=o onclick="go('/mot')">Motor cal (wheels LIFTED)</button><button class=o onclick="if(confirm('Reset defaults?'))go('/reset')">Reset defaults</button>
<script>
const $=i=>document.getElementById(i);const v=i=>$(i).value;let bz=0,zin=0;
function zq(){return 'w='+v('zw')+'&h='+v('zh')+'&off='+v('zo')+'&lane='+v('zl')+'&inset='+v('zi')+'&v='+v('zv')+'&ret='+($('zr').checked?1:0);}
async function st(){if(bz)return;bz=1;try{const j=await(await fetch('/s')).json();
if(!zin&&j.zw!==undefined){$('zw').value=j.zw;$('zh').value=j.zh;$('zo').value=j.zo;$('zl').value=j.zl;$('zi').value=j.zi;$('zv').value=j.zv;$('zr').checked=(j.zr==1);zin=1;}
$('m').innerText=j.msg+(j.busy?'   [MOVING step '+j.qi+'/'+j.qn+(j.z?'  '+j.z:'')+']':'');
$('s').innerText='odom x='+j.x+' y='+j.y+' th='+j.th+'deg\ncounts L='+j.cL+' R='+j.cR+'\ncpr '+j.cprL+' / '+j.cprR+'\ncorr '+j.corrL+' / '+j.corrR+'\nwheel '+j.wheel+'  track '+j.track+'\ndead '+j.deadL+'/'+j.deadR+'  kv '+j.kvL+'/'+j.kvR+'  vmax '+j.vmax+'\nkp '+j.kp+' ki '+j.ki+'  v '+j.v+' a '+j.a+' tv '+j.tv;
}catch(e){$('m').innerText='No connection - stay on JGB37-Robot Wi-Fi';}bz=0;}
async function go(u){while(bz)await new Promise(r=>setTimeout(r,60));bz=1;try{await fetch(u);}catch(e){}bz=0;st();}
setInterval(st,800);st();
</script></body></html>)HTML";

WiFiServer server(80);

String param(const String &q, const char *k) {
  String key = String(k) + "=";
  int i = q.indexOf("?" + key);
  if (i < 0) i = q.indexOf("&" + key);
  if (i < 0) return "";
  i += key.length() + 1;
  int j = q.indexOf('&', i);
  return j < 0 ? q.substring(i) : q.substring(i, j);
}
float pf(const String &q, const char *k, float def) {
  String s = param(q, k);
  return s.length() ? s.toFloat() : def;
}
bool has(const String &q, const char *k) { return param(q, k).length() > 0; }

void sendJson(WiFiClient &c, const String &b) {
  c.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
  c.print(b);
  c.stop();
}
void sendPage(WiFiClient &c) {
  c.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"));
  size_t n = strlen(PAGE);
  for (size_t i = 0; i < n; i += 256) {
    size_t k = (n - i < 256) ? (n - i) : 256;
    c.write((const uint8_t *)PAGE + i, k);
  }
  c.stop();
}
void sendOk(WiFiClient &c) { sendJson(c, "{\"ok\":1}"); }

void sendStatus(WiFiClient &c) {
  odomUpdate();
  String j = "{\"busy\":" + String((mAct || qWait) ? 1 : 0);
  j += ",\"msg\":\"" + lastMsg + "\"";
  j += ",\"x\":" + String(ox, 1) + ",\"y\":" + String(oy, 1) + ",\"th\":" + String(oth * 180.0f / PI, 2);
  j += ",\"cL\":" + String(getL()) + ",\"cR\":" + String(getR());
  j += ",\"cprL\":" + String(S.cprL, 3) + ",\"cprR\":" + String(S.cprR, 3);
  j += ",\"corrL\":" + String(S.corrL, 5) + ",\"corrR\":" + String(S.corrR, 5);
  j += ",\"wheel\":" + String(S.wheelDiam, 3) + ",\"track\":" + String(S.track, 3);
  j += ",\"deadL\":" + String(S.deadL, 1) + ",\"deadR\":" + String(S.deadR, 1);
  j += ",\"kvL\":" + String(S.kvL, 4) + ",\"kvR\":" + String(S.kvR, 4);
  j += ",\"vmax\":" + String(S.vmax, 1) + ",\"kp\":" + String(S.kp, 2) + ",\"ki\":" + String(S.ki, 2);
  j += ",\"v\":" + String(vCruise, 0) + ",\"a\":" + String(accel, 0) + ",\"tv\":" + String(tVel, 0);
  j += ",\"zw\":" + String(Z.w, 1) + ",\"zh\":" + String(Z.h, 1) + ",\"zo\":" + String(Z.off, 1) + ",\"zl\":" + String(Z.lane, 1);
  j += ",\"zi\":" + String(Z.inset, 1) + ",\"zv\":" + String(Z.v, 0) + ",\"zr\":" + String((int)Z.ret);
  j += ",\"qi\":" + String(qI) + ",\"qn\":" + String(qN);
  j += ",\"z\":\"" + String(curLane ? ("lane " + String(curLane) + "/" + String(scanLanesTotal)) : String("")) + "\"}";
  sendJson(c, j);
}

void handle(WiFiClient &c, String path) {
  path.replace("%2C", ","); path.replace("%2c", ","); path.replace("%20", ""); path.replace(" ", "");
  int qi = path.indexOf('?');
  String route = qi < 0 ? path : path.substring(0, qi);

  if (route == "/" || route == "/index.html") { sendPage(c); return; }
  if (route == "/s") { sendStatus(c); return; }
  if (route == "/stop") { abortAll("STOPPED by user"); sendOk(c); return; }
  if (route == "/favicon.ico") { c.print(F("HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n")); c.stop(); return; }

  if (mAct || qWait) { lastMsg = "Busy - wait or press STOP"; sendOk(c); return; }

  if (route == "/mv") {
    float mm = constrain(pf(path, "mm", 0), -5000.0f, 5000.0f);
    qN = qI = 0; queueStraight(mm); beginQueue();
    lastMsg = "Moving " + String(mm, 1) + " mm";
  } else if (route == "/tn") {
    float dg = constrain(pf(path, "deg", 0), -7200.0f, 7200.0f);
    qN = qI = 0; queueTurn(dg); beginQueue();
    lastMsg = "Turning " + String(dg, 1) + " deg";
  } else if (route == "/z") {
    zeroEnc(); odomReset(); lastMsg = "Counters + odometry zeroed";
  } else if (route == "/cpr") {
    float revs = pf(path, "revs", 0);
    long l = getL(), r = getR();
    if (revs <= 0) lastMsg = "Enter number of revolutions";
    else if (l <= 0 || r <= 0) lastMsg = "Count <= 0 (L=" + String(l) + " R=" + String(r) + "): turn wheels FORWARD or flip ENC_x_SIGN. Not changed.";
    else {
      backup(); S.cprL = l / revs; S.cprR = r / revs;
      lastMsg = "cprL=" + String(S.cprL, 3) + " cprR=" + String(S.cprR, 3) + " (not saved yet)";
    }
  } else if (route == "/dfix") {
    float cmd = pf(path, "cmd", 0), act = pf(path, "act", 0), dth = pf(path, "dth", 0);
    if (cmd < 1 || act < 1) lastMsg = "Enter the actual measured distance";
    else {
      backup();
      float kAvg = act / cmd;
      float kDiff = (dth * PI / 180.0f) * S.track / cmd;
      S.corrL *= kAvg - kDiff * 0.5f;
      S.corrR *= kAvg + kDiff * 0.5f;
      lastMsg = "corrL=" + String(S.corrL, 5) + " corrR=" + String(S.corrR, 5) + " - repeat the drive to verify";
    }
  } else if (route == "/tfix") {
    float cmd = pf(path, "cmd", 0), res = pf(path, "res", 0);
    float actual = cmd + res;
    if (cmd < 1 || actual < 1) lastMsg = "Bad values";
    else { backup(); S.track = S.track * cmd / actual; lastMsg = "track=" + String(S.track, 3) + " mm - repeat the spin to verify"; }
  } else if (route == "/sq") {
    float side = constrain(pf(path, "side", 1000), 100.0f, 3000.0f);
    bool ccw = param(path, "dir") == "ccw";
    qN = qI = 0;
    for (int i = 0; i < 4; i++) { queueStraight(side); queueTurn(ccw ? 90.0f : -90.0f); }
    beginQueue(); odomReset();
    lastMsg = String(ccw ? "CCW" : "CW") + " square running";
  } else if (route == "/ufix") {
    if (!has(path, "xcw") || !has(path, "xccw")) lastMsg = "Enter CW x and CCW x errors";
    else lastMsg = umbCompute(pf(path, "side", 1000), pf(path, "xcw", 0), pf(path, "xccw", 0), pf(path, "apply", 0) > 0.5f);
  } else if (route == "/p") {
    String s = param(path, "s"); s.toUpperCase();
    qN = qI = 0; qOver = false; curLane = 0; scanLanesTotal = 0;
    int i = 0; bool bad = false; const char *badWhy = nullptr;
    while (i < (int)s.length() && !bad) {
      int j = s.indexOf(',', i); if (j < 0) j = s.length();
      String tok = s.substring(i, j);
      if (tok.length() >= 2) {
        char ch = tok.charAt(0); float val = tok.substring(1).toFloat();
        if (ch == 'F') queueStraight(constrain(val, -5000.0f, 5000.0f));
        else if (ch == 'B') queueStraight(-constrain(val, -5000.0f, 5000.0f));
        else if (ch == 'T') queueTurn(constrain(val, -7200.0f, 7200.0f));
        else if (ch == 'Z') {                                  // Z1 = scan the area
          ScanInfo inf;
          if ((int)val != 1) { bad = true; badWhy = "Z takes the value 1 (Z1 = scan the area)"; }
          else if (!buildScan(true, inf)) { bad = true; badWhy = scanErr ? scanErr : "scan could not be planned"; }
        }
        else bad = true;
      } else if (tok.length()) bad = true;
      i = j + 1;
    }
    if (qOver) { bad = true; badWhy = "Path too long (max 128 steps)"; }
    if (bad || qN == 0) { qN = qI = 0; scanLanesTotal = 0; lastMsg = badWhy ? String("Bad path: ") + badWhy : String("Bad path. Example: F85,T90,F380,Z1"); }
    else { beginQueue(); odomReset(); lastMsg = "Running path: " + String(qN) + " steps" + (scanLanesTotal ? " (includes the scan)" : ""); }
  } else if (route == "/zcfg" || route == "/zplan" || route == "/zrun") {
    backup();
    Z.w = pf(path, "w", Z.w); Z.h = pf(path, "h", Z.h); Z.off = pf(path, "off", Z.off);
    Z.lane = pf(path, "lane", Z.lane); Z.inset = pf(path, "inset", Z.inset); Z.v = pf(path, "v", Z.v);
    if (param(path, "ret").length()) Z.ret = pf(path, "ret", Z.ret) > 0.5f ? 1 : 0;
    scanClamp();
    ScanInfo inf;
    if (route == "/zcfg") {
      bool ok = buildScan(false, inf);
      lastMsg = ok ? "Scan settings applied (SAVE to keep). " + scanSummary(inf) : String("Scan settings applied but the plan is invalid: ") + scanErr;
    } else if (route == "/zplan") {
      lastMsg = buildScan(false, inf) ? "Plan only, robot not moving. " + scanSummary(inf) : String("Scan plan invalid: ") + scanErr;
    } else {
      qN = qI = 0; curLane = 0; scanLanesTotal = 0;
      if (!buildScan(true, inf)) { qN = qI = 0; lastMsg = String("Scan not started: ") + scanErr; }
      else { beginQueue(); odomReset(); lastMsg = "Scan running. " + scanSummary(inf); }
    }
  } else if (route == "/set") {
    String k = param(path, "k"); float val = pf(path, "v", NAN);
    if (!isfinite(val)) lastMsg = "Enter a value";
    else {
      backup(); bool okk = true;
      if (k == "wheel") S.wheelDiam = val; else if (k == "track") S.track = val;
      else if (k == "kp") S.kp = val; else if (k == "ki") S.ki = val;
      else if (k == "vmax") S.vmax = val; else if (k == "v") vCruise = val;
      else if (k == "a") accel = constrain(val, 20.0f, 2000.0f); else if (k == "tv") tVel = val;
      else if (k == "deadL") S.deadL = val; else if (k == "deadR") S.deadR = val;
      else if (k == "kvL") S.kvL = val; else if (k == "kvR") S.kvR = val;
      else if (k == "cprL") S.cprL = val; else if (k == "cprR") S.cprR = val;
      else if (k == "corrL") S.corrL = val; else if (k == "corrR") S.corrR = val;
      else if (k == "scanW") Z.w = val; else if (k == "scanH") Z.h = val; else if (k == "scanOff") Z.off = val;
      else if (k == "scanLane") Z.lane = val; else if (k == "scanInset") Z.inset = val; else if (k == "scanV") Z.v = val;
      else if (k == "scanRet") Z.ret = val > 0.5f ? 1 : 0;
      else okk = false;
      if (okk) scanClamp();
      if (okk) lastMsg = k + " = " + String(val, 5);
      else lastMsg = String("Unknown setting name");
    }
  } else if (route == "/save") {
    saveSettings(); scanSave(); lastMsg = "Saved to EEPROM (calibration + scan settings)";
  } else if (route == "/undo") {
    if (haveBackup) { S = Sprev; Z = Zprev; haveBackup = false; lastMsg = "Reverted last change (not saved yet)"; }
    else lastMsg = "Nothing to undo";
  } else if (route == "/reset") {
    backup(); defaults(); scanDefaults(); lastMsg = "Defaults restored (not saved)";
  } else if (route == "/mot") {
    sendOk(c);                       // reply first, then run (blocking ~1 min)
    lastMsg = "Motor cal running...";
    calMotor();
    return;
  }
  sendOk(c);
}

void handleClient() {
  WiFiClient c = server.available();
  if (!c) return;
  String req = "";
  unsigned long t0 = millis();
  while (c.connected() && millis() - t0 < 400) {
    if (c.available()) { char ch = c.read(); if (ch == '\n') break; if (ch != '\r' && req.length() < 300) req += ch; }
  }
  unsigned long t1 = millis();
  while (millis() - t1 < 15) while (c.available()) { c.read(); t1 = millis(); }
  int a = req.indexOf(' '), b = req.indexOf(' ', a + 1);
  if (a < 0 || b < 0) { c.stop(); return; }
  handle(c, req.substring(a + 1, b));
}

// ============================ MAIN ==========================================
void setup() {
  Serial.begin(115200);
  unsigned long ts = millis();
  while (!Serial && millis() - ts < 2000) {}
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(M_L_PWM, OUTPUT); pinMode(M_L_DIR, OUTPUT);
  pinMode(M_R_PWM, OUTPUT); pinMode(M_R_DIR, OUTPUT);
  stopMotors();
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);
  loadSettings();
  scanLoad();
  odomReset();

  WiFi.config(IPAddress(192, 168, 4, 1));
  int st = WiFi.beginAP(AP_SSID, AP_PASS);
  if (st != WL_AP_LISTENING) {
    Serial.println(F("Access point failed - update the UNO R4 WiFi module firmware."));
    while (true) delay(1000);
  }
  server.begin();
  Serial.println(F("Join Wi-Fi JGB37-Robot, open http://192.168.4.1"));
}

void loop() {
  odomUpdate();
  motionTick();
  queueTick();
  static unsigned long lastNet = 0;
  if (!mAct || millis() - lastNet >= 60) {
    lastNet = millis();
    handleClient();
  }
}
