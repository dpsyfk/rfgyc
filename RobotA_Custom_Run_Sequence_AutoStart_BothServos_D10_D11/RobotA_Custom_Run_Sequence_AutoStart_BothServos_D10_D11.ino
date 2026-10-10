// ROBOT A AUTONOMOUS MERGE
// Motion/calibration: Robot_A_v2.9.4_forward_only_corrected(1).ino
// Custom route starts automatically after a calibrated 500 ms startup delay.
// Analog thresholds/averaging: robo_A_ir_sensor_test_ok.ino
// Automatic boot movement disabled. Serial/WiFi STOP aborts the route.
// NOTE: detection creates a two-second stationary pause, no arm is implemented.
// Sensor assertions require physical validation; no encoder direction verification.
// v2.9.1: Declare custom parameter types before Arduino's auto-generated
// function prototypes. This prevents the WheelPI compile error in Arduino IDE.
struct WheelPI {
  float integral;
  float measured;
  int pwm;
};
// Explicit prototype prevents Arduino preprocessor from generating an invalid
// declaration ahead of the WheelPI type.
int wheelOutput(WheelPI &c, float demand, float measurement, float dt, int maximum);
void runSequence();
// Declare before Arduino IDE auto-generates function prototypes.
struct DiscReading {
  int left, center, right;
  bool L, C, R;
};
DiscReading readDisc();
void executeMotion(char kind, float amount);

#include <EEPROM.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <WiFiS3.h>
#include <Servo.h>

// =====================================================
// WIFI - TESTING ONLY
// =====================================================
const char* WIFI_SSID = ""; // Optional: set own SSID
const char* WIFI_PASS = ""; // Optional: set own password

// IP address assigned automatically by router (DHCP).

const uint16_t SERVER_PORT = 23;

WiFiServer wifiServer(SERVER_PORT);
WiFiClient wifiClient;

String wifiCommandBuffer = "";

// Gate servos. Signals: right gate = D10, left gate = D11 (D9 is RIGHT_PWM, so not D9).
// Servos need their own 5-6 V supply with GND common to the Arduino.
// Each gate: neutral pulse, open pulse for GATE_PULSE_MS, then back to neutral.
// The left servo is mirror-mounted, so its open pulse is on the opposite side of 1500.
// If a gate moves the wrong way, swap that gate's OPEN_US (e.g. 1700 <-> 1300).
Servo rightGateServo;
Servo leftGateServo;
const byte RIGHT_GATE_SERVO_PIN = 10;
const byte LEFT_GATE_SERVO_PIN  = 11;
const int RIGHT_GATE_NEUTRAL_US = 1500;
const int RIGHT_GATE_OPEN_US    = 1700;
const int LEFT_GATE_NEUTRAL_US  = 1500;
const int LEFT_GATE_OPEN_US     = 1300;
const unsigned long GATE_PULSE_MS = 250;
bool rightGateServoAttached = false;
bool leftGateServoAttached  = false;

// =====================================================
// MOTOR PINS
// =====================================================
const int LEFT_PWM  = 6;
const int LEFT_DIR  = 7;

const int RIGHT_DIR = 8;
const int RIGHT_PWM = 9;

// =====================================================
// ENCODER PINS
// =====================================================
const int LEFT_ENC_A  = 2;
const int LEFT_ENC_B  = 4;

const int RIGHT_ENC_A = 3;
const int RIGHT_ENC_B = 5;

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

// =====================================================
// MOTOR DIRECTIONS
// =====================================================
const bool LEFT_FORWARD_DIR  = LOW;
const bool RIGHT_FORWARD_DIR = HIGH;

// =====================================================
// CALIBRATION v2.4: fourth physical test on v2.3.
// Confirmed: B100->97, B300->295, F300->300.
// F200->145 conflicts sharply with F300->300; treat as an anomaly,
// NOT an established wheel-scale correction. A full 200/145 increase
// might produce dangerous overshoot if the result was due to wheel slip,
// an obstruction or an encoder fault. Only an 8% diagnostic correction
// is allowed at F200 until repeated measurements confirm it.
// The reported "R100->100" is ambiguous; no assumption made about it.
// Heading drift cannot be corrected reliably without heading feedback.
// v2.8 persistent, user-confirmed motion calibration. No automatic angle sensing.
// A correction is only recorded after a physical measurement via CAL.
struct CalibrationPoint { float command; float multiplier; };
struct PersistentCalibration {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  CalibrationPoint points[4][8];  // F, B, L, R
  uint32_t checksum;
};
PersistentCalibration calibration;
const uint32_t CAL_MAGIC = 0x52423238UL;
const uint16_t CAL_VERSION = 1; // Exact v2.8 EEPROM layout retained
int kindIndex(char c) {
  if(c=='F') return 0;
  if(c=='B') return 1;
  if(c=='L') return 2;
  if(c=='R') return 3;
  return -1;
}
uint32_t checksumCalibration(const PersistentCalibration &c) {
  const uint8_t *p = reinterpret_cast<const uint8_t*>(&c);
  uint32_t hash = 2166136261UL;
  for(size_t i=0; i<offsetof(PersistentCalibration,checksum); ++i) {
    hash = (hash ^ p[i]) * 16777619UL;
  }
  return hash;
}
void resetCalibrationRAM() {
  memset(&calibration, 0, sizeof(calibration));
  calibration.magic=CAL_MAGIC;
  calibration.version=CAL_VERSION;
}
bool saveCalibration() {
  calibration.checksum=checksumCalibration(calibration);
  EEPROM.put(0, calibration);
  PersistentCalibration verify;
  EEPROM.get(0, verify);
  bool ok = verify.magic == CAL_MAGIC && verify.version == CAL_VERSION &&
            verify.checksum == checksumCalibration(verify);
  return ok;
}
bool calibrationValid = false;
void loadCalibration() {
  EEPROM.get(0, calibration);
  if(calibration.magic!=CAL_MAGIC || calibration.version!=CAL_VERSION ||
      calibration.checksum!=checksumCalibration(calibration)) {
    resetCalibrationRAM();
    calibrationValid = false;
    Serial.println("EEPROM invalid/unsupported: using RAM defaults; bytes NOT overwritten");
  } else {
    bool valid = true;
    for (int k=0;k<4;k++) for (int i=0;i<8;i++) {
      const CalibrationPoint &p=calibration.points[k][i];
      if (!isfinite(p.command) || !isfinite(p.multiplier) ||
          p.command < 0 || p.command > 720 ||
          (p.command > 0 && (p.multiplier < 0.5f || p.multiplier > 1.5f))) valid=false;
    }
    if (!valid) { resetCalibrationRAM(); Serial.println("EEPROM ranges invalid: RAM defaults; storage untouched"); }
    else { calibrationValid=true; Serial.println("EEPROM v2.8 verified and loaded"); }
  }
}
// Linear interpolation in commanded distance/angle; outside saved points use
// closest anchor. Unmeasured actions still have no accuracy guarantee.
float savedCorrection(int kind, float requested) {
  int lower=-1, upper=-1;
  for(int i=0;i<8;i++) {
    float d=calibration.points[kind][i].command;
    if(d<=0) continue;
    if(d<=requested && (lower<0 || d>calibration.points[kind][lower].command)) lower=i;
    if(d>=requested && (upper<0 || d<calibration.points[kind][upper].command)) upper=i;
  }
  if(lower<0 && upper<0) return 1.0f;
  if(lower<0) return calibration.points[kind][upper].multiplier;
  if(upper<0) return calibration.points[kind][lower].multiplier;
  if(lower==upper) return calibration.points[kind][lower].multiplier;
  const CalibrationPoint &a=calibration.points[kind][lower], &b=calibration.points[kind][upper];
  float t=(requested-a.command)/(b.command-a.command);
  return a.multiplier+t*(b.multiplier-a.multiplier);
}
// A new CAL correction multiplies the current calculated target by desired/actual.
// At repeated CAL for the same command, update the existing persistent factor.
bool recordCalibration(char motion, float requested, float actual) {
  int kind=kindIndex(motion);
  if(kind<0 || !isfinite(requested) || !isfinite(actual) ||
     requested < 1.0f || requested > 720.0f || actual <= 0.0f) return false;
  float ratio=requested/actual;
  if(ratio<0.7f || ratio>1.3f) return false; // suspicious readings need diagnosis
  int slot=-1;
  for(int i=0;i<8;i++) if(fabs(calibration.points[kind][i].command-requested)<0.01f) {slot=i;break;}
  if(slot<0) for(int i=0;i<8;i++) if(calibration.points[kind][i].command==0.0f) {slot=i;break;}
  if(slot<0) return false;
  // The requested result must be from the latest complete run, not a repeat report.
  float prior=savedCorrection(kind,requested);
  float adjusted=prior*ratio;
  if(adjusted<0.5f || adjusted>1.5f) return false;
  if (!calibrationValid) return false; // never overwrite unknown EEPROM format
  calibration.points[kind][slot].command=requested;
  calibration.points[kind][slot].multiplier=adjusted;
  return saveCalibration();
}

const float COUNTS_PER_MM = 6.54f;
const float LEFT_TURN_COUNTS_PER_DEGREE  = 10.4556f * (90.0f / 88.0f);
const float RIGHT_TURN_COUNTS_PER_DEGREE = 10.4556f * (90.0f / 92.0f);

float previousDistanceTargetMM(float requestMM, bool forward) {
  if (requestMM <= 0.0f) return 0.0f;
  if (forward) {
    const float d[] = {37.5f, 100.0f, 200.0f};
    const float ratio[] = {37.5f / 40.0f, 1.0f, 200.0f / 199.0f};
    if (requestMM <= d[0]) return requestMM * ratio[0];
    if (requestMM >= d[2]) return requestMM * ratio[2];
    int i = (requestMM <= d[1]) ? 0 : 1;
    float t = (requestMM - d[i]) / (d[i+1] - d[i]);
    return requestMM * (ratio[i] + t * (ratio[i+1] - ratio[i]));
  }
  const float loRatio = 100.0f / 105.0f;
  const float hiRatio = 200.0f / 192.0f;
  if (requestMM <= 100.0f) return requestMM * loRatio;
  if (requestMM >= 200.0f) return requestMM * hiRatio;
  float t = (requestMM - 100.0f) / 100.0f;
  return requestMM * (loRatio + t * (hiRatio - loRatio));
}

float interpolateCorrection(float requested, float a, float correctionA,
                            float b, float correctionB) {
  if (requested <= a) return correctionA;
  if (requested >= b) return correctionB;
  float t = (requested - a) / (b - a);
  return correctionA + t * (correctionB - correctionA);
}

// Reproduce exact v2.2 targets before applying v2.3 measurements.
float v22DistanceTargetMM(float requestMM, bool forward) {
  float correction = forward
    ? interpolateCorrection(requestMM, 100.0f, 1.0f, 300.0f, 300.0f/295.0f)
    : interpolateCorrection(requestMM, 100.0f, 100.0f/95.0f,
                            300.0f, 300.0f/305.0f);
  return previousDistanceTargetMM(requestMM, forward) * correction;
}

float correctedDistanceTargetMM(float requestMM, bool forward) {
  if (requestMM <= 0.0f) return 0.0f;
  float correction;
  if (forward) {
    // At 300 mm the v2.2 target was exact: leave unchanged.
    // At 200 mm cap correction because 145 mm is an unexplained outlier.
    if (requestMM <= 200.0f)
      correction = interpolateCorrection(requestMM, 100.0f, 1.0f, 200.0f, 1.08f);
    else
      correction = interpolateCorrection(requestMM, 200.0f, 1.08f, 300.0f, 1.0f);
  } else {
    correction = interpolateCorrection(requestMM, 100.0f, 100.0f/97.0f,
                                       300.0f, 300.0f/295.0f);
  }
  // v2.9.2 NEW physical observations made using the v2.9.1 motion loop:
  // F100 -> 95mm (target 654), F200 -> 195mm (target 1432).
  // Apply the incremental measured correction to the *existing* v2.8 base;
  // do not change COUNTS_PER_MM and do not touch B, L or R.
  // At <=10mm keep the old target, and at >=300mm return to the old target;
  // unmeasured intermediate values are interpolated and need verification.
  float newForwardFactor = 1.0f;
  if (forward) {
    if (requestMM <= 10.0f) newForwardFactor = 1.0f;
    else if (requestMM <= 100.0f)
      newForwardFactor = interpolateCorrection(requestMM, 10.0f, 1.0f, 100.0f, 100.0f/95.0f);
    else if (requestMM <= 200.0f)
      newForwardFactor = interpolateCorrection(requestMM, 100.0f, 100.0f/95.0f, 200.0f, 200.0f/195.0f);
    else
      newForwardFactor = interpolateCorrection(requestMM, 200.0f, 200.0f/195.0f, 300.0f, 1.0f);
  }
  // v2.9.3: latest measured runs ON v2.9.2:
  // F100 physically ~110 mm -> 100/110 correction on existing F100 target (688).
  // B200 physically ~210 mm -> 200/210 correction on existing B200 target (1420).
  // These are NOT corrections to the base scale and must not be reapplied with
  // CAL for the same observations. Nearby points taper to 1.0 (provisional).
  float newestFactor = 1.0f;
  if (forward) {
    if (requestMM <= 100.0f)
      newestFactor = 100.0f / 110.0f;
    else if (requestMM < 200.0f)
      newestFactor = interpolateCorrection(requestMM, 100.0f, 100.0f/110.0f, 200.0f, 1.0f);
  } else {
    if (requestMM <= 100.0f)
      newestFactor = 1.0f;
    else if (requestMM <= 200.0f)
      newestFactor = interpolateCorrection(requestMM, 100.0f, 1.0f, 200.0f, 200.0f/210.0f);
    else if (requestMM < 300.0f)
      newestFactor = interpolateCorrection(requestMM, 200.0f, 200.0f/210.0f, 300.0f, 1.0f);
  }
  float calibratedMM = v22DistanceTargetMM(requestMM, forward) * correction * newForwardFactor * newestFactor * savedCorrection(forward ? 0 : 1, requestMM);
  // v2.9.4: NEW physical report on v2.9.3: forward motions travel ~10 mm too far.
  // Subtract an ABSOLUTE 10 mm-equivalent target only in F, not another
  // percentage multiplier. This preserves existing B/L/R target calculations.
  // Smoothly ramp the correction from 0 at 0mm to 10mm at 100mm so
  // small movements do not acquire a negative/zero target.
  // Preliminary: verify actual F100/F200/F300 with a ruler before CAL.
  if (forward) {
    const float forwardOvershootMM = 10.0f;
    float reductionMM = forwardOvershootMM * constrain(requestMM / 100.0f, 0.0f, 1.0f);
    calibratedMM -= reductionMM;
    if (calibratedMM < 0.5f) calibratedMM = 0.5f;
  }
  return calibratedMM;
}

// v2.5: additional corrections from physical v2.4 turn testing.
// Keep the previously tuned 90- and 180-degree anchors.
// On v2.3 L270 commanded 270 degrees but physically turned ~250 degrees.
// At 270, v2.3 used 0.90. Apply 270/250 to that existing target.
float correctedLeftTurnCounts(float degrees) {
  if (degrees <= 0.0f) return 0.0f;
  const float c90 = 90.0f / 93.0f;
  const float c180 = 180.0f / 200.0f;
  const float c270 = c180 * (270.0f / 250.0f);
  float correction = degrees <= 180.0f
      ? interpolateCorrection(degrees, 90.0f, c90, 180.0f, c180)
      : interpolateCorrection(degrees, 180.0f, c180, 270.0f, c270);
  // v2.4 L270 physically reached 250 degrees: add 270/250 at 270.
  // Blend from zero extra correction at 180 degrees.
  const float extra = interpolateCorrection(degrees, 180.0f, 1.0f,
                                             270.0f, 270.0f / 250.0f);
  // Physical v2.7 test: L270 reached 315 degrees. Smoothly blend correction
  // from unchanged L180 (physically correct) to 270/315 at L270.
  float v28 = interpolateCorrection(degrees, 180.0f, 1.0f, 270.0f, 270.0f/315.0f);
  return degrees * LEFT_TURN_COUNTS_PER_DEGREE * correction * extra * v28 * savedCorrection(2, degrees);
}

// v2.6 additional calibration, relative to v2.5 firmware only.
// v2.5 tests: R90 -> 100 degrees; R270 -> 250 degrees.
// No new R180 test was supplied: retain its v2.5 calibration exactly.
// Piecewise scale factor linearly interpolated at requested angle.
// These are empirical estimates, not guaranteed physical outcomes.
float correctedRightTurnCounts(float degrees) {
  if (degrees <= 0.0f) return 0.0f;
  const float previousCorrection = interpolateCorrection(
      degrees, 90.0f, 1.0f, 180.0f, 180.0f / 190.0f);
  const float previousExtra = interpolateCorrection(
      degrees, 90.0f, 1.0f, 180.0f, 180.0f / 190.0f);
  const float newCorrection = (degrees <= 180.0f)
      ? interpolateCorrection(degrees, 90.0f, 90.0f / 100.0f,
                              180.0f, 1.0f)
      : interpolateCorrection(degrees, 180.0f, 1.0f,
                              270.0f, 270.0f / 250.0f);
  return degrees * RIGHT_TURN_COUNTS_PER_DEGREE
      * previousCorrection * previousExtra * newCorrection * savedCorrection(3, degrees);
}

const int FORWARD_PWM  = 80;
const int BACKWARD_PWM = 80;
const int TURN_PWM     = 65;
const float DRIVE_KP = 0.15;
// v2.7 experimental FORWARD-ONLY straightness trim.
// Measured: F300 reached 300 mm but ended ~20 degrees to the right.
// Differential wheel path correction: track * radians(20) = ~65.3 mm.
// Command LESS left and MORE right wheel travel, while preserving mean travel.
// This is open-loop compensation, NOT a measured heading correction.
// If robot curves left instead, decrease this value and retest.
const float FORWARD_RIGHT_DRIFT_DEG_PER_300MM = 20.0f;
const float EFFECTIVE_TRACK_MM = 187.01f;
// Hard time limit prevents endless driving on disconnected/stalled encoders.
const unsigned long MOTION_MAX_MS = 20000UL;

// =====================================================
// =====================================================
// CONTROL
// =====================================================
bool stopRequested = false;

// =====================================================
// OUTPUT
// =====================================================
void sendText(String text)
{
  Serial.print(text);

  if (wifiClient && wifiClient.connected())
    wifiClient.print(text);
}

void sendLine(String text = "")
{
  Serial.println(text);

  if (wifiClient && wifiClient.connected())
    wifiClient.println(text);
}

// =====================================================
// MOTOR CONTROL
// =====================================================
void setLeftMotor(int pwm, bool forward)
{
  digitalWrite(
    LEFT_DIR,
    forward ? LEFT_FORWARD_DIR : !LEFT_FORWARD_DIR
  );

  analogWrite(
    LEFT_PWM,
    constrain(pwm, 0, 255)
  );
}

void setRightMotor(int pwm, bool forward)
{
  digitalWrite(
    RIGHT_DIR,
    forward ? RIGHT_FORWARD_DIR : !RIGHT_FORWARD_DIR
  );

  analogWrite(
    RIGHT_PWM,
    constrain(pwm, 0, 255)
  );
}

void stopMotors()
{
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

// =====================================================
// ENCODERS
// =====================================================
void leftEncoderISR()
{
  leftEncoderCount++;
}

void rightEncoderISR()
{
  rightEncoderCount++;
}

void resetEncoders()
{
  noInterrupts();

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  interrupts();
}

// =====================================================
// WIFI CONNECT
// =====================================================
void connectWiFi()
{
  // Use DHCP (default). No WiFi.config() call.
  if (!WIFI_SSID[0]) { Serial.println("WiFi disabled: configure SSID to enable port 23"); return; }

  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  int status = WL_IDLE_STATUS;

  unsigned long wifiStarted=millis();
  while (status != WL_CONNECTED && millis()-wifiStarted<8000UL)
  {
    status = WiFi.begin(
      WIFI_SSID,
      WIFI_PASS
    );

    Serial.print(".");
    delay(1500);
  }

  Serial.println();
  if(status != WL_CONNECTED) { Serial.println("WiFi unavailable; USB Serial control available"); return; }
  Serial.println("WIFI CONNECTED");

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  wifiServer.begin();

  Serial.println("Port: 23");
}

// =====================================================
// ACCEPT PUTTY
// =====================================================
void acceptWiFiClient()
{
  if (wifiClient && wifiClient.connected())
    return;

  WiFiClient newClient = wifiServer.available();

  if (newClient)
  {
    wifiClient = newClient;

    wifiClient.println();
    wifiClient.println("==============================");
    wifiClient.println("ROBOT-A MANUAL CALIBRATION v2.9");
    wifiClient.println("==============================");
    wifiClient.println();

    wifiClient.println("F 100   Forward");
    wifiClient.println("B 100   Backward");
    wifiClient.println("L 10    Left");
    wifiClient.println("R 10    Right");
    wifiClient.println("SENSORS");
    wifiClient.println("STOP");
    wifiClient.println("HELP");
    wifiClient.println();
  }
}

// =====================================================
// CHECK STOP WHILE MOVING
// =====================================================
void serviceWiFiDuringMotion()
{
  acceptWiFiClient();

  if (!wifiClient || !wifiClient.connected())
    return;

  while (wifiClient.available())
  {
    char c = wifiClient.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      wifiCommandBuffer.trim();
      wifiCommandBuffer.toUpperCase();

      if (wifiCommandBuffer == "STOP")
      {
        stopRequested = true;

        stopMotors();

        sendLine();
        sendLine("*** STOPPED ***");
      }

      wifiCommandBuffer = "";
    }
    else
    {
      wifiCommandBuffer += c;
    }
  }
}

// =====================================================
// FEEDBACK MOTION CONTROL v2.9 - original pinout and calibration preserved
// ENC_A interrupts count edges only. They do NOT establish true wheel direction.
// All travel estimates assume wheel rotates as commanded without slipping.
// =====================================================
const char* FW_VERSION = "Robot A automatic custom sequence (base: autonomous alignment firmware)";
const uint32_t CONTROL_PERIOD_MS = 20;
const uint32_t STALL_WINDOW_MS = 700;
const uint32_t NEUTRAL_MS = 120;
const float SPEED_CRUISE_CPS = 520.0f;
const float SPEED_APPROACH_CPS = 175.0f;
const float ACCEL_CPS2 = 650.0f;
const float BRAKE_CPS2 = 900.0f;
const float SPEED_KP = 0.065f;       // PWM / (encoder counts per second)
const float SPEED_KI = 0.075f;       // PWM / (count/s * sec)
const float SYNCHRONIZE_KP = 0.13f;  // counts/s per accumulated count error
const float PWM_SPEED_FEEDFORWARD = 0.12f;
const int MAX_DRIVE_PWM = 100;
const int MAX_TURN_PWM = 85;
const int MIN_RUNNING_PWM = 43;
// Historical 20 degree forward offset is an unconfirmed single measurement.
// Keep it as a readable reference but do not force the robot to arc by default.
const bool ENABLE_EXPERIMENTAL_FORWARD_TRIM = false;

String serialStopBuffer;
void serviceSerialStopDuringMotion() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      serialStopBuffer.trim(); serialStopBuffer.toUpperCase();
      if (serialStopBuffer=="STOP") { stopRequested=true; stopMotors(); }
      serialStopBuffer="";
    } else if (serialStopBuffer.length()<48) serialStopBuffer+=c;
    else serialStopBuffer="";
  }
}

// ---- TCRT5000 test constants preserved exactly ----
const byte LEFT_PIN = A0;
const byte CENTER_PIN = A1;
const byte RIGHT_PIN = A2;
const int LEFT_THRESHOLD = 65;
const int CENTER_THRESHOLD = 200;
const int RIGHT_THRESHOLD = 200;

bool missionActive = false;
bool lastMotionOK = true;
bool discLatched = false;
unsigned long lastSensorAt = 0;
unsigned long bootTime = 0;
bool autorunPending = false; // Starts automatically after a 500 ms startup delay.
unsigned int detectedDiscs = 0;

int readAverage(byte pin) {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return sum / 20;
}

DiscReading readDisc() {
  DiscReading d;
  d.left = readAverage(LEFT_PIN);
  d.center = readAverage(CENTER_PIN);
  d.right = readAverage(RIGHT_PIN);
  d.L = d.left > LEFT_THRESHOLD;
  d.C = d.center > CENTER_THRESHOLD;
  d.R = d.right > RIGHT_THRESHOLD;
  return d;
}
void showSensors() {
  DiscReading d=readDisc();
  sendLine(String("RAW L=")+d.left+" C="+d.center+" R="+d.right+
      " | DETECT="+int(d.L)+int(d.C)+int(d.R));
}
// === ADDITIVE ONE-DISC ALIGNMENT FEATURE (from supplied BACK100 sketch) ===
// New sketch uses HW-006 digital outputs: L=A1, C=A2, R=A3; LOW=disc.
// The legacy A0/A1/A2 analog thresholds above are intentionally untouched.
const int ALIGN_IR_L = A1, ALIGN_IR_C = A2, ALIGN_IR_R = A3;
const int ALIGN_ACTIVE = LOW;
const int ALIGN_PWM = 75;
const unsigned long ALIGN_TIMEOUT_MS = 5000;
const unsigned long ALIGN_SETTLE_MS = 35;
const unsigned long ALIGN_PULSE_MS = 12;
const unsigned long ALIGN_PULSE_SETTLE_MS = 14;
const unsigned long ALIGN_MAX_DEAD_ZONE_MS = 450;
const unsigned long ALIGN_SEARCH_LIMIT_MS = 8000;
bool alignmentSearchActive = false;
bool alignmentDiscFound = false;
bool alignmentFeatureBusy = false;
int alignmentTriggerL = HIGH, alignmentTriggerC = HIGH, alignmentTriggerR = HIGH;
unsigned long alignmentCandidateAt = 0;
// Require a stable clear floor before accepting a HIGH->LOW disc transition.
// If the sensor is stuck LOW on white floor, stop safely rather than inventing a disc.
const unsigned long ALIGN_DEBOUNCE_MS = 100;
const unsigned long ALIGN_CLEAR_VERIFY_MS = 350;
const unsigned long ALIGN_CLEAR_TIMEOUT_MS = 1500;
bool alignmentClearVerified = false;
void alignServiceStop();
bool alignAny();

String alignmentLevels() {
  return String(digitalRead(ALIGN_IR_L))+"/"+digitalRead(ALIGN_IR_C)+"/"+digitalRead(ALIGN_IR_R);
}

bool verifyClearFloor() {
  unsigned long begun=millis(), highSince=0;
  while (!stopRequested && millis()-begun < ALIGN_CLEAR_TIMEOUT_MS) {
    alignServiceStop();
    if (!alignAny()) {
      if (!highSince) highSince=millis();
      if (millis()-highSince >= ALIGN_CLEAR_VERIFY_MS) return true;
    } else highSince=0;
    delay(5);
  }
  return false;
}

bool alignL() { return digitalRead(ALIGN_IR_L) == ALIGN_ACTIVE; }
bool alignC() { return digitalRead(ALIGN_IR_C) == ALIGN_ACTIVE; }
bool alignR() { return digitalRead(ALIGN_IR_R) == ALIGN_ACTIVE; }
bool alignAny() { return alignL() || alignC() || alignR(); }
void alignServiceStop() { serviceWiFiDuringMotion(); serviceSerialStopDuringMotion(); }
void alignmentPulse(bool left) {
  // Neutral-first on direction changes, preserving the existing motor mapping.
  stopMotors(); delay(NEUTRAL_MS);
  if (stopRequested) return;
  setLeftMotor(ALIGN_PWM, !left);
  setRightMotor(ALIGN_PWM, left);
  unsigned long started = millis();
  while (!stopRequested && millis()-started < ALIGN_PULSE_MS) alignServiceStop();
  stopMotors();
  unsigned long settling = millis();
  while (!stopRequested && millis()-settling < ALIGN_PULSE_SETTLE_MS) alignServiceStop();
}

bool alignOneDiscToCenter() {
  alignmentFeatureBusy = true;
  sendLine("ALIGNMENT START: all three digital sensors must detect disc");
  stopMotors();
  delay(ALIGN_SETTLE_MS);
  int lastCorrection = 0;
  if (alignmentTriggerL == LOW && alignmentTriggerR != LOW) lastCorrection = -1;
  else if (alignmentTriggerR == LOW && alignmentTriggerL != LOW) lastCorrection = 1;
  unsigned long begun = millis(), noSensorSince = 0;
  while (!stopRequested && millis()-begun < ALIGN_TIMEOUT_MS) {
    alignServiceStop();
    if (stopRequested) break;
    bool l=alignL(),c=alignC(),r=alignR();
    if (l && c && r) {
      stopMotors();
      sendLine("ALIGNMENT SUCCESS: L/C/R = 1/1/1 (disc detected)");
      alignmentFeatureBusy=false;
      return true;
    }
    if (l && !r) {lastCorrection=-1;noSensorSince=0;alignmentPulse(true);continue;}
    if (r && !l) {lastCorrection=1;noSensorSince=0;alignmentPulse(false);continue;}
    if (c || (l && r)) noSensorSince=0;
    else {
      if (!noSensorSince) noSensorSince=millis();
      if (millis()-noSensorSince > ALIGN_MAX_DEAD_ZONE_MS) break;
    }
    if (l && r && !c) {stopMotors();delay(ALIGN_PULSE_SETTLE_MS);continue;}
    if (lastCorrection<0) alignmentPulse(true);
    else if (lastCorrection>0) alignmentPulse(false);
    else {stopMotors();delay(ALIGN_PULSE_SETTLE_MS);}
  }
  stopMotors();
  alignmentFeatureBusy=false;
  sendLine(String("ALIGNMENT FAILED: timeout / disc lost / STOP; final DIGITAL L/C/R=")+digitalRead(ALIGN_IR_L)+"/"+digitalRead(ALIGN_IR_C)+"/"+digitalRead(ALIGN_IR_R));
  return false;
}

// The search leg remains the original calibrated F310 command.
// On detection, motion exits as DISC_FOUND; afterwards B100 uses unchanged PI/calibration.
bool searchAlignAndBack100() {
  alignmentDiscFound=false;
  alignmentSearchActive=true;
  alignmentTriggerL=alignmentTriggerC=alignmentTriggerR=HIGH;
  alignmentCandidateAt=0;
  alignmentClearVerified=false;
  pinMode(ALIGN_IR_L, INPUT_PULLUP);
  pinMode(ALIGN_IR_C, INPUT_PULLUP);
  pinMode(ALIGN_IR_R, INPUT_PULLUP);
  sendLine("F310 DISC SEARCH: stable LOW on A1/A2/A3 stops motors");
  sendLine(String("SEARCH SENSOR LEVELS (L/C/R): ")+alignmentLevels());
  stopMotors();
  if (!verifyClearFloor()) {
    alignmentSearchActive=false;
    lastMotionOK=false;
    sendLine(String("SENSOR FAULT: No verified clear floor (HIGH/HIGH/HIGH). Levels=")+alignmentLevels());
    sendLine("Check A1/A2/A3 vs actual sensor OUT pins, sensor polarity, height, sensitivity and grounding. F310 NOT STARTED.");
    return false;
  }
  alignmentClearVerified=true;
  sendLine("SEARCH ARMED: clear floor verified. Waiting for a new stable LOW transition.");
  executeMotion('F',310);
  alignmentSearchActive=false;
  if (stopRequested || !lastMotionOK) return false;
  if (!alignmentDiscFound) {
    stopMotors();
    sendLine("NO DISC DETECTED WITHIN F310. MISSION HELD STOPPED.");
    lastMotionOK=false;
    return false;
  }
  if (!alignOneDiscToCenter()) {lastMotionOK=false;return false;}
  if (stopRequested) {lastMotionOK=false;return false;}
  sendLine("CENTERED -> B100 (using existing calibrated motion engine)");
  alignmentFeatureBusy=true;
  executeMotion('B',100);
  alignmentFeatureBusy=false;
  if (stopRequested || !lastMotionOK) return false;
  sendLine("POST-ALIGNMENT B100 COMPLETE; RESUMING ORIGINAL ROUTE");
  return true;
}

// Record centered detections without modifying a single route command.
// Stop for 2 s at the currently reached encoder count and resume remaining movement.
void serviceDiscDuringMotion() {
  // The legacy analog sensor system shares A1/A2 with digital alignment.
  // Do not sample analog values during an autonomous digital-sensor mission.
  if (missionActive || alignmentSearchActive || alignmentFeatureBusy || millis()-lastSensorAt < 120) return;
  lastSensorAt=millis();
  DiscReading d=readDisc();
  if (!d.L && !d.C && !d.R) discLatched=false;
  if (!d.C || discLatched) return;
  discLatched=true;
  detectedDiscs++;
  stopMotors();
  sendLine(String("CENTER DISC DETECTED #")+detectedDiscs+"; HOLD 2000ms");
  unsigned long heldAt=millis();
  while (!stopRequested && millis()-heldAt<2000UL) {
    serviceWiFiDuringMotion();
    serviceSerialStopDuringMotion();
    delay(5);
  }
  stopMotors();
}

void readEncoders(long &l, long &r) {
  noInterrupts(); l=leftEncoderCount; r=rightEncoderCount; interrupts();
}
float clampFloat(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
int wheelOutput(WheelPI &c, float demand, float measurement, float dt, int maximum) {
  c.measured=measurement;
  if (demand < 0.5f) {c.integral=0; c.pwm=0; return 0;}
  float error=demand-measurement;
  float ff=PWM_SPEED_FEEDFORWARD*demand;
  float unsat=ff+SPEED_KP*error+c.integral;
  float clipped=clampFloat(unsat, MIN_RUNNING_PWM, maximum);
  // Conditional integration with anti-windup at both rails.
  if ((unsat >= MIN_RUNNING_PWM && unsat <= maximum) ||
      (unsat > maximum && error < 0) || (unsat < MIN_RUNNING_PWM && error > 0)) {
    c.integral=clampFloat(c.integral+SPEED_KI*error*dt,-50.0f,50.0f);
    clipped=clampFloat(ff+SPEED_KP*error+c.integral,MIN_RUNNING_PWM,maximum);
  }
  c.pwm=(int)(clipped+0.5f);
  return c.pwm;
}
void executeMotion(char kind, float amount) {
  if (!isfinite(amount) || amount<=0 || amount>720) {sendLine("ERROR: motion outside 0..720");return;}
  bool isTurn = kind=='L' || kind=='R';
  bool leftForward=(kind=='F'||kind=='R');
  bool rightForward=(kind=='F'||kind=='L');
  float base = isTurn ? ((kind=='L') ? correctedLeftTurnCounts(amount):correctedRightTurnCounts(amount))
                      : correctedDistanceTargetMM(amount,kind=='F')*COUNTS_PER_MM;
  if (!isfinite(base) || base<1 || base>50000) {sendLine("ERROR: invalid target");return;}
  float tLeft=base, tRight=base;
  if (kind=='F' && ENABLE_EXPERIMENTAL_FORWARD_TRIM) {
    float radians=FORWARD_RIGHT_DRIFT_DEG_PER_300MM*0.01745329252f*(amount/300.0f);
    float half=EFFECTIVE_TRACK_MM*radians*COUNTS_PER_MM*0.5f;
    tLeft=base-half; tRight=base+half;
  }
  if (tLeft<1 || tRight<1) {sendLine("ERROR: invalid wheel targets");return;}
  long goalL=lroundf(tLeft),goalR=lroundf(tRight);
  stopMotors(); delay(NEUTRAL_MS); // motor driver direction only changes at zero PWM
  resetEncoders(); stopRequested=false;
  sendLine();sendLine(String("FIRMWARE=")+FW_VERSION+" CAL_SCHEMA=1 LEGACY_BASELINE=v2.8");
  sendLine(String("COMMAND=")+kind+" "+String(amount,2)+" L_TARGET="+String(goalL)+" R_TARGET="+String(goalR));
  sendLine(String("CAL_FACTOR=")+String(savedCorrection(kindIndex(kind),amount),6)+
           " LEGACY_FORWARD_TRIM="+(ENABLE_EXPERIMENTAL_FORWARD_TRIM?"ON":"OFF"));
  unsigned long start=millis(),last=start,lastReport=start;
  unsigned long lastLeftProgress=start,lastRightProgress=start;
  long lastL=0,lastR=0,latestL=0,latestR=0;
  float vDesired=0,measuredL=0,measuredR=0;
  WheelPI ctrlL={0,0,0},ctrlR={0,0,0};
  String reason="COMPLETE";
  while (true) {
    serviceWiFiDuringMotion();serviceSerialStopDuringMotion();
    unsigned long now=millis();
    if (stopRequested) {reason="USER_STOP";break;}
    if (now-start > MOTION_MAX_MS) {reason="TIMEOUT";break;}
    if (now-last < CONTROL_PERIOD_MS) continue;
    float dt=(now-last)*0.001f;last=now;
    // Sampling is done with motor PWM temporarily zero on detection.
    unsigned long sensorStart=millis();
    if (alignmentSearchActive) {
      if (alignmentClearVerified && alignAny()) {
        if (alignmentCandidateAt == 0) alignmentCandidateAt = now;
        if (now-alignmentCandidateAt >= ALIGN_DEBOUNCE_MS) {
          stopMotors();
          alignmentTriggerL=digitalRead(ALIGN_IR_L);
          alignmentTriggerC=digitalRead(ALIGN_IR_C);
          alignmentTriggerR=digitalRead(ALIGN_IR_R);
          alignmentDiscFound=true;
          sendLine(String("DISC CONFIRMED: DIGITAL L/C/R=")+alignmentTriggerL+"/"+alignmentTriggerC+"/"+alignmentTriggerR);
          reason="DISC_FOUND";
          break;
        }
      } else alignmentCandidateAt=0;
    } else serviceDiscDuringMotion();
    unsigned long sensorElapsed=millis()-sensorStart;
    if (sensorElapsed>=1000UL) {
      start += sensorElapsed;
      lastLeftProgress += sensorElapsed;
      lastRightProgress += sensorElapsed;
      last = millis();
      vDesired=0;
      ctrlL.integral=0; ctrlR.integral=0;
      if (stopRequested) { reason="USER_STOP"; break; }
      continue;
    }
    readEncoders(latestL,latestR);
    float remainL=goalL-latestL,remainR=goalR-latestR;
    if (remainL<=2 && remainR<=2) break;
    if (latestL!=lastL) lastLeftProgress=now;
    if (latestR!=lastR) lastRightProgress=now;
    measuredL=(latestL-lastL)/dt;measuredR=(latestR-lastR)/dt;
    lastL=latestL;lastR=latestR;
    if ((remainL>5 && now-lastLeftProgress>STALL_WINDOW_MS) ||
        (remainR>5 && now-lastRightProgress>STALL_WINDOW_MS)) {reason="ENCODER_STALL";break;}
    // Trapezoidal speed envelope: rate-limited acceleration and distance-dependent deceleration.
    float remaining= fmaxf(0.0f, fminf(remainL,remainR));
    float brakeSpeed=sqrtf(2.0f*BRAKE_CPS2*remaining);
    float profile=clampFloat(brakeSpeed,0,SPEED_CRUISE_CPS);
    vDesired=fminf(vDesired+ACCEL_CPS2*dt,profile);
    // Below minimum observable speed the final 2-count tolerance handles completion.
    if (remaining>3) vDesired=fmaxf(vDesired,SPEED_APPROACH_CPS);
    float progressL=clampFloat((float)latestL/goalL,0,1);
    float progressR=clampFloat((float)latestR/goalR,0,1);
    float balance=(progressL-progressR)*0.5f*(goalL+goalR);
    float adjustment=clampFloat(balance*SYNCHRONIZE_KP,-110,110);
    float desiredL=(remainL<=2)?0:clampFloat(vDesired-adjustment,SPEED_APPROACH_CPS,SPEED_CRUISE_CPS);
    float desiredR=(remainR<=2)?0:clampFloat(vDesired+adjustment,SPEED_APPROACH_CPS,SPEED_CRUISE_CPS);
    int limit=isTurn?MAX_TURN_PWM:MAX_DRIVE_PWM;
    int pwmL=wheelOutput(ctrlL,desiredL,measuredL,dt,limit);
    int pwmR=wheelOutput(ctrlR,desiredR,measuredR,dt,limit);
    setLeftMotor(pwmL,leftForward);setRightMotor(pwmR,rightForward);
    if (now-lastReport>=250) {
      lastReport=now;
      sendLine(String("PROGRESS L=")+latestL+"/"+goalL+" R="+latestR+"/"+goalR+
               " SPEED_CPS="+String(measuredL,0)+","+String(measuredR,0)+
               " PWM="+String(pwmL)+","+String(pwmR));
    }
  }
  stopMotors();delay(150);
  lastMotionOK=(reason=="COMPLETE" || (alignmentSearchActive && reason=="DISC_FOUND"));
  if (!lastMotionOK) stopRequested=true;
  readEncoders(latestL,latestR);
  float meanCounts=0.5f*(latestL+latestR);
  float estimated= isTurn ? (meanCounts/(kind=='L'?LEFT_TURN_COUNTS_PER_DEGREE:RIGHT_TURN_COUNTS_PER_DEGREE))
                          : meanCounts/COUNTS_PER_MM;
  float signedHeading=(kind=='F'?1.0f:(kind=='B'?-1.0f:0.0f))*
                      ((latestR-latestL)/COUNTS_PER_MM)/EFFECTIVE_TRACK_MM*57.29578f;
  sendLine(String("RESULT reason=")+reason+" command="+kind+" "+String(amount,2));
  sendLine(String("ENC L=")+latestL+"/"+goalL+" R="+latestR+"/"+goalR+
           " mean="+String(meanCounts,1));
  sendLine(String("SPEED_LAST_CPS L=")+String(measuredL,1)+" R="+String(measuredR,1)+
           " LAST_PWM="+ctrlL.pwm+","+ctrlR.pwm);
  sendLine(String("ODOM_RAW=")+String(estimated,2)+(isTurn?" deg (uncalibrated proxy)":" mm")+
           " TRACK_HEADING_DELTA="+String(signedHeading,2)+" deg (encoder-only)");
  sendLine(String("TARGET_PROGRESS=")+String(100.0f*meanCounts/base,2)+"% elapsed_ms="+String(millis()-start));
  if (reason=="DISC_FOUND") sendLine("SEARCH STOPPED: disc trigger accepted; starting alignment");
  else if (reason!="COMPLETE") sendLine("MOTION ABORTED: inspect encoders, wiring and mechanics before retry");
}
void moveForwardMM(float mm) {executeMotion('F',mm);}
void moveBackwardMM(float mm) {executeMotion('B',mm);}
void turnLeftDegrees(float degrees) {executeMotion('L',degrees);}
void turnRightDegrees(float degrees) {executeMotion('R',degrees);}

// =====================================================
// HELP
// =====================================================
void showHelp()
{
  sendLine();
  sendLine("==============================");
  sendLine("COMMANDS");
  sendLine("==============================");

  sendLine("Automatic route: 21 movements + left gate (D11) then right gate (D10), no delay; calibration/EEPROM retained");
  sendLine("CALSHOW | CALSAVE | CALLOAD | CALRESET YES | STOP");
  sendLine("NOTE: CALRESET YES overwrites unknown EEPROM; backup first");
  sendLine("CAL L 270 315  (command L270 measured 315; save correction)");
  sendLine("CAL R 200 200  (store measured R200, no angle change)");
  sendLine("CAL F 300 300  (record NEW measured test only)");
  sendLine("CALSHOW        show stored corrections");
  sendLine("CALRESET       clear stored corrections (requires confirm)");
  sendLine("CALRESET YES   execute EEPROM calibration reset");
  sendLine("F 100     Forward 100 mm");
  sendLine("F 37.5    Forward 37.5 mm");

  sendLine("B 100     Backward 100 mm");

  sendLine("L 14      Left 14 degrees");
  sendLine("L 2.5     Left 2.5 degrees");

  sendLine("R 25      Right 25 degrees");
  sendLine("R 1       Right 1 degree");

  sendLine("SENSORS   Legacy analog diagnostics; autonomous alignment uses digital A1/A2/A3");

  sendLine("STOP      Stop immediately");

  sendLine("HELP");

  sendLine();
}

// Opens one gate immediately (no delay): motors stopped, servo pulsed open for
// GATE_PULSE_MS, then returned to its neutral pulse before movement resumes.
void openGate(Servo &servo, bool &attached, byte pin, int neutralUs, int openUs, const char* name) {
  stopMotors();
  if (stopRequested) return;

  if (!attached) {
    servo.attach(pin);
    servo.writeMicroseconds(neutralUs);
    attached = true;
  }

  sendLine(String("GATE ")+name+": opening pulse");
  servo.writeMicroseconds(openUs);
  const unsigned long pulseStarted = millis();
  while (!stopRequested && millis() - pulseStarted < GATE_PULSE_MS) {
    serviceWiFiDuringMotion();
    serviceSerialStopDuringMotion();
    if (stopRequested) break;
    delay(5);
  }

  servo.writeMicroseconds(neutralUs);
  sendLine(stopRequested ? String("GATE ")+name+": stopped; servo returned to neutral" :
                           String("GATE ")+name+": opening pulse complete; continuing route");
}

void openLeftGate()  { openGate(leftGateServo,  leftGateServoAttached,  LEFT_GATE_SERVO_PIN,  LEFT_GATE_NEUTRAL_US,  LEFT_GATE_OPEN_US,  "LEFT (D11)"); }
void openRightGate() { openGate(rightGateServo, rightGateServoAttached, RIGHT_GATE_SERVO_PIN, RIGHT_GATE_NEUTRAL_US, RIGHT_GATE_OPEN_US, "RIGHT (D10)"); }

// Custom route: first gate point opens the LEFT gate (D11), second gate point opens the RIGHT gate (D10). No delay before either.
void runSequence() {
  missionActive = true;
  stopRequested = false;
  lastMotionOK = true;
  sendLine("CUSTOM MOVEMENT SEQUENCE START");

  turnLeftDegrees(10);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveForwardMM(90);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  turnLeftDegrees(15);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveForwardMM(50);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  turnLeftDegrees(75);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveForwardMM(320);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  turnRightDegrees(20);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveForwardMM(50);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveForwardMM(260);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveBackwardMM(260);
  if (stopRequested || !lastMotionOK) goto sequence_abort;

  turnRightDegrees(30);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveForwardMM(50);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  turnRightDegrees(60);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  openLeftGate();
  if (stopRequested) goto sequence_abort;
  moveForwardMM(250);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveBackwardMM(120);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveForwardMM(50);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  turnRightDegrees(30);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveForwardMM(20);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  turnRightDegrees(20);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  openRightGate();
  if (stopRequested) goto sequence_abort;
  moveForwardMM(200);
  if (stopRequested || !lastMotionOK) goto sequence_abort;
  moveBackwardMM(250);
  if (stopRequested || !lastMotionOK) goto sequence_abort;

  stopMotors();
  missionActive = false;
  sendLine("CUSTOM MOVEMENT SEQUENCE COMPLETE. Motors OFF.");
  return;

sequence_abort:
  stopMotors();
  missionActive = false;
  sendLine("SEQUENCE ABORTED. Motors OFF. No automatic restart.");
}

// =====================================================
// COMMAND PROCESSOR
// =====================================================
void processCommand(String command)
{
  command.trim();
  command.toUpperCase();

  if (command.length() == 0)
    return;

  sendLine();
  sendText("> ");
  sendLine(command);

  if(command=="CALSAVE") {if (!calibrationValid) sendLine("PROTECTED: invalid/unknown EEPROM; CALRESET YES required"); else sendLine(saveCalibration()?"EEPROM saved and verified":"EEPROM WRITE VERIFY FAILED");return;}
  if(command=="CALLOAD") {loadCalibration();sendLine("EEPROM reload attempted");return;}
  if(command=="CALSHOW") {
    sendLine(String("FW=")+FW_VERSION+" schema="+String(CAL_VERSION)+" saved_valid="+(calibrationValid?"YES":"NO"));
    const char labels[]="FBLR";
    for(int k=0;k<4;k++) for(int i=0;i<8;i++) {
      const CalibrationPoint &p=calibration.points[k][i];
      if(p.command>0) sendLine(String(labels[k])+" "+String(p.command,1)+" x"+String(p.multiplier,5));
    }
    return;
  }
  if(command=="CALRESET") {sendLine("To clear saved anchors enter CALRESET YES");return;}
  if(command=="CALRESET YES") {resetCalibrationRAM();bool ok=saveCalibration();calibrationValid=ok;sendLine(ok?"Saved anchors cleared and verified":"RESET WRITE FAILED");return;}
  if(command.startsWith("CAL ")) {
    char motion=0; float requested=0, measured=0; char extra=0;
    int count=sscanf(command.c_str(),"CAL %c %f %f %c",&motion,&requested,&measured,&extra);
    if(count==3 && recordCalibration(motion,requested,measured))
       sendLine("Stored measured correction in EEPROM. Verify physically before using in mission.");
    else {
      if (count!=3 || kindIndex(motion)<0) sendLine("CAL rejected: syntax. Example CAL F 100 95");
      else if(!calibrationValid) sendLine("CAL rejected: EEPROM not initialized or invalid. CALSHOW to inspect. CALRESET YES only after backing up any valid data; this overwrites EEPROM.");
      else if(requested<1 || requested>720 || measured<=0 || requested/measured<0.7f || requested/measured>1.3f)
        sendLine("CAL rejected: range. Command 1..720; measured >0; requested/measured 0.7..1.3.");
      else sendLine("CAL rejected: no anchor slots, correction outside 0.5..1.5, or EEPROM write verification failed.");
    }
    return;
  }
  // ---------------------------------------------
  // STOP
  // ---------------------------------------------
  if (command == "STOP")
  {
    stopRequested = true;
    autorunPending = false;

    stopMotors();

    sendLine("*** STOPPED ***");

    return;
  }

  // ---------------------------------------------
  // SENSOR
  // ---------------------------------------------
  if (command == "SENSORS")
  {
      showSensors();
      return;
  }

  // ---------------------------------------------
  // HELP
  // ---------------------------------------------
  if (command == "HELP")
  {
    showHelp();
    return;
  }

  // ---------------------------------------------
  // FORWARD
  // ---------------------------------------------
  if (command.startsWith("F "))
  {
    float value =
      command.substring(2).toFloat();

    if (value > 0)
      moveForwardMM(value);
    else
      sendLine("Invalid distance");

    return;
  }

  // ---------------------------------------------
  // BACKWARD
  // ---------------------------------------------
  if (command.startsWith("B "))
  {
    float value =
      command.substring(2).toFloat();

    if (value > 0)
      moveBackwardMM(value);
    else
      sendLine("Invalid distance");

    return;
  }

  // ---------------------------------------------
  // LEFT
  // ---------------------------------------------
  if (command.startsWith("L "))
  {
    float value =
      command.substring(2).toFloat();

    if (value > 0)
      turnLeftDegrees(value);
    else
      sendLine("Invalid angle");

    return;
  }

  // ---------------------------------------------
  // RIGHT
  // ---------------------------------------------
  if (command.startsWith("R "))
  {
    float value =
      command.substring(2).toFloat();

    if (value > 0)
      turnRightDegrees(value);
    else
      sendLine("Invalid angle");

    return;
  }

  sendLine("Unknown command");
}

// =====================================================
// NORMAL WIFI HANDLER
// =====================================================
void handleWiFi()
{
  acceptWiFiClient();

  if (!wifiClient || !wifiClient.connected())
    return;

  while (wifiClient.available())
  {
    char c = wifiClient.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      String command =
        wifiCommandBuffer;

      wifiCommandBuffer = "";

      processCommand(command);
    }
    else
    {
      wifiCommandBuffer += c;
    }
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("Robot A auto sequence + left (D11) and right (D10) gate servos; base calibration/EEPROM retained");

  delay(1000);

  // Motors
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  stopMotors();

  // Encoders
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);

  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENC_A),
    leftEncoderISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENC_A),
    rightEncoderISR,
    CHANGE
  );

  loadCalibration();

  analogReadResolution(10);
  pinMode(LEFT_PIN, INPUT);
  pinMode(CENTER_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);

  // WiFi
  connectWiFi();

  Serial.println();
  Serial.println("==============================");
  Serial.println("ROBOT A READY - route starts automatically after 500 ms; STOP to abort");
  Serial.println("==============================");
  bootTime=millis();
  stopRequested=false;
  autorunPending=true;
  sendLine("AUTO-START ENABLED: sequence will retry after 500 ms. Send STOP to cancel/abort.");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  handleWiFi();
  // Automatic one-shot start after a 500 ms safety delay; STOP can cancel it.
  if (autorunPending && !stopRequested && millis() - bootTime >= 500UL) {
    autorunPending = false;
    runSequence();
  }
  static String serialCommand="";
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n') {processCommand(serialCommand);serialCommand="";}
    else if(serialCommand.length()<80) serialCommand+=c;
    else serialCommand="";
  }
}