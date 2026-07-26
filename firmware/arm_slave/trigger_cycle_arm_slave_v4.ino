/******************************************************************************
  Trigger-Cycle Slave (arm side) v4 -- locks in your safe-tested jog limits
  and moves choreo/demo control fully onto the serial monitor.

  WHAT CHANGED FROM v3:
    - Shoulder/Elbow/Wrist1/Wrist2 min/max are now hardcoded from your bench
      testing (see the jointMin*/jointMax* block) and jogHasRun* is pre-set
      true for those four. You do NOT need to re-jog them every session.
    - Gripper is the ONLY joint still a placeholder -- jog it live with key
      '5' before running demos, since it's the one still being tuned.
    - Removed the "CMD:NEXT_DEMO" Bluetooth listener. The glove (master) was
      never actually sending it -- that was leftover/aspirational text from
      an earlier draft. Demos are now driven exclusively from the serial
      monitor with 'g', so the glove can stay focused on live gesture
      control (modes 1-4) without also juggling demo triggers.
    - A genuine gesture command (mode 1-4 from the glove) at any point still
      immediately interrupts whatever demo is playing and hands control
      back to your hand -- that safety behavior is unchanged.

  Everything else from v3 (gripper as mode 4 driven by roll, the SHOW engine
  and its 4 routines, velocity/deadzone live control) is unchanged.

  ============================================================================
  WORKFLOW:
  ============================================================================
    1. Flash this. Open Serial Monitor @ 9600.
    2. Shoulder/Elbow/Wrist1/Wrist2 limits are already loaded -- just jog the
       Gripper: press '5', then '+'/'-' (nudge 1) or '>'/'<' (nudge 5), 'n'
       to mark MIN, 'x' to mark MAX, 'q' to exit jog and lock it in.
    3. (Optional) pose the arm and press 't' for Stand Tall, 'd' for Full Down.
    4. Press 'g' on the serial monitor to play the next demo in the cycle.
       Demos are serial-only -- there is no glove tap combo for this anymore.
    5. Any live gesture (glove modes 1-4) interrupts a demo instantly.

  GLOVE MODE MAP (unchanged, from master):
    1 tap  -> Shoulder      2 taps -> Elbow
    3 taps -> Wrist (pitch=Wrist1, roll=Wrist2)
    4 taps -> Gripper (roll opens/closes it; pitch unused in this mode)
******************************************************************************/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
SoftwareSerial BT(10, 11); // RX, TX -- from glove master board

#define SERVOMIN 125
#define SERVOMAX 575
#define SERVO_FREQ 50

// ---- Channel assignment (fixed wiring, don't change) ----
const int CH_SHOULDER_L = 0;
const int CH_SHOULDER_R = 1;
const int CH_ELBOW      = 2;
const int CH_WRIST1     = 3;
const int CH_WRIST2     = 4;
const int CH_GRIPPER    = 5;

const int L_CENTER = 125;
const int R_CENTER = 120;

// ---- Starting positions for the five joints under live control ----
const int shoulderStartCenter = 0;   // delta
const int elbowStartCenter    = 90;  // degrees
const int wrist1StartCenter   = 25;  // degrees
const int wrist2StartCenter   = 90;  // degrees
const int gripperStartCenter  = 122; // degrees

// ---- Shoulder/Elbow/Wrist1/Wrist2: safe-tested limits, locked in so you
// don't have to re-jog them every session. Gripper is intentionally left
// as a placeholder below -- jog that one live with key '5'. ----
int jointMinShoulder = -60,  jointMaxShoulder = 15;
int jointMinElbow    = 0,    jointMaxElbow    = 110;
int jointMinWrist1   = 10,   jointMaxWrist1   = 145;
int jointMinWrist2   = 5,    jointMaxWrist2   = 170;
int jointMinGripper  = gripperStartCenter - 20,  jointMaxGripper  = gripperStartCenter + 20; // still placeholder -- jog this one
bool jogHasRunShoulder = true, jogHasRunElbow = true, jogHasRunWrist1 = true, jogHasRunWrist2 = true, jogHasRunGripper = false;

// ---- Named poses for the Stand Tall <-> Full Down demo ----
// TODO: replace with your measured angles, or capture live with 't' / 'd'
// (serial commands, RUN mode only) -- they'll print back as CSV to paste here.
int standTallShoulder = shoulderStartCenter, standTallElbow = elbowStartCenter,
    standTallWrist1 = wrist1StartCenter, standTallWrist2 = wrist2StartCenter, standTallGripper = gripperStartCenter;
int fullDownShoulder = shoulderStartCenter, fullDownElbow = elbowStartCenter,
    fullDownWrist1 = wrist1StartCenter, fullDownWrist2 = wrist2StartCenter, fullDownGripper = gripperStartCenter;

// ---- Deadzone / speed ramp for LIVE control, tune to taste ----
const float DEADZONE_DEG  = 5.0;
const float ZONE_SLOW_MAX = 20.0;
const float ZONE_MED_MAX  = 40.0;
const int SPEED_SLOW = 1;
const int SPEED_MED  = 2;
const int SPEED_FAST = 4;

bool INVERT_SHOULDER = false, INVERT_ELBOW = false, INVERT_WRIST1 = false, INVERT_WRIST2 = false, INVERT_GRIPPER = false;

enum Mode { RUN, JOG, SHOW };
Mode mode = RUN;

enum JogTarget { JOG_NONE, JOG_SHOULDER, JOG_ELBOW, JOG_WRIST1, JOG_WRIST2, JOG_GRIPPER };
JogTarget jogTarget = JOG_NONE;
int jogPos = 0;

const int SLEW_DELAY_MS = 15;
const unsigned long STALE_MS = 500;

int currentPosShoulder = shoulderStartCenter, targetPosShoulder = shoulderStartCenter;
int currentPosElbow    = elbowStartCenter,    targetPosElbow    = elbowStartCenter;
int currentPosWrist1   = wrist1StartCenter,   targetPosWrist1   = wrist1StartCenter;
int currentPosWrist2   = wrist2StartCenter,   targetPosWrist2   = wrist2StartCenter;
int currentPosGripper  = gripperStartCenter,  targetPosGripper  = gripperStartCenter;

unsigned long lastPacketTime = 0;
unsigned long lastPrint = 0;
int lastMode = 1;
float lastDeltaPitch = 0, lastDeltaRoll = 0;

// ============================================================================
// SHOW ENGINE -- two sub-engines: a generic step queue (used by Calib Sweep,
// Breathe, and Pose Cycle) and a dedicated Wave engine (staggered/overlapping
// timing that a simple step queue can't express cleanly).
// ============================================================================
const int SKIP_JOINT = 32000; // sentinel meaning "leave this joint where it is"

struct ShowStep {
  int shoulder, elbow, wrist1, wrist2, gripper; // target angle, or SKIP_JOINT
  unsigned long duration;                        // ms to ease into this step
};
const int MAX_SHOW_STEPS = 30;
ShowStep showSteps[MAX_SHOW_STEPS];
int showStepCount = 0, showStepIndex = 0;
unsigned long showStepStart = 0;
int stepFromShoulder, stepFromElbow, stepFromWrist1, stepFromWrist2, stepFromGripper;

bool usingWaveEngine = false;
unsigned long waveStart = 0;
bool waveOutPhase = true;
int waveOrder[5]; // permutation of {0..4} = {shoulder,elbow,wrist1,wrist2,gripper}, arrival order
const unsigned long WAVE_TRAVEL_MS  = 1000; // time for ONE joint's own out (or back) travel
const unsigned long WAVE_STAGGER_MS = 180;  // delay between each joint's start

int demoIndex = 0;
const int NUM_DEMOS = 4;

float smoothstepf(float t) {
  t = constrain(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}
float lerpf(float a, float b, float t) { return a + (b - a) * t; }
int roundToInt(float v) { return (int)(v + (v >= 0 ? 0.5 : -0.5)); }

bool allJogged() {
  return jogHasRunShoulder && jogHasRunElbow && jogHasRunWrist1 && jogHasRunWrist2 && jogHasRunGripper;
}

void addStepRaw(int sh, int el, int w1, int w2, int gr, unsigned long dur) {
  if (showStepCount >= MAX_SHOW_STEPS) return;
  showSteps[showStepCount++] = { sh, el, w1, w2, gr, dur };
}
void addSingleJointStep(int jointIdx, int value, unsigned long dur) {
  int v[5] = { SKIP_JOINT, SKIP_JOINT, SKIP_JOINT, SKIP_JOINT, SKIP_JOINT };
  v[jointIdx] = value;
  addStepRaw(v[0], v[1], v[2], v[3], v[4], dur);
}
void addAllJointsStep(int v[5], unsigned long dur) {
  addStepRaw(v[0], v[1], v[2], v[3], v[4], dur);
}

void startShowStep(int idx) {
  showStepIndex = idx;
  showStepStart = millis();
  stepFromShoulder = currentPosShoulder;
  stepFromElbow    = currentPosElbow;
  stepFromWrist1   = currentPosWrist1;
  stepFromWrist2   = currentPosWrist2;
  stepFromGripper  = currentPosGripper;
}

void beginStepQueueShow() {
  mode = SHOW;
  usingWaveEngine = false;
  startShowStep(0);
}

void updateStepQueueShow() {
  if (showStepIndex >= showStepCount) { mode = RUN; return; }
  ShowStep &s = showSteps[showStepIndex];
  unsigned long elapsed = millis() - showStepStart;
  float t = (s.duration == 0) ? 1.0 : (float)elapsed / (float)s.duration;
  float e = smoothstepf(t);

  if (s.shoulder != SKIP_JOINT) currentPosShoulder = roundToInt(lerpf(stepFromShoulder, s.shoulder, e));
  if (s.elbow    != SKIP_JOINT) currentPosElbow    = roundToInt(lerpf(stepFromElbow,    s.elbow,    e));
  if (s.wrist1   != SKIP_JOINT) currentPosWrist1   = roundToInt(lerpf(stepFromWrist1,   s.wrist1,   e));
  if (s.wrist2   != SKIP_JOINT) currentPosWrist2   = roundToInt(lerpf(stepFromWrist2,   s.wrist2,   e));
  if (s.gripper  != SKIP_JOINT) currentPosGripper  = roundToInt(lerpf(stepFromGripper,  s.gripper,  e));

  writeShoulder(currentPosShoulder);
  writeElbow(currentPosElbow);
  writeWrist1(currentPosWrist1);
  writeWrist2(currentPosWrist2);
  writeGripper(currentPosGripper);

  if (t >= 1.0) {
    targetPosShoulder = currentPosShoulder; targetPosElbow = currentPosElbow;
    targetPosWrist1 = currentPosWrist1;     targetPosWrist2 = currentPosWrist2;
    targetPosGripper = currentPosGripper;
    showStepIndex++;
    if (showStepIndex < showStepCount) startShowStep(showStepIndex);
    else mode = RUN; // demo finished, hand control back
  }
}

void startWaveShow() {
  mode = SHOW;
  usingWaveEngine = true;
  waveOutPhase = true;
  waveStart = millis();
  for (int i = 0; i < 5; i++) waveOrder[i] = i;
  randomSeed(micros());
  for (int i = 4; i > 0; i--) { // Fisher-Yates -- fresh cascade order every run
    int j = random(0, i + 1);
    int tmp = waveOrder[i]; waveOrder[i] = waveOrder[j]; waveOrder[j] = tmp;
  }
}

void updateWaveShow() {
  int mins[5]   = { jointMinShoulder, jointMinElbow, jointMinWrist1, jointMinWrist2, jointMinGripper };
  int maxs[5]   = { jointMaxShoulder, jointMaxElbow, jointMaxWrist1, jointMaxWrist2, jointMaxGripper };
  int starts[5] = { shoulderStartCenter, elbowStartCenter, wrist1StartCenter, wrist2StartCenter, gripperStartCenter };
  int pos[5];
  unsigned long elapsed = millis() - waveStart;
  bool anyActive = false;

  for (int rank = 0; rank < 5; rank++) {
    int j = waveOrder[rank]; // which joint arrives at this rank in the cascade
    long localElapsed = (long)elapsed - (long)(rank * WAVE_STAGGER_MS);
    float t = (localElapsed <= 0) ? 0.0 : (float)localElapsed / (float)WAVE_TRAVEL_MS;
    if (t < 1.0) anyActive = true;
    float e = smoothstepf(t);
    int fromV = waveOutPhase ? starts[j] : maxs[j];
    int toV   = waveOutPhase ? maxs[j]   : starts[j];
    pos[j] = roundToInt(lerpf(fromV, toV, e));
  }

  currentPosShoulder = pos[0]; currentPosElbow = pos[1]; currentPosWrist1 = pos[2];
  currentPosWrist2 = pos[3];   currentPosGripper = pos[4];
  writeShoulder(currentPosShoulder); writeElbow(currentPosElbow);
  writeWrist1(currentPosWrist1);     writeWrist2(currentPosWrist2); writeGripper(currentPosGripper);

  if (!anyActive) {
    if (waveOutPhase) {
      waveOutPhase = false; // ripple back home
      waveStart = millis();
    } else {
      targetPosShoulder = currentPosShoulder; targetPosElbow = currentPosElbow;
      targetPosWrist1 = currentPosWrist1;     targetPosWrist2 = currentPosWrist2;
      targetPosGripper = currentPosGripper;
      mode = RUN; // full cycle done, hand control back
    }
  }
}

void startCalibSweepShow() {
  showStepCount = 0;
  int mins[5]   = { jointMinShoulder, jointMinElbow, jointMinWrist1, jointMinWrist2, jointMinGripper };
  int maxs[5]   = { jointMaxShoulder, jointMaxElbow, jointMaxWrist1, jointMaxWrist2, jointMaxGripper };
  int starts[5] = { shoulderStartCenter, elbowStartCenter, wrist1StartCenter, wrist2StartCenter, gripperStartCenter };

  addAllJointsStep(starts, 800); // known baseline before the sweep starts
  for (int j = 0; j < 5; j++) {
    addSingleJointStep(j, mins[j],   1000);
    addSingleJointStep(j, mins[j],   400);  // brief hold at min
    addSingleJointStep(j, maxs[j],   1400);
    addSingleJointStep(j, maxs[j],   400);  // brief hold at max
    addSingleJointStep(j, starts[j], 900);  // back to center before the next joint
  }
  beginStepQueueShow();
}

void startBreatheShow() {
  showStepCount = 0;
  int mins[5]   = { jointMinShoulder, jointMinElbow, jointMinWrist1, jointMinWrist2, jointMinGripper };
  int maxs[5]   = { jointMaxShoulder, jointMaxElbow, jointMaxWrist1, jointMaxWrist2, jointMaxGripper };
  int starts[5] = { shoulderStartCenter, elbowStartCenter, wrist1StartCenter, wrist2StartCenter, gripperStartCenter };

  addAllJointsStep(starts, 600);
  for (int cycle = 0; cycle < 3; cycle++) {
    addAllJointsStep(maxs, 1200);
    addAllJointsStep(maxs, 300);
    addAllJointsStep(mins, 1400);
    addAllJointsStep(mins, 300);
  }
  addAllJointsStep(starts, 900);
  beginStepQueueShow();
}

void startPoseCycleShow() {
  showStepCount = 0;
  int tall[5] = { standTallShoulder, standTallElbow, standTallWrist1, standTallWrist2, standTallGripper };
  int down[5] = { fullDownShoulder,  fullDownElbow,  fullDownWrist1,  fullDownWrist2,  fullDownGripper };

  addAllJointsStep(tall, 700);
  for (int i = 0; i < 3; i++) {
    addAllJointsStep(down, 1600); // controlled descent, not a flop
    addAllJointsStep(down, 500);
    addAllJointsStep(tall, 1600);
    addAllJointsStep(tall, 500);
  }
  beginStepQueueShow();
}

void triggerNextDemo() {
  if (mode == JOG) return;
  if (!allJogged()) {
    Serial.println(F("!! Can't run demos yet -- jog the Gripper first (key 5), its limits are still a placeholder."));
    return;
  }
  switch (demoIndex) {
    case 0: Serial.println(F(">> DEMO: Calibration Sweep"));         startCalibSweepShow(); break;
    case 1: Serial.println(F(">> DEMO: Wave Cascade"));              startWaveShow();       break;
    case 2: Serial.println(F(">> DEMO: Synchronized Breathe"));      startBreatheShow();    break;
    case 3: Serial.println(F(">> DEMO: Stand Tall <-> Full Down"));  startPoseCycleShow();  break;
  }
  demoIndex = (demoIndex + 1) % NUM_DEMOS;
}

void stopDemo() {
  mode = RUN;
  targetPosShoulder = currentPosShoulder; targetPosElbow = currentPosElbow;
  targetPosWrist1 = currentPosWrist1;     targetPosWrist2 = currentPosWrist2;
  targetPosGripper = currentPosGripper;
  Serial.println(F(">> Demo stopped, holding position."));
}

// ============================================================================
// Core joint I/O (unchanged from v2)
// ============================================================================
int degToPulse(int deg) { return map(deg, 0, 180, SERVOMIN, SERVOMAX); }

void writeShoulder(int delta) {
  delta = constrain(delta, -90, 90);
  int lPos = constrain(L_CENTER + delta, 0, 180);
  int rPos = constrain(R_CENTER - delta, 0, 180);
  pca.setPWM(CH_SHOULDER_L, 0, degToPulse(lPos));
  pca.setPWM(CH_SHOULDER_R, 0, degToPulse(rPos));
}
void writeElbow(int deg)   { pca.setPWM(CH_ELBOW,   0, degToPulse(constrain(deg, 0, 180))); }
void writeWrist1(int deg)  { pca.setPWM(CH_WRIST1,  0, degToPulse(constrain(deg, 0, 180))); }
void writeWrist2(int deg)  { pca.setPWM(CH_WRIST2,  0, degToPulse(constrain(deg, 0, 180))); }
void writeGripper(int deg) { pca.setPWM(CH_GRIPPER, 0, degToPulse(constrain(deg, 0, 180))); }

bool parsePacket(String line, int &m, float &dPitch, float &dRoll) {
  int i1 = line.indexOf(',');
  if (i1 <= 0) return false;
  int i2 = line.indexOf(',', i1 + 1);
  if (i2 <= 0) return false;

  m = line.substring(0, i1).toInt();
  dPitch = line.substring(i1 + 1, i2).toFloat();
  dRoll  = line.substring(i2 + 1).toFloat();
  return true;
}

int applyVelocity(float delta, int currentTarget, int jointMin, int jointMax, bool invert) {
  float ad = fabs(delta);
  if (ad < DEADZONE_DEG) return currentTarget;

  int dir = (delta < 0) ? -1 : 1;
  if (invert) dir = -dir;

  int speed;
  if (ad < ZONE_SLOW_MAX) speed = SPEED_SLOW;
  else if (ad < ZONE_MED_MAX) speed = SPEED_MED;
  else speed = SPEED_FAST;

  return constrain(currentTarget + dir * speed, jointMin, jointMax);
}

void printStatus() {
  Serial.println(F("---- STATUS ----"));
  Serial.print(F("Mode (last received): ")); Serial.println(lastMode);
  Serial.print(F("Next demo index: ")); Serial.println(demoIndex);
  Serial.print(F("Shoulder: min=")); Serial.print(jointMinShoulder);
  Serial.print(F(" max=")); Serial.print(jointMaxShoulder);
  Serial.print(F(" jogHasRun=")); Serial.println(jogHasRunShoulder ? F("true") : F("false (PLACEHOLDER)"));
  Serial.print(F("Elbow:    min=")); Serial.print(jointMinElbow);
  Serial.print(F(" max=")); Serial.print(jointMaxElbow);
  Serial.print(F(" jogHasRun=")); Serial.println(jogHasRunElbow ? F("true") : F("false (PLACEHOLDER)"));
  Serial.print(F("Wrist1:   min=")); Serial.print(jointMinWrist1);
  Serial.print(F(" max=")); Serial.print(jointMaxWrist1);
  Serial.print(F(" jogHasRun=")); Serial.println(jogHasRunWrist1 ? F("true") : F("false (PLACEHOLDER)"));
  Serial.print(F("Wrist2:   min=")); Serial.print(jointMinWrist2);
  Serial.print(F(" max=")); Serial.print(jointMaxWrist2);
  Serial.print(F(" jogHasRun=")); Serial.println(jogHasRunWrist2 ? F("true") : F("false (PLACEHOLDER)"));
  Serial.print(F("Gripper:  min=")); Serial.print(jointMinGripper);
  Serial.print(F(" max=")); Serial.print(jointMaxGripper);
  Serial.print(F(" jogHasRun=")); Serial.println(jogHasRunGripper ? F("true") : F("false (PLACEHOLDER)"));
  Serial.println(F("----------------"));
}

void printJogHelp() {
  Serial.print(F(">> JOG MODE ("));
  switch (jogTarget) {
    case JOG_SHOULDER: Serial.print(F("Shoulder")); break;
    case JOG_ELBOW:     Serial.print(F("Elbow")); break;
    case JOG_WRIST1:    Serial.print(F("Wrist1")); break;
    case JOG_WRIST2:    Serial.print(F("Wrist2")); break;
    case JOG_GRIPPER:   Serial.print(F("Gripper")); break;
    default: break;
  }
  Serial.println(F("). '+'/'-' = nudge 1, '>'/'<' = nudge 5, 'n' = mark MIN, 'x' = mark MAX, 'q' = exit"));
}

void enterJog(JogTarget t, int startPos) {
  mode = JOG;
  jogTarget = t;
  jogPos = startPos;
  printJogHelp();
}

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  delay(500);

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);

  writeShoulder(currentPosShoulder);
  writeElbow(currentPosElbow);
  writeWrist1(currentPosWrist1);
  writeWrist2(currentPosWrist2);
  writeGripper(currentPosGripper);
  delay(300);

  Serial.println(F("=== Trigger-Cycle Arm Control v4 ==="));
  Serial.println(F("Shoulder/Elbow/Wrist1/Wrist2 limits pre-loaded from bench testing."));
  Serial.println(F("Jog: 1/2/3/4/5 = Shoulder/Elbow/Wrist1/Wrist2/Gripper, ?=status"));
  Serial.println(F("Demo: g=next demo, h=stop demo/hold, t=capture Stand Tall pose, d=capture Full Down pose"));
  Serial.println(F("!! Jog the Gripper (key 5) before running demos -- its limits are placeholders until then."));
  printStatus();
}

void loop() {
  // ---- Serial command handling ----
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'h' && mode != JOG) {
      stopDemo();
    }
    else if (mode == RUN) {
      if (cmd == '1') enterJog(JOG_SHOULDER, currentPosShoulder);
      else if (cmd == '2') enterJog(JOG_ELBOW, currentPosElbow);
      else if (cmd == '3') enterJog(JOG_WRIST1, currentPosWrist1);
      else if (cmd == '4') enterJog(JOG_WRIST2, currentPosWrist2);
      else if (cmd == '5') enterJog(JOG_GRIPPER, currentPosGripper);
      else if (cmd == 'g') triggerNextDemo();
      else if (cmd == 't') {
        standTallShoulder = currentPosShoulder; standTallElbow = currentPosElbow;
        standTallWrist1 = currentPosWrist1;     standTallWrist2 = currentPosWrist2;
        standTallGripper = currentPosGripper;
        Serial.print(F(">> Captured STAND TALL: "));
        Serial.print(standTallShoulder); Serial.print(',');
        Serial.print(standTallElbow); Serial.print(',');
        Serial.print(standTallWrist1); Serial.print(',');
        Serial.print(standTallWrist2); Serial.print(',');
        Serial.println(standTallGripper);
      }
      else if (cmd == 'd') {
        fullDownShoulder = currentPosShoulder; fullDownElbow = currentPosElbow;
        fullDownWrist1 = currentPosWrist1;     fullDownWrist2 = currentPosWrist2;
        fullDownGripper = currentPosGripper;
        Serial.print(F(">> Captured FULL DOWN: "));
        Serial.print(fullDownShoulder); Serial.print(',');
        Serial.print(fullDownElbow); Serial.print(',');
        Serial.print(fullDownWrist1); Serial.print(',');
        Serial.print(fullDownWrist2); Serial.print(',');
        Serial.println(fullDownGripper);
      }
      else if (cmd == '?') printStatus();
    }
    else if (mode == JOG) {
      int *jMin, *jMax;
      switch (jogTarget) {
        case JOG_SHOULDER: jMin = &jointMinShoulder; jMax = &jointMaxShoulder; break;
        case JOG_ELBOW:     jMin = &jointMinElbow;     jMax = &jointMaxElbow;     break;
        case JOG_WRIST1:    jMin = &jointMinWrist1;    jMax = &jointMaxWrist1;    break;
        case JOG_WRIST2:    jMin = &jointMinWrist2;    jMax = &jointMaxWrist2;    break;
        default:            jMin = &jointMinGripper;   jMax = &jointMaxGripper;   break;
      }

      if (cmd == '+') jogPos++;
      else if (cmd == '-') jogPos--;
      else if (cmd == '>') jogPos += 5;
      else if (cmd == '<') jogPos -= 5;
      else if (cmd == 'n') { *jMin = jogPos; Serial.print(F(">> MIN set to ")); Serial.println(*jMin); }
      else if (cmd == 'x') { *jMax = jogPos; Serial.print(F(">> MAX set to ")); Serial.println(*jMax); }
      else if (cmd == 'q') {
        if (*jMin > *jMax) { int t = *jMin; *jMin = *jMax; *jMax = t; }
        switch (jogTarget) {
          case JOG_SHOULDER: currentPosShoulder = jogPos; targetPosShoulder = jogPos; jogHasRunShoulder = true; break;
          case JOG_ELBOW:     currentPosElbow = jogPos;     targetPosElbow = jogPos;     jogHasRunElbow = true;     break;
          case JOG_WRIST1:    currentPosWrist1 = jogPos;    targetPosWrist1 = jogPos;    jogHasRunWrist1 = true;    break;
          case JOG_WRIST2:    currentPosWrist2 = jogPos;    targetPosWrist2 = jogPos;    jogHasRunWrist2 = true;    break;
          default:            currentPosGripper = jogPos;   targetPosGripper = jogPos;   jogHasRunGripper = true;   break;
        }
        Serial.print(F(">> JOG DONE. MIN=")); Serial.print(*jMin);
        Serial.print(F(" MAX=")); Serial.println(*jMax);
        mode = RUN;
        jogTarget = JOG_NONE;
      }
      else if (cmd == '?') printStatus();

      if (cmd == '+' || cmd == '-' || cmd == '>' || cmd == '<') {
        switch (jogTarget) {
          case JOG_SHOULDER: writeShoulder(jogPos); break;
          case JOG_ELBOW:     writeElbow(jogPos);     break;
          case JOG_WRIST1:    writeWrist1(jogPos);    break;
          case JOG_WRIST2:    writeWrist2(jogPos);    break;
          default:            writeGripper(jogPos);   break;
        }
        Serial.print(F("pos=")); Serial.println(jogPos);
      }
    }
  }

  // ---- Read Bluetooth packets ----
  if (BT.available()) {
    String line = BT.readStringUntil('\n');
    line.trim();

    // Demos/choreo are serial-monitor only now (see 'g' below) -- the glove
    // only ever sends gesture packets (mode,deltaPitch,deltaRoll), so we just
    // parse and apply those directly. No CMD:NEXT_DEMO path anymore.
    int pMode; float dPitch, dRoll;
    if (parsePacket(line, pMode, dPitch, dRoll)) {
      lastPacketTime = millis();
      lastMode = pMode;
      lastDeltaPitch = dPitch;
      lastDeltaRoll = dRoll;

      if (pMode >= 1 && pMode <= 4 && mode == SHOW) {
        // A genuine gesture command still interrupts any running demo and
        // gives control back to your hand immediately.
        mode = RUN;
      }

      if (mode == RUN) {
        if (pMode == 1) {
          targetPosShoulder = applyVelocity(dPitch, targetPosShoulder, jointMinShoulder, jointMaxShoulder, INVERT_SHOULDER);
        } else if (pMode == 2) {
          targetPosElbow  = applyVelocity(dPitch, targetPosElbow,  jointMinElbow,  jointMaxElbow,  INVERT_ELBOW);
        } else if (pMode == 3) {
          targetPosWrist1 = applyVelocity(dPitch, targetPosWrist1, jointMinWrist1, jointMaxWrist1, INVERT_WRIST1);
          targetPosWrist2 = applyVelocity(dRoll,  targetPosWrist2, jointMinWrist2, jointMaxWrist2, INVERT_WRIST2);
        } else if (pMode == 4) {
          targetPosGripper = applyVelocity(dRoll, targetPosGripper, jointMinGripper, jointMaxGripper, INVERT_GRIPPER);
        }
      }
    }
  }

  // ---- JOG mode: skip everything else ----
  if (mode == JOG) {
    if (millis() - lastPrint > 300) {
      Serial.print(F("jog pos=")); Serial.println(jogPos);
      lastPrint = millis();
    }
    return;
  }

  // ---- SHOW mode: run the sequencer, skip normal RUN easing ----
  if (mode == SHOW) {
    if (usingWaveEngine) updateWaveShow(); else updateStepQueueShow();
    if (millis() - lastPrint > 500) {
      Serial.print(F("[SHOW] Sh=")); Serial.print(currentPosShoulder);
      Serial.print(F(" El=")); Serial.print(currentPosElbow);
      Serial.print(F(" W1=")); Serial.print(currentPosWrist1);
      Serial.print(F(" W2=")); Serial.print(currentPosWrist2);
      Serial.print(F(" Gr=")); Serial.println(currentPosGripper);
      lastPrint = millis();
    }
    return;
  }

  // ---- RUN mode: ease all five joints toward target, 1 unit per loop each ----
  bool linkStale = (millis() - lastPacketTime > STALE_MS) && (lastPacketTime != 0);
  bool moved = false;
  if (!linkStale) {
    if (currentPosShoulder != targetPosShoulder) { currentPosShoulder += (currentPosShoulder < targetPosShoulder) ? 1 : -1; moved = true; }
    if (currentPosElbow    != targetPosElbow)    { currentPosElbow    += (currentPosElbow    < targetPosElbow)    ? 1 : -1; moved = true; }
    if (currentPosWrist1   != targetPosWrist1)   { currentPosWrist1   += (currentPosWrist1   < targetPosWrist1)   ? 1 : -1; moved = true; }
    if (currentPosWrist2   != targetPosWrist2)   { currentPosWrist2   += (currentPosWrist2   < targetPosWrist2)   ? 1 : -1; moved = true; }
    if (currentPosGripper  != targetPosGripper)  { currentPosGripper  += (currentPosGripper  < targetPosGripper)  ? 1 : -1; moved = true; }
  }
  if (moved) {
    writeShoulder(currentPosShoulder);
    writeElbow(currentPosElbow);
    writeWrist1(currentPosWrist1);
    writeWrist2(currentPosWrist2);
    writeGripper(currentPosGripper);
    delay(SLEW_DELAY_MS);
  }

  if (millis() - lastPrint > 500) {
    if (!allJogged()) {
      Serial.println(F("!! WARNING: Gripper hasn't been jogged this session -- its limits are still a placeholder."));
    }
    Serial.print(F("mode=")); Serial.print(lastMode);
    Serial.print(F(" dPitch=")); Serial.print(lastDeltaPitch, 1);
    Serial.print(F(" dRoll=")); Serial.print(lastDeltaRoll, 1);
    Serial.print(F("  Sh=")); Serial.print(currentPosShoulder);
    Serial.print(F(" El=")); Serial.print(currentPosElbow);
    Serial.print(F(" W1=")); Serial.print(currentPosWrist1);
    Serial.print(F(" W2=")); Serial.print(currentPosWrist2);
    Serial.print(F(" Gr=")); Serial.print(currentPosGripper);
    Serial.println(linkStale ? F("  [LINK STALE]") : F(""));
    lastPrint = millis();
  }
}
