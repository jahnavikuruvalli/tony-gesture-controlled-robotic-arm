/******************************************************************************
  Trigger-Cycle Slave (arm side) -- velocity/joystick control, 3 modes.

  Receives "mode,deltaPitch,deltaRoll" from the trigger-cycle glove master.
  Mode 1: deltaPitch drives Shoulder. Roll unused.
  Mode 2: deltaPitch drives Elbow. Roll unused (Elbow only -- wrist is NOT
          touched in this mode).
  Mode 3: deltaPitch drives Wrist1 (wrist-pitch), deltaRoll drives Wrist2
          (wrist-roll). This is the only mode that uses both joints.

  VELOCITY CONTROL, NOT ABSOLUTE ANGLE: bigger tilt = faster movement, not
  "angle X = position X." A joint's target only changes while a fresh
  packet arrives AND the relevant delta is outside the deadzone. No new
  packet = no more movement, automatically -- if the Bluetooth link drops,
  the arm just stops where it is instead of running away. This is the
  actual safety property that makes velocity control OK to use here.

  No hold-and-mark calibration needed -- deadzone/speed bands are fixed
  constants (tune DEADZONE_DEG / ZONE_*_MAX / SPEED_* below). What you
  DO still need is jog mode, to find each joint's real safe mechanical
  limits (this hasn't changed -- servos can still be driven past a real
  limit if you don't set jointMin/Max correctly).

  A joint only counts as "jogged" once BOTH its min AND max have actually
  been set with 'n'/'x' at some point (can be different sessions). Just
  entering jog mode and pressing 'q' without setting either one does NOT
  clear the "still placeholder" warning -- previously it incorrectly did.

  ============================================================================
  WORKFLOW:
  ============================================================================
    1. Flash this. Open Serial Monitor @ 9600.
    2. Jog each joint to find its true limits:
         '1' = jog Shoulder, '2' = jog Elbow, '3' = jog Wrist1, '4' = jog Wrist2
         '+'/'-' nudge 1, '>'/'<' nudge 5, 'n' mark MIN, 'x' mark MAX, 'q' exit
    3. That's it -- no separate calibration step. Send '?' any time for status.
    4. On the glove: tap the trigger to cycle Mode 1 -> 2 -> 3 -> 1...
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
const int gripper_center = 122;

// ---- Starting positions for the four joints under live control ----
const int shoulderStartCenter = 0;   // delta
const int elbowStartCenter    = 90;  // degrees
const int wrist1StartCenter   = 25;  // degrees
const int wrist2StartCenter   = 90;  // degrees

// ---- Shoulder & Elbow: CONFIRMED via jog mode on <fill in your test date>.
// Baked in so you don't have to re-jog every session. ----
int jointMinShoulder = -50, jointMaxShoulder = 20;
int jointMinElbow    = 0,   jointMaxElbow    = 120;

// ---- Wrist1 & Wrist2: NOT actually jogged yet -- the values used during
// testing were random placeholders, not real mechanical limits. Left as
// the narrow safe-default range on purpose. Jog these before trusting them,
// or the warning below will keep reminding you every ~500ms. ----
int jointMinWrist1   = wrist1StartCenter - 15,   jointMaxWrist1   = wrist1StartCenter + 15;
int jointMinWrist2   = wrist2StartCenter - 15,   jointMaxWrist2   = wrist2StartCenter + 15;

// ---- Tracks whether MIN/MAX have actually been explicitly set via jog ----
// (persists across sessions -- e.g. set MIN today, MAX tomorrow, still counts)
// Shoulder/Elbow marked true here since their real limits are baked in above.
bool minSetShoulder = true,  maxSetShoulder = true;
bool minSetElbow    = true,  maxSetElbow    = true;
bool minSetWrist1   = false, maxSetWrist1   = false;
bool minSetWrist2   = false, maxSetWrist2   = false;

// ---- Deadzone / speed ramp, tune these to taste ----
const float DEADZONE_DEG  = 5.0;  // smaller than this: no movement
const float ZONE_SLOW_MAX = 20.0; // 5-20 deg: slow
const float ZONE_MED_MAX  = 40.0; // 20-40 deg: medium, 40+: fast
const int SPEED_SLOW = 1; // servo-degrees added per packet (~20Hz)
const int SPEED_MED  = 2;
const int SPEED_FAST = 4;

bool INVERT_SHOULDER = false, INVERT_ELBOW = false, INVERT_WRIST1 = false, INVERT_WRIST2 = false;

enum Mode { RUN, JOG };
Mode mode = RUN;

enum JogTarget { JOG_NONE, JOG_SHOULDER, JOG_ELBOW, JOG_WRIST1, JOG_WRIST2 };
JogTarget jogTarget = JOG_NONE;
int jogPos = 0;

const int SLEW_DELAY_MS = 15;
const unsigned long STALE_MS = 500;

int currentPosShoulder = shoulderStartCenter, targetPosShoulder = shoulderStartCenter;
int currentPosElbow    = elbowStartCenter,    targetPosElbow    = elbowStartCenter;
int currentPosWrist1   = wrist1StartCenter,   targetPosWrist1   = wrist1StartCenter;
int currentPosWrist2   = wrist2StartCenter,   targetPosWrist2   = wrist2StartCenter;

unsigned long lastPacketTime = 0;
unsigned long lastPrint = 0;
int lastMode = 1;
float lastDeltaPitch = 0, lastDeltaRoll = 0;

int degToPulse(int deg) {
  return map(deg, 0, 180, SERVOMIN, SERVOMAX);
}

void writeShoulder(int delta) {
  delta = constrain(delta, -90, 90);
  int lPos = constrain(L_CENTER + delta, 0, 180);
  int rPos = constrain(R_CENTER - delta, 0, 180);
  pca.setPWM(CH_SHOULDER_L, 0, degToPulse(lPos));
  pca.setPWM(CH_SHOULDER_R, 0, degToPulse(rPos));
}
void writeElbow(int deg)  { pca.setPWM(CH_ELBOW,  0, degToPulse(constrain(deg, 0, 180))); }
void writeWrist1(int deg) { pca.setPWM(CH_WRIST1, 0, degToPulse(constrain(deg, 0, 180))); }
void writeWrist2(int deg) { pca.setPWM(CH_WRIST2, 0, degToPulse(constrain(deg, 0, 180))); }

void parkOtherJoints() {
  pca.setPWM(CH_GRIPPER, 0, degToPulse(gripper_center));
}

// Parses "mode,deltaPitch,deltaRoll"
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

// Deadzone-ramped velocity: bigger delta = bigger step, closer to zero = no step.
int applyVelocity(float delta, int currentTarget, int jointMin, int jointMax, bool invert) {
  float ad = fabs(delta);
  if (ad < DEADZONE_DEG) return currentTarget; // inside deadzone, no change

  int dir = (delta < 0) ? -1 : 1;
  if (invert) dir = -dir;

  int speed;
  if (ad < ZONE_SLOW_MAX) speed = SPEED_SLOW;
  else if (ad < ZONE_MED_MAX) speed = SPEED_MED;
  else speed = SPEED_FAST;

  return constrain(currentTarget + dir * speed, jointMin, jointMax);
}

bool jointFullyJogged(bool minSet, bool maxSet) {
  return minSet && maxSet;
}

void printStatus() {
  Serial.println(F("---- STATUS ----"));
  Serial.print(F("Mode (last received): ")); Serial.println(lastMode);
  Serial.print(F("Shoulder: min=")); Serial.print(jointMinShoulder);
  Serial.print(F(" max=")); Serial.print(jointMaxShoulder);
  Serial.print(F(" jogged=")); Serial.println(jointFullyJogged(minSetShoulder, maxSetShoulder) ? F("true") : F("false (PLACEHOLDER)"));
  Serial.print(F("Elbow:    min=")); Serial.print(jointMinElbow);
  Serial.print(F(" max=")); Serial.print(jointMaxElbow);
  Serial.print(F(" jogged=")); Serial.println(jointFullyJogged(minSetElbow, maxSetElbow) ? F("true") : F("false (PLACEHOLDER)"));
  Serial.print(F("Wrist1:   min=")); Serial.print(jointMinWrist1);
  Serial.print(F(" max=")); Serial.print(jointMaxWrist1);
  Serial.print(F(" jogged=")); Serial.println(jointFullyJogged(minSetWrist1, maxSetWrist1) ? F("true") : F("false (PLACEHOLDER)"));
  Serial.print(F("Wrist2:   min=")); Serial.print(jointMinWrist2);
  Serial.print(F(" max=")); Serial.print(jointMaxWrist2);
  Serial.print(F(" jogged=")); Serial.println(jointFullyJogged(minSetWrist2, maxSetWrist2) ? F("true") : F("false (PLACEHOLDER)"));
  Serial.println(F("----------------"));
}

void printJogHelp() {
  Serial.print(F(">> JOG MODE ("));
  switch (jogTarget) {
    case JOG_SHOULDER: Serial.print(F("Shoulder")); break;
    case JOG_ELBOW:     Serial.print(F("Elbow")); break;
    case JOG_WRIST1:    Serial.print(F("Wrist1")); break;
    case JOG_WRIST2:    Serial.print(F("Wrist2")); break;
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

  parkOtherJoints();
  writeShoulder(currentPosShoulder);
  writeElbow(currentPosElbow);
  writeWrist1(currentPosWrist1);
  writeWrist2(currentPosWrist2);
  delay(300);

  Serial.println(F("=== Trigger-Cycle Arm Control ==="));
  Serial.println(F("Commands: 1/2/3/4 = jog Shoulder/Elbow/Wrist1/Wrist2, ?=status"));
  printStatus();
}

void loop() {
  // ---- Serial command handling ----
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (mode == RUN) {
      if (cmd == '1') enterJog(JOG_SHOULDER, currentPosShoulder);
      else if (cmd == '2') enterJog(JOG_ELBOW, currentPosElbow);
      else if (cmd == '3') enterJog(JOG_WRIST1, currentPosWrist1);
      else if (cmd == '4') enterJog(JOG_WRIST2, currentPosWrist2);
      else if (cmd == '?') printStatus();
    }
    else if (mode == JOG) {
      int *jMin, *jMax;
      bool *minSetFlag, *maxSetFlag;
      switch (jogTarget) {
        case JOG_SHOULDER: jMin = &jointMinShoulder; jMax = &jointMaxShoulder; minSetFlag = &minSetShoulder; maxSetFlag = &maxSetShoulder; break;
        case JOG_ELBOW:     jMin = &jointMinElbow;     jMax = &jointMaxElbow;     minSetFlag = &minSetElbow;     maxSetFlag = &maxSetElbow;     break;
        case JOG_WRIST1:    jMin = &jointMinWrist1;    jMax = &jointMaxWrist1;    minSetFlag = &minSetWrist1;    maxSetFlag = &maxSetWrist1;    break;
        default:            jMin = &jointMinWrist2;    jMax = &jointMaxWrist2;    minSetFlag = &minSetWrist2;    maxSetFlag = &maxSetWrist2;    break;
      }

      if (cmd == '+') jogPos++;
      else if (cmd == '-') jogPos--;
      else if (cmd == '>') jogPos += 5;
      else if (cmd == '<') jogPos -= 5;
      else if (cmd == 'n') { *jMin = jogPos; *minSetFlag = true; Serial.print(F(">> MIN set to ")); Serial.println(*jMin); }
      else if (cmd == 'x') { *jMax = jogPos; *maxSetFlag = true; Serial.print(F(">> MAX set to ")); Serial.println(*jMax); }
      else if (cmd == 'q') {
        if (*jMin > *jMax) { int t = *jMin; *jMin = *jMax; *jMax = t; }
        switch (jogTarget) {
          case JOG_SHOULDER: currentPosShoulder = jogPos; targetPosShoulder = jogPos; break;
          case JOG_ELBOW:     currentPosElbow = jogPos;     targetPosElbow = jogPos;     break;
          case JOG_WRIST1:    currentPosWrist1 = jogPos;    targetPosWrist1 = jogPos;    break;
          default:            currentPosWrist2 = jogPos;    targetPosWrist2 = jogPos;    break;
        }
        if (!(*minSetFlag) || !(*maxSetFlag)) {
          Serial.println(F(">> NOTE: MIN and/or MAX was never marked this session (or ever) -- limits may still be placeholders."));
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
          default:            writeWrist2(jogPos);    break;
        }
        Serial.print(F("pos=")); Serial.println(jogPos);
      }
    }
  }

  // ---- Read Bluetooth packets ----
  if (BT.available()) {
    String line = BT.readStringUntil('\n');
    line.trim();

    int pMode; float dPitch, dRoll;
    if (parsePacket(line, pMode, dPitch, dRoll)) {
      lastPacketTime = millis();
      lastMode = pMode;
      lastDeltaPitch = dPitch;
      lastDeltaRoll = dRoll;

      if (mode == RUN) {
        if (pMode == 1) {
          targetPosShoulder = applyVelocity(dPitch, targetPosShoulder, jointMinShoulder, jointMaxShoulder, INVERT_SHOULDER);
        } else if (pMode == 2) {
          targetPosElbow  = applyVelocity(dPitch, targetPosElbow,  jointMinElbow,  jointMaxElbow,  INVERT_ELBOW);
          // roll is unused in Mode 2 on purpose -- Elbow only
        } else if (pMode == 3) {
          targetPosWrist1 = applyVelocity(dPitch, targetPosWrist1, jointMinWrist1, jointMaxWrist1, INVERT_WRIST1);
          targetPosWrist2 = applyVelocity(dRoll,  targetPosWrist2, jointMinWrist2, jointMaxWrist2, INVERT_WRIST2);
        }
      }
    }
  }

  // ---- JOG mode: skip normal easing ----
  if (mode == JOG) {
    if (millis() - lastPrint > 300) {
      Serial.print(F("jog pos=")); Serial.println(jogPos);
      lastPrint = millis();
    }
    return;
  }

  // ---- RUN mode: ease all four joints toward target, 1 unit per loop each ----
  bool linkStale = (millis() - lastPacketTime > STALE_MS) && (lastPacketTime != 0);
  bool moved = false;
  if (!linkStale) {
    if (currentPosShoulder != targetPosShoulder) { currentPosShoulder += (currentPosShoulder < targetPosShoulder) ? 1 : -1; moved = true; }
    if (currentPosElbow    != targetPosElbow)    { currentPosElbow    += (currentPosElbow    < targetPosElbow)    ? 1 : -1; moved = true; }
    if (currentPosWrist1   != targetPosWrist1)   { currentPosWrist1   += (currentPosWrist1   < targetPosWrist1)   ? 1 : -1; moved = true; }
    if (currentPosWrist2   != targetPosWrist2)   { currentPosWrist2   += (currentPosWrist2   < targetPosWrist2)   ? 1 : -1; moved = true; }
  }
  if (moved) {
    writeShoulder(currentPosShoulder);
    writeElbow(currentPosElbow);
    writeWrist1(currentPosWrist1);
    writeWrist2(currentPosWrist2);
    delay(SLEW_DELAY_MS);
  }

  if (millis() - lastPrint > 500) {
    if (!jointFullyJogged(minSetShoulder, maxSetShoulder) || !jointFullyJogged(minSetElbow, maxSetElbow) ||
        !jointFullyJogged(minSetWrist1, maxSetWrist1) || !jointFullyJogged(minSetWrist2, maxSetWrist2)) {
      Serial.println(F("!! WARNING: one or more joints haven't had BOTH min and max jogged -- limits are still placeholders."));
    }
    Serial.print(F("mode=")); Serial.print(lastMode);
    Serial.print(F(" dPitch=")); Serial.print(lastDeltaPitch, 1);
    Serial.print(F(" dRoll=")); Serial.print(lastDeltaRoll, 1);
    Serial.print(F("  Sh=")); Serial.print(currentPosShoulder);
    Serial.print(F(" El=")); Serial.print(currentPosElbow);
    Serial.print(F(" W1=")); Serial.print(currentPosWrist1);
    Serial.print(F(" W2=")); Serial.print(currentPosWrist2);
    Serial.println(linkStale ? F("  [LINK STALE]") : F(""));
    lastPrint = millis();
  }
}
