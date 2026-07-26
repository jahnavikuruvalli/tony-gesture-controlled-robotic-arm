/******************************************************************************
  Single-Potentiometer MULTI-Joint Control (PCA9685) -- v3
  Now with POT CALIBRATION to fix "never quite reaches max" precision issues,
  and corrected gripper range.

  Wiring:
    Pot outer leg 1 -> Arduino 5V
    Pot outer leg 2 -> Arduino GND
    Pot wiper (middle) -> Arduino A0

  FIRST-TIME SETUP -- calibrate the pot before normal use:
    1. Send 'c' to start calibration.
    2. Slowly turn the pot through its ENTIRE physical range, end to end,
       a couple of times (takes ~5 seconds -- code is recording the real
       min/max raw values it sees).
    3. Send 'c' again to stop and lock in calibration.
    This fixes pots that never quite reach a true 0 or 1023 raw value,
    which was causing wrist1 to cap at "49" instead of its real max.

  SAFETY FEATURES (kept from earlier sketches):
  - Each selected joint eases 1 degree per loop cycle toward the pot's
    mapped target -- no instant jumps.
  - CAUTION: moving multiple servos at once multiplies current draw.
    Select 1-2 joints at a time until power supply is confirmed stable.
  - Selecting a new joint latches its target to its current position first,
    avoiding a jump until the pot sweeps back through that spot.

  Channel assignment:
    Channel 0 = Shoulder L (MG995)      Channel 1 = Shoulder R (DS3218)
    Channel 2 = Elbow                    Channel 3 = Wrist 1
    Channel 4 = Wrist 2                  Channel 5 = Gripper

  Serial commands:
    1-5 = toggle Shoulder/Elbow/Wrist1/Wrist2/Gripper
    a = select all   n = select none   ? = status
    c = start/stop pot calibration

  UPDATED LIMITS:
    Gripper corrected to 65-180 degrees (was mistakenly 0-65).
    Wrist1 still 0-50 -- TELL ME the real max once you've found it with
    a calibrated pot (or the earlier 5-degree serial step test), and I'll
    update WRIST1_MAX to match.
******************************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 125
#define SERVOMAX 575
#define SERVO_FREQ 50

const int POT_PIN = A0;

// ---- Channel assignment ----
const int CH_SHOULDER_L = 0;
const int CH_SHOULDER_R = 1;
const int CH_ELBOW      = 2;
const int CH_WRIST1     = 3;
const int CH_WRIST2     = 4;
const int CH_GRIPPER    = 5;

// ---- Shoulder calibration ----
const int L_CENTER = 85;
const int R_CENTER = 160;
int SHOULDER_DELTA_MIN = -20;
int SHOULDER_DELTA_MAX = 95;

// ---- Other joint centers ----
const int elbow_center    = 90;
const int wrist1_center   = 25;
const int wrist2_center   = 90;
const int gripper_center  = 122; // midpoint of corrected 65-180 range

// ---- Other joint limits ----
const int ELBOW_MIN    = 0,   ELBOW_MAX    = 180; // not yet tested
const int WRIST1_MIN   = 0,   WRIST1_MAX   = 50;  // tell me the real max once confirmed
const int WRIST2_MIN   = 0,   WRIST2_MAX   = 180; // not yet tested
const int GRIPPER_MIN  = 65,  GRIPPER_MAX  = 180; // CORRECTED

const int SLEW_DELAY_MS = 15;

// ---- Pot calibration ----
int potRawMin = 0;
int potRawMax = 1023;
bool calibrating = false;

// ---- Joint indices ----
enum Joint { SHOULDER = 0, ELBOW, WRIST1, WRIST2, GRIPPER, NUM_JOINTS };
const char* jointNames[NUM_JOINTS] = {"Shoulder", "Elbow", "Wrist1", "Wrist2", "Gripper"};

bool selected[NUM_JOINTS] = {false, true, false, false, false};

int currentPos[NUM_JOINTS] = {0, elbow_center, wrist1_center, wrist2_center, gripper_center};
int targetPos[NUM_JOINTS]  = {0, elbow_center, wrist1_center, wrist2_center, gripper_center};

unsigned long lastPrint = 0;

int degToPulse(int deg) {
  return map(deg, 0, 180, SERVOMIN, SERVOMAX);
}

void writeShoulder(int delta) {
  int lPos = constrain(L_CENTER + delta, 0, 180);
  int rPos = constrain(R_CENTER - delta, 0, 180);
  pca.setPWM(CH_SHOULDER_L, 0, degToPulse(lPos));
  pca.setPWM(CH_SHOULDER_R, 0, degToPulse(rPos));
}

void writeJoint(int joint, int pos) {
  switch (joint) {
    case SHOULDER: writeShoulder(pos); break;
    case ELBOW:    pca.setPWM(CH_ELBOW,   0, degToPulse(pos)); break;
    case WRIST1:   pca.setPWM(CH_WRIST1,  0, degToPulse(pos)); break;
    case WRIST2:   pca.setPWM(CH_WRIST2,  0, degToPulse(pos)); break;
    case GRIPPER:  pca.setPWM(CH_GRIPPER, 0, degToPulse(pos)); break;
  }
}

void getJointRange(int joint, int &lo, int &hi) {
  switch (joint) {
    case SHOULDER: lo = SHOULDER_DELTA_MIN; hi = SHOULDER_DELTA_MAX; break;
    case ELBOW:    lo = ELBOW_MIN;   hi = ELBOW_MAX;   break;
    case WRIST1:   lo = WRIST1_MIN;  hi = WRIST1_MAX;  break;
    case WRIST2:   lo = WRIST2_MIN;  hi = WRIST2_MAX;  break;
    case GRIPPER:  lo = GRIPPER_MIN; hi = GRIPPER_MAX; break;
  }
}

void toggleJoint(int joint) {
  selected[joint] = !selected[joint];
  if (selected[joint]) {
    targetPos[joint] = currentPos[joint];
  }
  Serial.print(">> ");
  Serial.print(jointNames[joint]);
  Serial.println(selected[joint] ? " added to selection" : " removed from selection");
}

void setup() {
  Serial.begin(9600);
  delay(500);

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);

  for (int j = 0; j < NUM_JOINTS; j++) {
    writeJoint(j, currentPos[j]);
    delay(300);
  }

  printMenu();
  printStatus();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1': toggleJoint(SHOULDER); break;
      case '2': toggleJoint(ELBOW);    break;
      case '3': toggleJoint(WRIST1);   break;
      case '4': toggleJoint(WRIST2);   break;
      case '5': toggleJoint(GRIPPER);  break;

      case 'a':
        for (int j = 0; j < NUM_JOINTS; j++) {
          selected[j] = true;
          targetPos[j] = currentPos[j];
        }
        Serial.println(">> All joints selected");
        break;

      case 'n':
        for (int j = 0; j < NUM_JOINTS; j++) selected[j] = false;
        Serial.println(">> Selection cleared");
        break;

      case 'c':
        calibrating = !calibrating;
        if (calibrating) {
          potRawMin = 1023;
          potRawMax = 0;
          Serial.println(">> CALIBRATION STARTED -- slowly turn pot through its full range");
        } else {
          Serial.print(">> CALIBRATION LOCKED. Raw min=");
          Serial.print(potRawMin);
          Serial.print(" max=");
          Serial.println(potRawMax);
        }
        break;

      case '?': printStatus(); break;
      default: break;
    }
  }

  int rawPot = analogRead(POT_PIN);

  if (calibrating) {
    if (rawPot < potRawMin) potRawMin = rawPot;
    if (rawPot > potRawMax) potRawMax = rawPot;
    if (millis() - lastPrint > 300) {
      Serial.print("Calibrating... raw=");
      Serial.print(rawPot);
      Serial.print("  seen min=");
      Serial.print(potRawMin);
      Serial.print("  seen max=");
      Serial.println(potRawMax);
      lastPrint = millis();
    }
    return; // don't move servos while calibrating
  }

  int potValue = constrain(map(rawPot, potRawMin, potRawMax, 0, 1023), 0, 1023);
  bool anyMoved = false;

  for (int j = 0; j < NUM_JOINTS; j++) {
    if (!selected[j]) continue;

    int lo, hi;
    getJointRange(j, lo, hi);
    targetPos[j] = map(potValue, 0, 1023, lo, hi);

    if (currentPos[j] < targetPos[j]) {
      currentPos[j]++;
      writeJoint(j, currentPos[j]);
      anyMoved = true;
    } else if (currentPos[j] > targetPos[j]) {
      currentPos[j]--;
      writeJoint(j, currentPos[j]);
      anyMoved = true;
    }
  }

  if (anyMoved) delay(SLEW_DELAY_MS);

  if (millis() - lastPrint > 500) {
    printStatus();
    lastPrint = millis();
  }
}

void printStatus() {
  Serial.println("---- Selection & Positions ----");
  for (int j = 0; j < NUM_JOINTS; j++) {
    Serial.print(jointNames[j]);
    Serial.print(" [");
    Serial.print(selected[j] ? "X" : " ");
    Serial.print("]: ");
    Serial.println(currentPos[j]);
  }
  Serial.println("--------------------------------");
}

void printMenu() {
  Serial.println("=== Single-Pot Multi-Joint Control v3 ===");
  Serial.println("1-5 = toggle Shoulder/Elbow/Wrist1/Wrist2/Gripper");
  Serial.println("a = select all   n = select none   ? = status");
  Serial.println("c = start/stop pot calibration (do this first!)");
  Serial.println("CAUTION: multiple simultaneous servos = higher current draw.");
}
