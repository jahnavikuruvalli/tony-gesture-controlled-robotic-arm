/******************************************************************************
  Multi-Joint Test via PCA9685 — Calibrated Shoulder + Elbow/Wrist1/Wrist2/Gripper
  Select multiple joints, move simultaneously

  Shoulder uses the calibrated delta formula (NOT simple mirroring), since
  the two servos have different centers and opposite rotation directions:
    L (MG995)  command = L_CENTER + delta
    R (DS3218) command = R_CENTER - delta
  If you've since re-seated the DS3218 horn, update R_CENTER below to the
  new value you find with the calibration sketch.

  Requires: "Adafruit PWM Servo Driver Library" (install via Library Manager)

  PCA9685 wiring:
    VCC -> Arduino 5V      GND -> Arduino GND
    SCL -> Arduino A5      SDA -> Arduino A4
    OE  -> not connected
    V+  -> buck converter output (6.0V)
    GND (screw terminal) -> buck GND, common with Arduino GND

  Channel assignment:
    Channel 0 = Shoulder L (MG995)
    Channel 1 = Shoulder R (DS3218, opposite rotation, calibrated separately)
    Channel 2 = Elbow
    Channel 3 = Wrist 1
    Channel 4 = Wrist 2
    Channel 5 = Gripper

  CAUTION: Moving multiple servos at once multiplies current draw.
  Start with 1-2 joints selected until your buck/battery setup is
  confirmed stable under load.

  Serial commands:
    1 = toggle Shoulder     2 = toggle Elbow      3 = toggle Wrist 1
    4 = toggle Wrist 2      5 = toggle Gripper
    a = select all          n = select none
    f = +1 degree, all selected (simultaneously)
    b = -1 degree, all selected (simultaneously)
    r = reset all selected to center
    ? = print selection and positions

  PHYSICAL LIMITS:
    Wrist 1 clamped to 0-50 (known hard stop).
    Shoulder delta clamped to -20..+95 (servo command limits from L/R
    centers -- update if you re-calibrate R_CENTER).
    Elbow/Wrist2/Gripper default 0-180 -- not yet tested. Go slowly and
    update the matching MIN/MAX constants once real limits are found.
******************************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 125
#define SERVOMAX 575
#define SERVO_FREQ 50

// ---- Channel assignment ----
const int CH_SHOULDER_L = 0;
const int CH_SHOULDER_R = 1;
const int CH_ELBOW      = 2;
const int CH_WRIST1     = 3;
const int CH_WRIST2     = 4;
const int CH_GRIPPER    = 5;

// ---- Shoulder calibration ----
const int L_CENTER = 85;
const int R_CENTER = 160; // update after re-seating DS3218 horn, if done
int SHOULDER_DELTA_MIN = -20;
int SHOULDER_DELTA_MAX = 95;

// ---- Other joint centers ----
const int elbow_center    = 90;
const int wrist1_center   = 25; // mid of its safe 0-50 range
const int wrist2_center   = 90;
const int gripper_center  = 90;

// ---- Other joint limits ----
const int ELBOW_MIN    = 0,  ELBOW_MAX    = 180; // not yet tested
const int WRIST1_MIN   = 0,  WRIST1_MAX   = 50;  // known hard stop
const int WRIST2_MIN   = 0,  WRIST2_MAX   = 180; // not yet tested
const int GRIPPER_MIN  = 0,  GRIPPER_MAX  = 180; // not yet tested

// ---- Current positions ----
int shoulder_delta = 0; // 0 = center
int elbow_pos    = elbow_center;
int wrist1_pos   = wrist1_center;
int wrist2_pos   = wrist2_center;
int gripper_pos  = gripper_center;

// ---- Selection flags: 0=Shoulder 1=Elbow 2=Wrist1 3=Wrist2 4=Gripper ----
bool selected[5] = {false, false, false, false, false};
const char* jointNames[5] = {"Shoulder", "Elbow", "Wrist1", "Wrist2", "Gripper"};

int degToPulse(int deg) {
  return map(deg, 0, 180, SERVOMIN, SERVOMAX);
}

void writeShoulder(int delta) {
  int lPos = constrain(L_CENTER + delta, 0, 180);
  int rPos = constrain(R_CENTER - delta, 0, 180);
  pca.setPWM(CH_SHOULDER_L, 0, degToPulse(lPos));
  pca.setPWM(CH_SHOULDER_R, 0, degToPulse(rPos));
}

void setup() {
  Serial.begin(9600);
  delay(500);

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Center each joint ONE AT A TIME to avoid current inrush
  writeShoulder(shoulder_delta);
  delay(300);

  pca.setPWM(CH_ELBOW, 0, degToPulse(elbow_pos));
  delay(300);

  pca.setPWM(CH_WRIST1, 0, degToPulse(wrist1_pos));
  delay(300);

  pca.setPWM(CH_WRIST2, 0, degToPulse(wrist2_pos));
  delay(300);

  pca.setPWM(CH_GRIPPER, 0, degToPulse(gripper_pos));
  delay(300);

  printMenu();
  printStatus();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case '1': selected[0] = !selected[0]; announceToggle(0); break;
      case '2': selected[1] = !selected[1]; announceToggle(1); break;
      case '3': selected[2] = !selected[2]; announceToggle(2); break;
      case '4': selected[3] = !selected[3]; announceToggle(3); break;
      case '5': selected[4] = !selected[4]; announceToggle(4); break;

      case 'a':
        for (int i = 0; i < 5; i++) selected[i] = true;
        Serial.println(">> All joints selected");
        break;

      case 'n':
        for (int i = 0; i < 5; i++) selected[i] = false;
        Serial.println(">> Selection cleared");
        break;

      case 'f': moveSelected(1);  break;
      case 'b': moveSelected(-1); break;
      case 'r': resetSelected();  break;
      case '?': printStatus();    break;

      default: break; // ignore newline / unknown chars
    }
  }
}

void moveSelected(int delta) {
  if (selected[0]) {
    shoulder_delta = constrain(shoulder_delta + delta, SHOULDER_DELTA_MIN, SHOULDER_DELTA_MAX);
    writeShoulder(shoulder_delta);
  }
  if (selected[1]) {
    elbow_pos = constrain(elbow_pos + delta, ELBOW_MIN, ELBOW_MAX);
    pca.setPWM(CH_ELBOW, 0, degToPulse(elbow_pos));
  }
  if (selected[2]) {
    wrist1_pos = constrain(wrist1_pos + delta, WRIST1_MIN, WRIST1_MAX);
    pca.setPWM(CH_WRIST1, 0, degToPulse(wrist1_pos));
  }
  if (selected[3]) {
    wrist2_pos = constrain(wrist2_pos + delta, WRIST2_MIN, WRIST2_MAX);
    pca.setPWM(CH_WRIST2, 0, degToPulse(wrist2_pos));
  }
  if (selected[4]) {
    gripper_pos = constrain(gripper_pos + delta, GRIPPER_MIN, GRIPPER_MAX);
    pca.setPWM(CH_GRIPPER, 0, degToPulse(gripper_pos));
  }
  printStatus();
}

void resetSelected() {
  if (selected[0]) { shoulder_delta = 0; writeShoulder(shoulder_delta); }
  if (selected[1]) { elbow_pos = elbow_center; pca.setPWM(CH_ELBOW, 0, degToPulse(elbow_pos)); }
  if (selected[2]) { wrist1_pos = wrist1_center; pca.setPWM(CH_WRIST1, 0, degToPulse(wrist1_pos)); }
  if (selected[3]) { wrist2_pos = wrist2_center; pca.setPWM(CH_WRIST2, 0, degToPulse(wrist2_pos)); }
  if (selected[4]) { gripper_pos = gripper_center; pca.setPWM(CH_GRIPPER, 0, degToPulse(gripper_pos)); }
  Serial.println(">> Selected joints reset to center");
  printStatus();
}

void announceToggle(int idx) {
  Serial.print(">> ");
  Serial.print(jointNames[idx]);
  Serial.println(selected[idx] ? " added to selection" : " removed from selection");
}

void printStatus() {
  int lPos = constrain(L_CENTER + shoulder_delta, 0, 180);
  int rPos = constrain(R_CENTER - shoulder_delta, 0, 180);

  Serial.println("---- Selection & Positions ----");
  Serial.print("Shoulder ["); Serial.print(selected[0] ? "X" : " ");
  Serial.print("]: delta="); Serial.print(shoulder_delta);
  Serial.print(" (L="); Serial.print(lPos); Serial.print(" R="); Serial.print(rPos); Serial.println(")");
  Serial.print("Elbow    ["); Serial.print(selected[1] ? "X" : " "); Serial.print("]: "); Serial.println(elbow_pos);
  Serial.print("Wrist1   ["); Serial.print(selected[2] ? "X" : " "); Serial.print("]: "); Serial.println(wrist1_pos);
  Serial.print("Wrist2   ["); Serial.print(selected[3] ? "X" : " "); Serial.print("]: "); Serial.println(wrist2_pos);
  Serial.print("Gripper  ["); Serial.print(selected[4] ? "X" : " "); Serial.print("]: "); Serial.println(gripper_pos);
  Serial.println("--------------------------------");
}

void printMenu() {
  Serial.println("=== Multi-Joint Test Ready (Calibrated Shoulder) ===");
  Serial.println("1-5 = toggle Shoulder/Elbow/Wrist1/Wrist2/Gripper");
  Serial.println("a = select all   n = select none");
  Serial.println("f = +1deg all selected   b = -1deg all selected   r = reset selected");
  Serial.println("? = status");
  Serial.println("CAUTION: multiple simultaneous servos = higher current draw.");
  Serial.println("Start with 1-2 joints selected until power supply is confirmed stable.");
}
