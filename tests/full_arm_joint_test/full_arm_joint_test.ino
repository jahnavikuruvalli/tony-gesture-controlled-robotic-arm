/******************************************************************************
  Full Arm Joint Test — Shoulder (2 servos), Elbow, Wrist1, Wrist2, Gripper
  Direct Arduino Servo control (no PCA9685), MG995 servos

  All 6 servos are attached and centered at startup, ONE AT A TIME with a
  delay between each, to avoid a current inrush spike.

  Only ONE joint moves at a time -- select it, then step it. This is
  intentional: moving multiple MG995s simultaneously is what caused the
  earlier power supply voltage collapse. Testing one at a time keeps
  current draw low and predictable.

  Pins:
    Shoulder L -> Pin 8
    Shoulder R -> Pin 7   (mirrors Shoulder L automatically)
    Elbow      -> Pin 9
    Wrist 1    -> Pin 6
    Wrist 2    -> Pin 5
    Gripper    -> Pin 3

  Serial commands:
    1 = select Shoulder    2 = select Elbow      3 = select Wrist 1
    4 = select Wrist 2     5 = select Gripper
    f = move active joint forward 1 degree
    b = move active joint backward 1 degree
    r = reset active joint to its center position
    ? = print current status of all joints

  PHYSICAL LIMITS:
    Wrist 1 is clamped to 0-50 degrees (known physical stop at 50).
    All other joints default to 0-180 -- these have NOT been physically
    tested yet. Go slowly (one 'f'/'b' press at a time) the first time
    you run each joint, and stop immediately if you feel resistance or
    hear grinding. Once you find a real limit, update the matching
    MIN/MAX constants below.
******************************************************************************/

#include <Servo.h>

Servo shoulderL;
Servo shoulderR;
Servo elbow;
Servo wrist1;
Servo wrist2;
Servo gripper;

const int shoulderL_pin = 8;
const int shoulderR_pin = 7;
const int elbow_pin     = 9;
const int wrist1_pin    = 6;
const int wrist2_pin    = 5;
const int gripper_pin   = 3;

// ---- Center positions ----
const int shoulder_center = 90;
const int elbow_center    = 90;
const int wrist1_center   = 25;  // mid-point of its safe 0-50 range
const int wrist2_center   = 90;
const int gripper_center  = 90;

// ---- Limits (MIN/MAX). Update these once real physical limits are found ----
const int SHOULDER_MIN = 0,  SHOULDER_MAX = 180; // not yet tested -- go slow
const int ELBOW_MIN    = 0,  ELBOW_MAX    = 180; // not yet tested -- go slow
const int WRIST1_MIN   = 0,  WRIST1_MAX   = 50;  // known: hard stop at 50
const int WRIST2_MIN   = 0,  WRIST2_MAX   = 180; // not yet tested -- go slow
const int GRIPPER_MIN  = 0,  GRIPPER_MAX  = 180; // not yet tested -- go slow

const int SHOULDER_MIRROR_MAX = 180; // used to mirror shoulderR = MAX - shoulderL

// ---- Current tracked positions ----
int shoulder_pos = shoulder_center;
int elbow_pos    = elbow_center;
int wrist1_pos   = wrist1_center;
int wrist2_pos   = wrist2_center;
int gripper_pos  = gripper_center;

// Active joint: 1=Shoulder, 2=Elbow, 3=Wrist1, 4=Wrist2, 5=Gripper
int active_joint = 1;

void setup() {
  Serial.begin(9600);
  delay(500);

  // Attach + center each servo ONE AT A TIME to avoid current inrush
  shoulderL.attach(shoulderL_pin);
  shoulderL.write(shoulder_pos);
  delay(350);

  shoulderR.attach(shoulderR_pin);
  shoulderR.write(SHOULDER_MIRROR_MAX - shoulder_pos);
  delay(350);

  elbow.attach(elbow_pin);
  elbow.write(elbow_pos);
  delay(350);

  wrist1.attach(wrist1_pin);
  wrist1.write(wrist1_pos);
  delay(350);

  wrist2.attach(wrist2_pin);
  wrist2.write(wrist2_pos);
  delay(350);

  gripper.attach(gripper_pin);
  gripper.write(gripper_pos);
  delay(350);

  printMenu();
  printStatus();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case '1': active_joint = 1; Serial.println(">> Active joint: SHOULDER"); break;
      case '2': active_joint = 2; Serial.println(">> Active joint: ELBOW");    break;
      case '3': active_joint = 3; Serial.println(">> Active joint: WRIST 1");  break;
      case '4': active_joint = 4; Serial.println(">> Active joint: WRIST 2");  break;
      case '5': active_joint = 5; Serial.println(">> Active joint: GRIPPER");  break;

      case 'f': moveActiveJoint(1);  break;
      case 'b': moveActiveJoint(-1); break;
      case 'r': resetActiveJoint();  break;
      case '?': printStatus();       break;

      default: break; // ignore newline / unknown chars
    }
  }
}

void moveActiveJoint(int delta) {
  switch (active_joint) {
    case 1:
      shoulder_pos = constrain(shoulder_pos + delta, SHOULDER_MIN, SHOULDER_MAX);
      shoulderL.write(shoulder_pos);
      shoulderR.write(SHOULDER_MIRROR_MAX - shoulder_pos);
      Serial.print("Shoulder position: ");
      Serial.println(shoulder_pos);
      break;

    case 2:
      elbow_pos = constrain(elbow_pos + delta, ELBOW_MIN, ELBOW_MAX);
      elbow.write(elbow_pos);
      Serial.print("Elbow position: ");
      Serial.println(elbow_pos);
      break;

    case 3:
      wrist1_pos = constrain(wrist1_pos + delta, WRIST1_MIN, WRIST1_MAX);
      wrist1.write(wrist1_pos);
      Serial.print("Wrist 1 position: ");
      Serial.println(wrist1_pos);
      break;

    case 4:
      wrist2_pos = constrain(wrist2_pos + delta, WRIST2_MIN, WRIST2_MAX);
      wrist2.write(wrist2_pos);
      Serial.print("Wrist 2 position: ");
      Serial.println(wrist2_pos);
      break;

    case 5:
      gripper_pos = constrain(gripper_pos + delta, GRIPPER_MIN, GRIPPER_MAX);
      gripper.write(gripper_pos);
      Serial.print("Gripper position: ");
      Serial.println(gripper_pos);
      break;
  }
}

void resetActiveJoint() {
  switch (active_joint) {
    case 1:
      shoulder_pos = shoulder_center;
      shoulderL.write(shoulder_pos);
      shoulderR.write(SHOULDER_MIRROR_MAX - shoulder_pos);
      Serial.println("Shoulder reset to center");
      break;

    case 2:
      elbow_pos = elbow_center;
      elbow.write(elbow_pos);
      Serial.println("Elbow reset to center");
      break;

    case 3:
      wrist1_pos = wrist1_center;
      wrist1.write(wrist1_pos);
      Serial.println("Wrist 1 reset to center");
      break;

    case 4:
      wrist2_pos = wrist2_center;
      wrist2.write(wrist2_pos);
      Serial.println("Wrist 2 reset to center");
      break;

    case 5:
      gripper_pos = gripper_center;
      gripper.write(gripper_pos);
      Serial.println("Gripper reset to center");
      break;
  }
}

void printStatus() {
  Serial.println("---- Current positions ----");
  Serial.print("Shoulder: "); Serial.println(shoulder_pos);
  Serial.print("Elbow:    "); Serial.println(elbow_pos);
  Serial.print("Wrist 1:  "); Serial.println(wrist1_pos);
  Serial.print("Wrist 2:  "); Serial.println(wrist2_pos);
  Serial.print("Gripper:  "); Serial.println(gripper_pos);
  Serial.println("----------------------------");
}

void printMenu() {
  Serial.println("=== Full Arm Joint Test Ready ===");
  Serial.println("1=Shoulder 2=Elbow 3=Wrist1 4=Wrist2 5=Gripper");
  Serial.println("f=+1deg  b=-1deg  r=reset  ?=status");
  Serial.println("Wrist1 clamped 0-50 (known limit). Others 0-180, untested -- go slow.");
}
