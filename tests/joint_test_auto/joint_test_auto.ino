/******************************************************************************
  Elbow & Wrist Automatic Joint Test — No serial input required
  Direct Arduino Servo control (no PCA9685), MG995 servos

  On power-up, this automatically tests each joint ONE AT A TIME in sequence:
    1. Elbow sweeps +-45 deg from center, then returns to center
    2. Wrist 1 sweeps +-45 deg from center, then returns to center
    3. Wrist 2 sweeps +-45 deg from center, then returns to center
  Then it repeats forever, with a pause between full cycles.

  Serial output (9600 baud) shows what's happening, but is not required
  to run the test -- just for monitoring/debugging.

  Pins:
    Elbow   -> Pin 9
    Wrist 1 -> Pin 6
    Wrist 2 -> Pin 5   (rotation)
******************************************************************************/

#include <Servo.h>

Servo elbow;
Servo wrist1;
Servo wrist2;

const int elbow_pin  = 9;
const int wrist1_pin = 6;
const int wrist2_pin = 5;

// Center (parking) position for each joint
const int elbow_center  = 90;
const int wrist1_center = 90;
const int wrist2_center = 90;

// Total allowed swing -- increased from 10 to 45 degrees each way
// (90 degrees total sweep). Adjust this number for a bigger/smaller range.
const int max_swing = 45;

const int move_delay = 500;   // ms to wait after each move, lets servo settle
const int cycle_pause = 2000; // ms pause between full test cycles

void setup() {
  Serial.begin(9600);
  delay(500);

  // Attach and center servos ONE AT A TIME to avoid a current inrush spike
  elbow.attach(elbow_pin);
  elbow.write(elbow_center);
  delay(400);

  wrist1.attach(wrist1_pin);
  wrist1.write(wrist1_center);
  delay(400);

  wrist2.attach(wrist2_pin);
  wrist2.write(wrist2_center);
  delay(400);

  Serial.println("=== Automatic Joint Test Starting ===");
}

void loop() {
  testJoint(elbow, elbow_center, "Elbow");
  delay(500);

  testJoint(wrist1, wrist1_center, "Wrist 1");
  delay(500);

  testJoint(wrist2, wrist2_center, "Wrist 2");
  delay(500);

  Serial.println("=== Cycle complete, pausing ===");
  delay(cycle_pause);
}

void testJoint(Servo &srv, int center, const char* name) {
  Serial.print("Testing ");
  Serial.println(name);

  int highPos = constrain(center + max_swing, 0, 180);
  int lowPos  = constrain(center - max_swing, 0, 180);

  srv.write(highPos);
  Serial.print("  -> ");
  Serial.println(highPos);
  delay(move_delay);

  srv.write(center);
  Serial.print("  -> ");
  Serial.println(center);
  delay(move_delay);

  srv.write(lowPos);
  Serial.print("  -> ");
  Serial.println(lowPos);
  delay(move_delay);

  srv.write(center);
  Serial.print("  -> ");
  Serial.println(center);
  delay(move_delay);
}
