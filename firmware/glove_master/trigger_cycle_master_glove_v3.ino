/******************************************************************************
  Trigger-Cycle Master (glove side) v2 -- ONE MPU6050, ONE trigger, 4 modes,
  DIRECT (non-sequential) mode addressing via tap-count bursts.

  WHAT CHANGED FROM v1:
    - v1: each tap incremented mode by 1 (1->2->3->1...). To get from Wrist
      back to Elbow you had to cycle all the way around.
    - v2: taps are counted in a burst. You tap N times quickly (within
      TAP_WINDOW_MS of each other), then the code waits for the burst to
      end and jumps STRAIGHT to mode N. No more cycling.
        1 tap  -> Mode 1 (Shoulder)
        2 taps -> Mode 2 (Elbow)
        3 taps -> Mode 3 (Wrist: pitch=Wrist1, roll=Wrist2)
        4 taps -> Mode 4 (Gripper: roll drives gripper open/close)
      5+ taps wraps back around via modulo, so it's forgiving if you
      overshoot -- worst case you just tap again.

    - Added Mode 4: Gripper. Pitch is unused in this mode; roll (twisting
      your wrist) opens/closes the gripper. This mirrors how you'd
      naturally mime "grabbing" with a twist of the hand.

    - NOTE: choreographed demo routines (calibration sweep, wave, breathe,
      pose cycle) are controlled entirely from the arm-side serial monitor
      ('g' = next demo), not from the glove. The glove only ever sends
      gesture packets for modes 1-4 -- there's no 5-tap-for-demo behavior
      here, intentionally, so the glove doesn't have to juggle live control
      and demo triggering at the same time.

  TRADE-OFF TO KNOW: there's a mandatory ~450ms pause after your last tap
  before the mode commits (needed to know the burst is over). If that
  latency bugs you, the more robust upgrade is a physical 3-4 position
  rotary switch wired to spare digital pins -- instant, no ambiguity,
  looks more finished in a demo. Software fix here needs zero new parts.

  RELATIVE CONTROL: unchanged from v1 -- every mode change recenters
  pitch/roll to "wherever your hand already is" so there's no jump.

  PACKET FORMAT sent over Bluetooth (comma-separated, newline-terminated):
      mode,deltaPitch,deltaRoll\n
******************************************************************************/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>

const int MPU_ADDR = 0x68;

const int TRIGGER_PIN = 15;   // momentary switch to GND, internal pull-up
const int FLEX_PIN    = A0;   // only used if you swap to the flex-sensor line below
const int FLEX_THRESHOLD = 600; // tune once you have a sensor wired

const int LED_PIN = 2; // blinks N times to show current mode

const int BT_RX = 4, BT_TX = 5; // change to match your HC-05/06 wiring
SoftwareSerial BT(BT_RX, BT_TX);

// ---- Mode addressing ----
const int MODE_COUNT = 4; // 1=Shoulder, 2=Elbow, 3=Wrist, 4=Gripper
int mode = 1;

// ---- Tap-count burst detection (replaces v1's simple cycle) ----
bool lastRawPressed = false;
bool debouncedPressed = false;
unsigned long lastEdgeTime = 0;
const unsigned long DEBOUNCE_MS = 40; // kills raw electrical bounce

int tapCount = 0;
unsigned long lastTapTime = 0;
const unsigned long MIN_TAP_GAP_MS = 80;   // ignore taps faster than this (bounce/false retrigger)
const unsigned long TAP_WINDOW_MS  = 450;  // burst ends after this much silence -> commit mode

float centerPitch = 0, centerRoll = 0;

const unsigned long SEND_INTERVAL_MS = 50; // ~20Hz
unsigned long lastSend = 0;

void wakeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);
  Wire.endTransmission(true);
}

void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
}

float rollFromAccel(int16_t ay, int16_t az) {
  return atan2((float)ay, (float)az) * 180.0 / PI;
}
float pitchFromAccel(int16_t ax, int16_t ay, int16_t az) {
  return atan2(-(float)ax, sqrt((float)ay * (float)ay + (float)az * (float)az)) * 180.0 / PI;
}

bool readTriggerRaw() {
  // Real button (momentary to GND, pull-up enabled):
  return digitalRead(TRIGGER_PIN) == LOW;

  // NO BUTTON YET? Comment the line above and uncomment this one instead --
  // works with a flex sensor bent past a threshold. Same debounce/tap-count
  // logic downstream handles it exactly the same as a button:
  // return analogRead(FLEX_PIN) > FLEX_THRESHOLD;
}

void blinkMode(int m) {
  for (int i = 0; i < m; i++) {
    digitalWrite(LED_PIN, HIGH); delay(120);
    digitalWrite(LED_PIN, LOW);  delay(120);
  }
}

void recenter() {
  int16_t ax, ay, az;
  readAccel(ax, ay, az);
  centerPitch = pitchFromAccel(ax, ay, az);
  centerRoll  = rollFromAccel(ay, az);
}

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin();
  delay(200);
  wakeMPU();
  delay(100);

  recenter();
  Serial.println(F("Trigger-cycle master v2 ready. Mode 1 = Shoulder. Tap N times fast to jump to Mode N."));
  blinkMode(mode);
}

void loop() {
  // ---- debounce ----
  bool rawPressed = readTriggerRaw();
  if (rawPressed != lastRawPressed) {
    lastEdgeTime = millis();
    lastRawPressed = rawPressed;
  }
  if ((millis() - lastEdgeTime) > DEBOUNCE_MS && rawPressed != debouncedPressed) {
    debouncedPressed = rawPressed;

    // Count a tap on the press edge, ignoring anything too fast to be a real tap
    if (debouncedPressed && (millis() - lastTapTime) > MIN_TAP_GAP_MS) {
      tapCount++;
      lastTapTime = millis();
    }
  }

  // ---- burst finished? commit directly to mode = tapCount ----
  if (tapCount > 0 && (millis() - lastTapTime) > TAP_WINDOW_MS) {
    int newMode = ((tapCount - 1) % MODE_COUNT) + 1; // 1..MODE_COUNT, wraps if you overshoot
    tapCount = 0;

    if (newMode != mode) {
      mode = newMode;
      recenter(); // fresh baseline for the new mode -- no jump
      Serial.print(F("Mode -> ")); Serial.println(mode);
    }
    blinkMode(mode); // confirms even if you re-tapped into the same mode
  }

  // ---- send at a fixed rate ----
  if (millis() - lastSend < SEND_INTERVAL_MS) return;
  lastSend = millis();

  int16_t ax, ay, az;
  readAccel(ax, ay, az);
  float pitch = pitchFromAccel(ax, ay, az);
  float roll  = rollFromAccel(ay, az);

  float deltaPitch = pitch - centerPitch;
  float deltaRoll  = roll - centerRoll;

  String packet = String(mode) + "," + String(deltaPitch, 1) + "," + String(deltaRoll, 1);
  BT.println(packet);
  Serial.println(packet); // debug
}
