# 🦾 Tony — A Gesture-Controlled Robotic Arm

> Wave your hand. Tony waves back (eventually, once the gripper is calibrated).

Tony is a 5-DOF robotic arm controlled by a glove. Tilt your wrist, twist it, tap a button — and a wireless signal flies from an IMU on your hand to a servo-driven arm across the room. No app, no joystick, just your hand doing the talking.

<p align="center">
  <!-- 📸 Drop a hero shot or demo GIF of Tony here! e.g. docs/images/tony_hero.jpg -->
  <img src="docs/images/tony_hero.jpg" alt="Tony the gesture-controlled robotic arm" width="600">
</p>

<p align="center">
  <a href="#-demo">Demo</a> •
  <a href="#-how-it-works">How it works</a> •
  <a href="#-bill-of-materials">Parts</a> •
  <a href="#-repo-structure">Repo structure</a> •
  <a href="#-gesture-map">Gesture map</a> •
  <a href="#-getting-it-running">Setup</a> •
  <a href="#-serial-command-cheat-sheet">Commands</a> •
  <a href="#-known-quirks--roadmap">Roadmap</a>
</p>

---

## 🎬 Demo

<!-- 🎥 Drop your demo video/gif here, e.g.: -->
<!-- ![Tony demo](docs/videos/tony_demo.gif) -->

*(Video/GIF coming soon — see `docs/videos/`.)*

---

## ✨ How It Works

Tony is a **two-board wireless puppet show**:

```
   YOUR HAND                          TONY'S BRAIN
┌───────────────┐   Bluetooth    ┌───────────────────┐
│  Glove (Arduino│  ───(HC-05)──▶│  Arm (Arduino Uno) │
│  Uno + MPU6050 │   "mode,ΔP,ΔR"│  + PCA9685 driver  │
│  + push button)│                │  + 6 servos        │
└───────────────┘                └───────────────────┘
```

1. **The glove** reads pitch/roll off an **MPU6050 IMU** strapped to the back of your hand.
2. A **push button** lets you burst-tap (1–4 taps) to pick *which joint* you're currently driving — Shoulder, Elbow, Wrist, or Gripper — without needing four separate switches.
3. Every mode switch **recenters** to wherever your hand currently is, so there's no sudden jump when you change joints.
4. The glove streams `mode,deltaPitch,deltaRoll` packets over an **HC-05 Bluetooth module** at ~20Hz.
5. **The arm** (a second Arduino + **PCA9685** servo driver) receives packets over its own HC-05, and uses **velocity control**: the bigger your tilt, the faster the joint moves — not "tilt = absolute angle." If the Bluetooth link drops, the arm simply stops moving instead of holding (or worse, running away with) a stale command.
6. All 6 servos ease smoothly toward their targets (no instant jumps), and every joint's safe range is locked in through a one-time **jog & calibrate** routine so Tony never over-rotates a joint and strips a gear.

---

## 🧰 Bill of Materials

**🧤 Glove (Master / Transmitter)**
| Part | Role |
|---|---|
| Arduino Uno | Reads sensor + button, sends BT packets |
| MPU6050 (IMU) | Pitch/roll of your hand |
| HC-05 (Bluetooth, master) | Sends gesture packets to the arm |
| Push button | Tap-burst mode switching (1–4 taps) |
| Cloth glove | IMU + button mount |

**🦾 Arm**
| Part | Role |
|---|---|
| Arduino Uno | Receives BT packets, runs control logic |
| PCA9685 | 16-channel PWM servo driver (I²C) |
| HC-05 (Bluetooth, receiver) | Receives gesture packets from the glove |
| MG-995 servos | Elbow, wrist (×2), gripper, one shoulder axis |
| DS-3218 servo | Shoulder (the heavy-lifting axis — extra torque to support the arm's weight) |
| 3D-printed frame | From a [Thingiverse](https://www.thingiverse.com/) model — *link the exact one here!* |

---

## 📁 Repo Structure

```
.
├── firmware/                      ← the actual "run Tony" code
│   ├── glove_master/               → flash to the GLOVE Arduino
│   │   └── trigger_cycle_master_glove_v3.ino
│   └── arm_slave/                  → flash to the ARM Arduino
│       └── trigger_cycle_arm_slave_v4.ino
│
├── tests/                          ← bring-up & calibration sketches, run these FIRST
│   ├── full_arm_joint_test/            manual one-joint-at-a-time test, direct servo pins
│   ├── joint_test_auto/                automatic sweep test, no serial input needed
│   ├── multi_joint_test_calibrated/    PCA9685 test w/ calibrated shoulder formula
│   ├── single_pot_multi_joint_control/ drive any joint from a single potentiometer
│   └── trigger_cycle_slave_v1_legacy/  earlier arm-side sketch, kept for reference
│
├── docs/
│   ├── images/                     ← wiring photos, build pics, glove close-ups
│   └── videos/                     ← demo clips / gifs
│
└── LICENSE
```

> 💡 Each sketch lives in its own folder matching its filename — that's an Arduino IDE requirement, so you can just open the `.ino` directly and it'll behave.

---

## 🖐️ Gesture Map

The glove uses **tap bursts** on a single button to jump straight to a joint — no cycling through modes one at a time.

| Taps (quick burst) | Mode | Pitch controls | Roll controls |
|:---:|---|---|---|
| 1 | **Shoulder** | Shoulder up/down | — |
| 2 | **Elbow** | Elbow bend | — |
| 3 | **Wrist** | Wrist pitch | Wrist roll |
| 4 | **Gripper** | — (unused) | Open/close (twist your wrist like you're grabbing something) |

Tap 5+ times and it just wraps back around (1, 2, 3, 4, 1, 2...) — so overshooting isn't a big deal, just tap again. Tony's onboard LED blinks *N* times to confirm which mode you landed on.

---

## ⚙️ Getting It Running

### 1. Install the libraries
Via Arduino IDE → *Sketch → Include Library → Manage Libraries*:
- `Adafruit PWM Servo Driver Library` (for the PCA9685, on the arm side)
- `Servo` (built-in, used by the direct-servo test sketches)
- `Wire` and `SoftwareSerial` are built-in — no install needed.

### 2. Wire it up
- **Glove:** MPU6050 on I²C (`SDA`/`SCL`), push button on a digital pin with `INPUT_PULLUP`, HC-05 on `SoftwareSerial` pins.
- **Arm:** PCA9685 on I²C (`SDA → A4`, `SCL → A5`), HC-05 on `SoftwareSerial` pins 10/11, servos on PCA9685 channels 0–5 (Shoulder L/R, Elbow, Wrist1, Wrist2, Gripper).
- Power the servos from a separate supply (buck converter) sharing common ground with the Arduino — **don't** run MG995/DS3218 servos off the Arduino's 5V pin, that's how you get brownouts.

### 3. Bring it up in this order (don't skip to live control!)
1. Run `tests/full_arm_joint_test` or `tests/joint_test_auto` first, to confirm every servo moves correctly on its own pin before anything touches the PCA9685.
2. Move to `tests/multi_joint_test_calibrated` once servos are on the PCA9685, to confirm the shoulder's dual-servo calibration (different centers, opposite rotation directions for the MG995/DS3218 pair).
3. Flash `firmware/arm_slave/trigger_cycle_arm_slave_v4.ino` to the arm, open Serial Monitor @ 9600, and **jog the gripper** (the only joint whose limits aren't pre-locked — see cheat sheet below).
4. Flash `firmware/glove_master/trigger_cycle_master_glove_v3.ino` to the glove.
5. Power both boards, pair the HC-05s, and start gesturing.

---

## ⌨️ Serial Command Cheat Sheet
*(arm side, via Serial Monitor @ 9600 baud — mainly for setup/calibration, not needed for normal gesture use)*

| Key | Action |
|:---:|---|
| `1`–`5` | Enter jog mode for Shoulder / Elbow / Wrist1 / Wrist2 / Gripper |
| `+` / `-` | Nudge active joint by 1° |
| `>` / `<` | Nudge active joint by 5° |
| `n` | Mark current jog position as that joint's MIN |
| `x` | Mark current jog position as that joint's MAX |
| `q` | Lock in the jog and return to live control |
| `t` | Capture current pose as "Stand Tall" |
| `d` | Capture current pose as "Full Down" |
| `g` | Play the next demo routine (calibration sweep / wave / breathe / pose cycle) |
| `?` | Print current status |

A real gesture from the glove always interrupts a running demo instantly and hands control back to your hand.

---

## 🐛 Known Quirks & Roadmap

- **Gripper limits are the only ones not pre-calibrated** — jog it with `5` before your first run each session (or once you're happy with it, hardcode it like the other four joints).
- **~450ms pause after your last tap** before a mode change commits, since the glove needs to know your tap burst is over. Works fine, but if the latency bugs you, swapping in a physical 3–4 position rotary switch would make mode-switching instant.
- **`tests/trigger_cycle_slave_v1_legacy`** is kept around purely for reference/history — the real arm firmware is `firmware/arm_slave/trigger_cycle_arm_slave_v4.ino`.
- Next up: maybe ditch the tap-burst entirely for that rotary switch, and get the "Stand Tall"/"Full Down" poses dialed in for a cleaner demo routine.

---

## 🙌 Credits

- 3D-printed arm structure adapted from a model on [Thingiverse]((https://www.thingiverse.com/thing:1838120)) — *swap in the exact link!*
- Built by **Jahnavi Karanam & Pranav Krishna**.

## 📄 License

MIT — see [LICENSE](LICENSE). Fork it, build your own Tony, make it wave at people.
