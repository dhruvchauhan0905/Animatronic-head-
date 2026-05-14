# Animatronic-head-
# 🤖 Animatronic Robotic Head — Real-Time Face Tracking

> A 9-servo animatronic head that mirrors human facial movements in real time using MediaPipe face mesh tracking, Arduino, and a PCA9685 PWM driver. Includes a full calibration system, physics-based motion control, and predictive compensation for smooth, lag-free neck movement.

## Features

- **Real-time face tracking** via MediaPipe Face Mesh (468 landmarks + iris refinement)
- **9-servo system** — eyes (X/Y), left/right eyelids (2 servos each), and a 3-DOF neck (tilt, vertical, yaw)
- **Calibration phase** — 2-step Y/N confirmation to set user neutral pose before tracking begins
- **Center locking** — neck snaps to neutral when head is stationary, preventing servo drift
- **Eyelid blink detection** — threshold-based open/close state with smooth interpolation
- **Yaw-corrected eye Y** — blends toward individual eye ratio when head turns sharply
- **Perspective-aware eyelid swap** — swaps left/right eyelid state when head turns beyond ±48° yaw
- **Physics-based motion** — per-channel speed, acceleration, and deceleration constants
- **Throttled neck updates** — CH6/CH7 commit target every 17 ms (40% read reduction) to eliminate jitter
- **Predictive target compensation** — 40 ms lookahead cancels serial + processing lag on neck servos
- **Live motor overlay** — real-time HUD showing all 9 channel values colour-coded by deviation

---

## System Architecture

```
Webcam (1280×720)
  │
  ▼
MediaPipe FaceMesh  ──  refine_landmarks=True  ──  iris landmarks 468, 473
  │
  ├─ Iris position (relative to eye corners)       →  Eye X / Y target
  ├─ Forehead / nose / chin distance ratio         →  Neck vertical target
  ├─ Face tilt angle (left/right face points)      →  Neck tilt target
  ├─ Nose-to-face asymmetry ratio                  →  Neck yaw target
  └─ Eyelid landmark distance (159↔145, 386↔374)  →  Blink state
  │
  ▼
Python tracking script
  │  Rolling average smoothing (window = 10 frames)
  │  Center lock, dead zones, yaw damping
  │  Packet: "EyeX,EyeY,LidR1,LidR2,LidL1,LidL2,Neck6,Neck7,Neck8\n"
  │
  ▼
PySerial  ──  COM9  ──  115200 baud
  │
  ▼
Arduino Uno
  │  readVisionSerial()  — parses comma-delimited packet
  │  updateNeck67()      — throttle + predictive commit every 17 ms
  │  driveEyeY()         — hybrid physics for CH1
  │  drivePhysics67()    — physics for CH6/CH7
  │  drivePhysics8()     — physics for CH8
  │  loop delay(10)      — 100 Hz
  │
  ▼
PCA9685  (I2C, 60 Hz PWM)
  │
  ├─ CH0  Eye X     (direct pulse)
  ├─ CH1  Eye Y     (physics)
  ├─ CH2  LidR1     (direct pulse)
  ├─ CH3  LidR2     (direct pulse)
  ├─ CH4  LidL1     (direct pulse)
  ├─ CH5  LidL2     (direct pulse)
  ├─ CH6  Neck A    (throttle + physics)
  ├─ CH7  Neck B    (throttle + physics)
  └─ CH8  Neck Yaw  (physics)
```

---

## Channel Mapping

| Channel | Name | Drive type | Neutral | Range | Notes |
|---|---|---|---|---|---|
| CH0 | Eye X | Direct pulse | 330 | 195 – 465 | Horizontal iris tracking |
| CH1 | Eye Y | Physics | 350 | 260 – 445 | Vertical iris tracking |
| CH2 | Right Eyelid 1 | Direct pulse | 330 | 193 – 477 | Open = 477, Closed = 330 |
| CH3 | Right Eyelid 2 | Direct pulse | 330 | 193 – 477 | Open = 193, Closed = 330 |
| CH4 | Left Eyelid 1 | Direct pulse | 330 | 193 – 477 | Open = 193, Closed = 330 |
| CH5 | Left Eyelid 2 | Direct pulse | 330 | 193 – 477 | Open = 477, Closed = 330 |
| CH6 | Neck A (vertical + tilt) | Throttled physics | 90° | 5° – 175° | Paired with CH7 |
| CH7 | Neck B (vertical + tilt) | Throttled physics | 90° | 5° – 175° | Paired with CH6 |
| CH8 | Neck Yaw | Physics | 90° | 3° – 175° | Maps yaw ratio −0.30 → 0.30 |

> CH6 and CH7 are driven together. Vertical lift adds to both equally; tilt adds equally in the same direction, producing roll from the difference.

---

## Serial Packet Format

Python sends one ASCII line per frame over serial:

```
{EyeX},{EyeY},{LidR1},{LidR2},{LidL1},{LidL2},{Neck6},{Neck7},{NeckYaw}\n
```

Example:
```
328,352,477,193,193,477,94,86,91
```

CH0–CH5 are raw PCA9685 pulse counts. CH6–CH8 are angles in degrees (0–180), converted to pulses on the Arduino side via `angleToPulse()`.

> ⚠️ **Baud rate mismatch in current code:** `face_tracker.py` opens serial at **9600** but `animatronic_head.ino` uses `Serial.begin(115200)`. Set both to **115200** — see Setup section.

---

## Motion Tuning

### Arduino — CH1 Eye Y physics

```cpp
float eyeMaxSpeed      = 9.5;    // max degrees per loop tick
float eyeAccel         = 0.55;   // acceleration ramp
float eyeDecel         = 0.78;   // deceleration multiplier
float eyeDecelDistance = 10;     // distance from target to begin braking
```

### Arduino — CH6 / CH7 Neck (vertical + tilt)

```cpp
float maxSpeed67      = 9.52;    // 40% raised from original 6.8
float accel67         = 0.336;   // 40% raised from original 0.24
float decel67         = 0.84;
float decelDistance67 = 16;

const unsigned long NECK67_INTERVAL = 17;    // ms — commit throttle
const float         LOOKAHEAD_SEC   = 0.040; // 40 ms predictive lead
```

Incoming CH6/CH7 targets are **averaged** across all serial packets received in the 17 ms window, then a velocity estimate is computed and the target is nudged 40 ms ahead before it is handed to the physics driver. This removes the lag introduced by serial buffering and processing delay.

### Arduino — CH8 Neck Yaw

```cpp
float maxSpeed8      = 5.6;
float accel8         = 0.18;
float decel8         = 0.86;
float decelDistance8 = 18;
```

### Python — Smoothing factors

| Parameter | Value | Effect |
|---|---|---|
| Eye X blend | 0.90 | Very fast, near-instant |
| Eye Y blend | 0.67 | Slightly damped |
| Eyelid blend | 0.39 | Slow, natural-looking blink |
| Yaw / vertical / tilt window | 10 frames | Rolling average |

### Python — Center lock thresholds

| Lock | Engages when | Releases when |
|---|---|---|
| Neck center (CH6/CH7) | total error < 5 | total error > 13 |
| Yaw center (CH8) | \|avgYaw\| < 0.04 | \|avgYaw\| > 0.09 |

When locked, the corresponding channels hold exactly at their neutral value regardless of small sensor noise.

---

## Calibration

On launch the script runs a calibration loop before tracking starts.

1. Sit in your neutral resting position facing the camera
2. Wait at least 10 seconds for stable face detection
3. Press **`Y`** — first confirmation, captures current pose as tentative reference
4. Press **`Y`** again — confirms and locks the reference zero
5. Press **`N`** at any step to reset and recapture

The calibration stores:

| Variable | What it captures |
|---|---|
| `baseEyeRelX / baseEyeRelY` | Iris position normalised to eye corner span |
| `baseTop / baseBottom` | Forehead-to-nose and nose-to-chin ratios (normalised to face width) |
| `baseTilt` | Face roll angle in degrees |
| `baseYaw` | Nose-to-face-edge asymmetry ratio |

All live values are computed as **deltas from this reference**, so the robot stays centred regardless of where you sit relative to the camera.

---

## Eyelid Logic

Blink detection uses vertical distance between eyelid landmarks:

```
Left eye:   lm[159] (top lid)  ↔  lm[145] (bottom lid)
Right eye:  lm[386] (top lid)  ↔  lm[374] (bottom lid)
```

| Event | Threshold | Result |
|---|---|---|
| Opening | distance > 0.0132 | State → Open |
| Closing | distance < 0.0108 | State → Closed |

Hysteresis between the two thresholds prevents rapid state flicker.

When **yaw exceeds ±48°**, the left and right eyelid states are swapped — at that angle the camera perspective reverses which eye appears to be blinking.

---

## Hardware

### Components

| Component | Qty | Notes |
|---|---|---|
| Arduino Uno | 1 | Microcontroller |
| PCA9685 16-ch PWM driver | 1 | I2C, 60 Hz |
| MG996R servo | 3 | Neck — CH6, CH7, CH8 |
| SG90 servo | 6 | Eyes — CH0, CH1 + eyelids — CH2–CH5 |
| 5V 5A SMPS | 1 | Powers servos via PCA9685 V+ terminal |
| Webcam | 1 | Face tracking input |

### Wiring

```
Arduino Uno              PCA9685
-----------              -------
5V          ──────────▶  VCC
GND         ──────────▶  GND
A4 (SDA)    ──────────▶  SDA
A5 (SCL)    ──────────▶  SCL

External PSU (5V 5A)     PCA9685
--------------------     -------
+           ──────────▶  V+  (power terminal)
−           ──────────▶  GND (power terminal)
```

> ⚠️ Never power servos from Arduino 5V. Always use the external PSU through the PCA9685 V+ terminal.

### Pulse range

```cpp
#define SERVOMIN 140   // ≈ 0°
#define SERVOMAX 520   // ≈ 180°
```

---

## Setup

### 1. Python dependencies

```bash
pip install mediapipe opencv-python pyserial
```

### 2. Arduino libraries

Install via Arduino Library Manager:
- `Adafruit PWM Servo Driver Library`
- `Wire` (built-in)

### 3. Upload firmware

Open `firmware/animatronic_head.ino`, select **Arduino Uno** and your COM port, then upload.

### 4. Fix baud rate (important)

In `tracking/face_tracker.py`, change line 4:
```python
# Before
arduino = serial.Serial('COM9', 9600)

# After
arduino = serial.Serial('COM9', 115200)   # match Arduino sketch
```

Also update `'COM9'` to your actual port (`/dev/ttyUSB0` on Linux, `/dev/cu.usbmodem...` on Mac).

### 5. Run the tracker

```bash
python tracking/face_tracker.py
```

Follow the calibration prompt on screen. Press **`Y`** twice to confirm neutral pose. Press **`ESC`** to quit.

---

## Repository Structure

```
animatronic-head/
├── README.md
├── firmware/
│   └── animatronic_head.ino      # Arduino sketch
├── tracking/
│   └── face_tracker.py           # Python face tracking + serial bridge
├── cad/
│   ├── eye_hemisphere.stl
│   ├── neck_mechanism.stl
│   └── source/                   # Fusion 360 .f3d source files
├── media/
│   ├── demo.gif
│   ├── wiring_diagram.svg
│   └── photos/
└── docs/
    └── wiring.md
```

---

## 3D Printing

| Setting | Value |
|---|---|
| Material | PLA |
| Layer height | 0.2 mm |
| Infill | 20% |
| Supports | Yes (neck assembly) |

---


## Author

**Dhruv** — Engineering student, Delhi  
Interests: robotics, animatronics, hardware systems  
GitHub: [@dhruvchauhan0905](https://github.com/dhruvchauhan0905)

---

## License

MIT License — free to use, modify, and build on.
