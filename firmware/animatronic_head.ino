#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 140
#define SERVOMAX 520

int pulseX = 330, pulseY = 375;
int ch2 = 330, ch3 = 330, ch4 = 330, ch5 = 330;

// ===== CH1 HYBRID EYE Y — original unchanged =====
float eyeYTarget = 375;
float eyeYPos = 375;
float eyeYVel = 0;

// ===== CH8 YAW — original unchanged =====
float target8 = 90, pos8 = 90, vel8 = 0;

String incoming = "";

// ===== CH1 EYE Y CONSTANTS — original unchanged =====
float eyeMaxSpeed      = 9.5;
float eyeAccel         = 0.55;
float eyeDecel         = 0.78;
float eyeDecelDistance = 10;

// ===== CH6/CH7 CONSTANTS — speed raised 40% from original =====
// Original: maxSpeed67=6.8, accel67=0.24
// x1.4   : maxSpeed67=9.52, accel67=0.336
float maxSpeed67      = 9.52;
float accel67         = 0.336;
float decel67         = 0.84;   // original unchanged
float decelDistance67 = 16;     // original unchanged

// ===== CH8 CONSTANTS — original unchanged =====
float maxSpeed8      = 5.6;
float accel8         = 0.18;
float decel8         = 0.86;
float decelDistance8 = 18;

// ==========================================================
//  CH6/CH7 THROTTLE + PREDICTIVE COMPENSATION
//
//  40% read reduction: loop runs every 10ms (100Hz).
//  Original = 100 target updates/sec.
//  40% less  = 60 updates/sec = update every 17ms.
//  So ch6/ch7 target only commits every 17ms.
//
//  Predictive compensation: every commit we calculate how
//  fast the target is moving (degrees/sec) and look 40ms
//  ahead. Motor aims at where the neck WILL be, not where
//  it was — this cancels the serial+processing lag.
// ==========================================================
const unsigned long NECK67_INTERVAL = 17;   // ms — 40% less than 10ms loop
const float         LOOKAHEAD_SEC   = 0.040; // 40ms prediction window

float accum6 = 0, accum7 = 0;
int   accumCount = 0;

float committedTarget6 = 90.0;
float committedTarget7 = 90.0;

float prevAvg6 = 90.0, prevAvg7 = 90.0;

float pos6 = 90, vel6 = 0;
float pos7 = 90, vel7 = 0;

unsigned long lastNeck67Update = 0;

// ----------------------------------------------------------
int angleToPulse(float angle) {
  return map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ----------------------------------------------------------
void readVisionSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      int p1 = incoming.indexOf(',');
      int p2 = incoming.indexOf(',', p1 + 1);
      int p3 = incoming.indexOf(',', p2 + 1);
      int p4 = incoming.indexOf(',', p3 + 1);
      int p5 = incoming.indexOf(',', p4 + 1);
      int p6 = incoming.indexOf(',', p5 + 1);
      int p7 = incoming.indexOf(',', p6 + 1);
      int p8 = incoming.indexOf(',', p7 + 1);

      if (p1>0 && p2>0 && p3>0 && p4>0 && p5>0 && p6>0 && p7>0 && p8>0) {
        pulseX = incoming.substring(0,      p1).toInt();
        pulseY = incoming.substring(p1 + 1, p2).toInt();
        ch2    = incoming.substring(p2 + 1, p3).toInt();
        ch3    = incoming.substring(p3 + 1, p4).toInt();
        ch4    = incoming.substring(p4 + 1, p5).toInt();
        ch5    = incoming.substring(p5 + 1, p6).toInt();

        // Accumulate ch6/ch7 — commit happens on timer below
        float raw6 = incoming.substring(p6 + 1, p7).toFloat();
        float raw7 = incoming.substring(p7 + 1, p8).toFloat();
        target8    = incoming.substring(p8 + 1).toFloat();

        eyeYTarget = pulseY;

        accum6 += raw6;
        accum7 += raw7;
        accumCount++;
      }
      incoming = "";
    }
    else {
      incoming += c;
    }
  }
}

// ----------------------------------------------------------
//  Runs every loop. Only commits a new target to ch6/ch7
//  every 17ms (40% less than 10ms loop = 40% read reduction).
//  Applies predictive compensation to remove lag.
// ----------------------------------------------------------
void updateNeck67() {
  unsigned long now = millis();
  if (now - lastNeck67Update < NECK67_INTERVAL) return;

  float dt = (now - lastNeck67Update) / 1000.0;  // actual elapsed seconds
  lastNeck67Update = now;

  if (accumCount > 0) {
    float avg6 = accum6 / accumCount;
    float avg7 = accum7 / accumCount;

    // Velocity of target in degrees/sec
    float targetVel6 = (avg6 - prevAvg6) / dt;
    float targetVel7 = (avg7 - prevAvg7) / dt;

    // Predict where target will be LOOKAHEAD_SEC from now
    // This compensates for serial delay + processing delay
    float predicted6 = avg6 + targetVel6 * LOOKAHEAD_SEC;
    float predicted7 = avg7 + targetVel7 * LOOKAHEAD_SEC;

    // Clamp prediction to safe servo range
    committedTarget6 = constrain(predicted6, 5.0, 175.0);
    committedTarget7 = constrain(predicted7, 5.0, 175.0);

    prevAvg6 = avg6;
    prevAvg7 = avg7;

    accum6 = 0;
    accum7 = 0;
    accumCount = 0;
  }
}

// ===== ORIGINAL PHYSICS FUNCTIONS — structure unchanged =====

void driveEyeY(float &pos, float &vel, float target) {
  float error = target - pos;
  if (abs(error) > eyeDecelDistance) {
    vel += (error > 0 ? eyeAccel : -eyeAccel);
  } else {
    vel *= eyeDecel;
  }
  vel = constrain(vel, -eyeMaxSpeed, eyeMaxSpeed);
  if (abs(error) < 0.3) vel = 0;
  pos += vel;
  pos  = constrain(pos, 260, 445);
}

void drivePhysics67(float &pos, float &vel, float target,
                    float maxSpeed, float accel, float decel, float decelDistance) {
  float error = target - pos;
  if (abs(error) > decelDistance) {
    vel += (error > 0 ? accel : -accel);
  } else {
    vel *= decel;
  }
  vel = constrain(vel, -maxSpeed, maxSpeed);
  if (abs(error) < 0.4) vel = 0;
  pos += vel;
  pos  = constrain(pos, 3, 178);
}

void drivePhysics8(float &pos, float &vel, float target,
                   float maxSpeed, float accel, float decel, float decelDistance) {
  float error = target - pos;
  if (abs(error) > decelDistance) {
    vel += (error > 0 ? accel : -accel);
  } else {
    vel *= decel;
  }
  vel = constrain(vel, -maxSpeed, maxSpeed);
  if (abs(error) < 0.4) vel = 0;
  pos += vel;
  pos  = constrain(pos, 3, 178);
}

// ----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);
  lastNeck67Update = millis();
}

void loop() {

  readVisionSerial();
  updateNeck67();

  // CH0 — direct, original
  pwm.setPWM(0, 0, pulseX);

  // CH1 — hybrid eye Y, original
  driveEyeY(eyeYPos, eyeYVel, eyeYTarget);
  pwm.setPWM(1, 0, (int)eyeYPos);

  // CH2-5 — eyelids direct, original
  pwm.setPWM(2, 0, ch2);
  pwm.setPWM(3, 0, ch3);
  pwm.setPWM(4, 0, ch4);
  pwm.setPWM(5, 0, ch5);

  // CH6/CH7 — throttled + predicted target, original physics function
  drivePhysics67(pos6, vel6, committedTarget6, maxSpeed67, accel67, decel67, decelDistance67);
  drivePhysics67(pos7, vel7, committedTarget7, maxSpeed67, accel67, decel67, decelDistance67);
  pwm.setPWM(6, 0, angleToPulse(pos6));
  pwm.setPWM(7, 0, angleToPulse(pos7));

  // CH8 — yaw, original unchanged
  drivePhysics8(pos8, vel8, target8, maxSpeed8, accel8, decel8, decelDistance8);
  pwm.setPWM(8, 0, angleToPulse(pos8));

  delay(10);
}
