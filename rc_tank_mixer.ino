// =======================================================================================
// RC Tank Drive Mixer for Arduino Pro Micro (ATmega32U4)
// ---------------------------------------------------------------------------------------
// Reads two PWM channels from an RC receiver (throttle + steering) and mixes them
// into a tank drive (left motor + right motor) output, written as 1000-2000us pulses.
//
// Mixing algorithm adapted from BigHappyDude's "diamond mix" via nhutchison/shadow_md_q85.
//
// Pin assignments (Pro Micro):
//   D2 - INT0  : RC input - Forward/Reverse channel (throttle)
//   D3 - INT1  : RC input - Left/Right channel       (steering)
//   D9         : PWM output - LEFT  motor (servo-style 1000-2000us)
//   D10        : PWM output - RIGHT motor (servo-style 1000-2000us)
// =======================================================================================

#include <Servo.h>

// ---------------------------------------------------------------------------------------
// User-tunable configuration
// ---------------------------------------------------------------------------------------
#define THROTTLE_PIN   2     // must be an interrupt-capable pin (D0,D1,D2,D3,D7 on Pro Micro)
#define STEERING_PIN   3     // must be an interrupt-capable pin
#define LEFT_OUT_PIN   9
#define RIGHT_OUT_PIN  10

// Invert a motor if it spins the wrong direction (0 = normal, 1 = inverted)
#define INVERT_LEFT    0
#define INVERT_RIGHT   0

// Set to 1 if you want to swap which stick does what (or just swap the input wires)
#define INVERT_THROTTLE 0
#define INVERT_STEERING 0

// RC pulse range (microseconds). Standard RC is 1000-2000 with 1500 center.
// Widen these slightly if your TX gives a bit more or less than spec.
const int RC_MIN     = 1000;
const int RC_CENTER  = 1500;
const int RC_MAX     = 2000;

// Deadzone around center stick (in microseconds). 1500 +/- this value reads as 0.
const int DEADZONE_US = 30;

// Maximum drive output as a percentage (0-100). Lower this for a "training" speed limit.
const int MAX_SPEED_PCT = 100;

// Failsafe: if no valid pulse received within this many ms, stop motors.
const unsigned long FAILSAFE_MS = 500;

// Loop update interval (ms). 20ms = 50Hz, matches a typical servo/ESC frame rate.
const unsigned long UPDATE_MS = 20;

// Optional debug over USB serial. Comment out to disable.
#define DEBUG

// ---------------------------------------------------------------------------------------
// PWM input capture (interrupt-driven, non-blocking)
// ---------------------------------------------------------------------------------------
volatile unsigned long throttleRise = 0;
volatile unsigned long steeringRise = 0;
volatile int           throttlePulse = RC_CENTER;
volatile int           steeringPulse = RC_CENTER;
volatile unsigned long lastThrottleUpdate = 0;
volatile unsigned long lastSteeringUpdate = 0;

void throttleISR() {
  unsigned long now = micros();
  if (digitalRead(THROTTLE_PIN) == HIGH) {
    throttleRise = now;
  } else {
    unsigned long w = now - throttleRise;
    if (w >= 800 && w <= 2200) {       // sanity-check the pulse width
      throttlePulse = (int)w;
      lastThrottleUpdate = millis();
    }
  }
}

void steeringISR() {
  unsigned long now = micros();
  if (digitalRead(STEERING_PIN) == HIGH) {
    steeringRise = now;
  } else {
    unsigned long w = now - steeringRise;
    if (w >= 800 && w <= 2200) {
      steeringPulse = (int)w;
      lastSteeringUpdate = millis();
    }
  }
}

// ---------------------------------------------------------------------------------------
// Motor outputs (Servo library produces clean 1000-2000us pulses)
// ---------------------------------------------------------------------------------------
Servo leftMotor;
Servo rightMotor;

// ---------------------------------------------------------------------------------------
// Diamond mixing — adapted from BigHappyDude / shadow_md_q85
// Inputs:  stickX, stickY in range 0..255 (128 = centered, joystick-style)
// Outputs: leftOut, rightOut as integer percentages -100..+100
// ---------------------------------------------------------------------------------------
void mixDiamond(int stickX, int stickY, int &leftOut, int &rightOut) {
  // Apply deadzone in the 0..255 domain (joystickFootDeadZoneRange equivalent)
  const int DZ = map(DEADZONE_US, 0, (RC_MAX - RC_CENTER), 0, 127);

  if (abs(stickX - 128) < DZ) stickX = 128;
  if (abs(stickY - 128) < DZ) stickY = 128;

  if (stickX == 128 && stickY == 128) {
    leftOut = 0;
    rightOut = 0;
    return;
  }

  // Map stick to -100..+100 grid (Y: forward positive, X: right positive)
  int YDist = 0;
  int XDist = 0;
  if (stickY < 128)      YDist = map(stickY, 0,   128, 100,  1);
  else if (stickY > 128) YDist = map(stickY, 128, 255, -1, -100);

  if (stickX < 128)      XDist = map(stickX, 0,   128, -100, -1);
  else if (stickX > 128) XDist = map(stickX, 128, 255,  1,  100);

  // Constrain to the diamond by computing intersections of the stick vector
  // with each of the four diamond edges (|x|+|y| = 100).
  float TempXDist = XDist;
  float TempYDist = YDist;

  if (YDist > (XDist + 100)) {            // outside top-left edge
    TempXDist = -100.0f / (1.0f - ((float)YDist / (float)XDist));
    TempYDist = TempXDist + 100.0f;
  } else if (YDist > (100 - XDist)) {     // outside top-right edge
    TempXDist = -100.0f / (-1.0f - ((float)YDist / (float)XDist));
    TempYDist = -TempXDist + 100.0f;
  } else if (YDist < (-XDist - 100)) {    // outside bottom-left edge
    TempXDist =  100.0f / (-1.0f - ((float)YDist / (float)XDist));
    TempYDist = -TempXDist - 100.0f;
  } else if (YDist < (XDist - 100)) {     // outside bottom-right edge
    TempXDist =  100.0f / (1.0f - ((float)YDist / (float)XDist));
    TempYDist =  TempXDist - 100.0f;
  }

  // Translate diamond coordinates to per-track speeds (-100..+100)
  float left  = ((TempXDist + TempYDist - 100.0f) / 2.0f) + 100.0f;
  left = (left - 50.0f) * 2.0f;

  float right = ((TempYDist - TempXDist - 100.0f) / 2.0f) + 100.0f;
  right = (right - 50.0f) * 2.0f;

  leftOut  = constrain((int)left,  -100, 100);
  rightOut = constrain((int)right, -100, 100);
}

// ---------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------
// Convert an RC pulse (1000..2000us) to a joystick-style 0..255 value with 128 = center.
int pulseTo255(int pulse, bool invert) {
  pulse = constrain(pulse, RC_MIN, RC_MAX);
  int v = map(pulse, RC_MIN, RC_MAX, 0, 255);
  if (invert) v = 255 - v;
  return v;
}

// Convert a -100..+100 speed (scaled by MAX_SPEED_PCT) to a 1000..2000us pulse.
int speedToMicros(int speed, bool invert) {
  long scaled = (long)speed * MAX_SPEED_PCT / 100;     // apply speed limit
  scaled = constrain(scaled, -100, 100);
  if (invert) scaled = -scaled;
  return map(scaled, -100, 100, RC_MIN, RC_MAX);
}

void writeStop() {
  leftMotor.writeMicroseconds(RC_CENTER);
  rightMotor.writeMicroseconds(RC_CENTER);
}

// ---------------------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------------------
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  // Pro Micro: don't wait for Serial — the bot must run without USB attached.
#endif

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);

  leftMotor.attach(LEFT_OUT_PIN);
  rightMotor.attach(RIGHT_OUT_PIN);
  writeStop();

  // Give ESCs a moment to see neutral before any commands arrive.
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN), steeringISR, CHANGE);
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_MS) return;
  lastUpdate = now;

  // Snapshot ISR variables with interrupts briefly disabled
  noInterrupts();
  int   tPulse = throttlePulse;
  int   sPulse = steeringPulse;
  unsigned long tAge = now - lastThrottleUpdate;
  unsigned long sAge = now - lastSteeringUpdate;
  interrupts();

  // Failsafe: if either channel is stale, stop.
  if (tAge > FAILSAFE_MS || sAge > FAILSAFE_MS) {
    writeStop();
#ifdef DEBUG
    Serial.println(F("FAILSAFE - no RC signal"));
#endif
    return;
  }

  // Convert pulses to the joystick 0..255 domain expected by mixDiamond
  int stickY = pulseTo255(tPulse, INVERT_THROTTLE);   // forward/reverse
  int stickX = pulseTo255(sPulse, INVERT_STEERING);   // left/right

  int leftSpeed, rightSpeed;   // -100..+100
  mixDiamond(stickX, stickY, leftSpeed, rightSpeed);

  int leftUs  = speedToMicros(leftSpeed,  INVERT_LEFT);
  int rightUs = speedToMicros(rightSpeed, INVERT_RIGHT);

  leftMotor.writeMicroseconds(leftUs);
  rightMotor.writeMicroseconds(rightUs);

#ifdef DEBUG
  static unsigned long lastDbg = 0;
  if (now - lastDbg > 200) {
    lastDbg = now;
    Serial.print(F("T=")); Serial.print(tPulse);
    Serial.print(F("us S=")); Serial.print(sPulse);
    Serial.print(F("us  L=")); Serial.print(leftSpeed);
    Serial.print(F("% R=")); Serial.print(rightSpeed);
    Serial.print(F("%  Lus=")); Serial.print(leftUs);
    Serial.print(F(" Rus=")); Serial.println(rightUs);
  }
#endif
}
