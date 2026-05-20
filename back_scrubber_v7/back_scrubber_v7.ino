// ============================================================
//  Back Scrubber Gantry Controller v7
//  15 inches horizontal, 20 inches vertical (5 sweeps, 4 steps down)
//  4x microstepping, starts top-left
//
//  A3 (SPEED)   → cycles Slow → Medium → Fast → Pause, always starts Slow
//                 hold 3 seconds = hard reset regardless of state
//  A2 (IGNORED) → broken wire, does nothing
//  A1 (AXIS)    → cycles Horizontal → Vertical → Pause
//  A0 (REVERSE) → cycles Forward → Backward → Pause (per active axis only)
//
//  Cycle: 5 horizontal sweeps, 4 vertical steps down, then home
// ============================================================

// ---- PIN ASSIGNMENTS ----------------------------------------
#define H_PUL       2
#define H_DIR       3
#define H_EN        4

#define V_PUL       5
#define V_DIR       6
#define V_EN        7

#define PUMP_PIN    13

#define BTN_REVERSE A0
#define BTN_AXIS    A1
#define BTN_IGNORED A2  // broken, never read
#define BTN_SPEED   A3

// ---- MOTION PARAMETERS — 4x microstepping -------------------
const int   STEPS_PER_REV = 800;
const float PITCH_MM      = 8.0;
const float STEPS_PER_MM  = STEPS_PER_REV / PITCH_MM;  // 100 steps/mm

const float H_TRAVEL_MM   = 381.0;   // 15 inches
const float V_STEP_MM     = 101.6;   // 4 inches per step

const long  H_STEPS       = (long)(H_TRAVEL_MM * STEPS_PER_MM);  // 38,100
const long  V_STEPS       = (long)(V_STEP_MM   * STEPS_PER_MM);  // 10,160

const int   NUM_H_SWEEPS  = 5;    // 5 horizontal sweeps
const int   NUM_V_STEPS   = 4;    // 4 steps down between sweeps
const long  ACCEL_STEPS   = 100;

// Speed delays (µs) — lower = faster
const unsigned int H_SLOW   = 464;
const unsigned int H_MEDIUM = 368;
const unsigned int H_FAST   = 309;

const unsigned int V_SLOW   = 2000;
const unsigned int V_MEDIUM = 1587;
const unsigned int V_FAST   = 1333;

// Hold duration for hard reset (ms)
const unsigned long HOLD_RESET_MS = 3000;

// ---- DIRECTION DEFINES --------------------------------------
#define DIR_RIGHT  HIGH
#define DIR_LEFT   LOW
#define DIR_DOWN   HIGH
#define DIR_UP     LOW

// ---- CYCLE STATES -------------------------------------------
enum SpeedState { SPD_SLOW, SPD_MEDIUM, SPD_FAST, SPD_PAUSE };
enum AxisState  { AXIS_H, AXIS_V, AXIS_PAUSE };
enum RevState   { REV_FORWARD, REV_BACKWARD, REV_PAUSE };

SpeedState speedState = SPD_SLOW;
AxisState  axisState  = AXIS_H;
RevState   hRevState  = REV_FORWARD;  // horizontal reverse state
RevState   vRevState  = REV_FORWARD;  // vertical reverse state (independent)

// ---- SEQUENCE STATE -----------------------------------------
bool sequenceActive  = false;
int  currentSweep    = 0;   // 0-4, which horizontal sweep we're on
int  vStepsDone      = 0;   // how many vertical steps completed

// Steps completed in current partial move (for resume)
long hStepsCompleted = 0;
long vStepsCompleted = 0;

// Track horizontal direction for alternating sweeps
bool sweepGoingRight = true;

// ---- HELPERS ------------------------------------------------
unsigned int hDelay() {
  if (speedState == SPD_SLOW)   return H_SLOW;
  if (speedState == SPD_MEDIUM) return H_MEDIUM;
  return H_FAST;
}
unsigned int vDelay() {
  if (speedState == SPD_SLOW)   return V_SLOW;
  if (speedState == SPD_MEDIUM) return V_MEDIUM;
  return V_FAST;
}

bool hGoingRight() {
  // Base direction from sweep alternation, flipped if reversed
  bool base = sweepGoingRight;
  return (hRevState == REV_FORWARD) ? base : !base;
}
bool vGoingDown() {
  return (vRevState == REV_FORWARD) ? true : false;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  digitalWrite(H_EN, HIGH);
  digitalWrite(V_EN, HIGH);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(H_PUL,    OUTPUT);
  pinMode(H_DIR,    OUTPUT);
  pinMode(H_EN,     OUTPUT);
  pinMode(V_PUL,    OUTPUT);
  pinMode(V_DIR,    OUTPUT);
  pinMode(V_EN,     OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  pinMode(BTN_REVERSE, INPUT_PULLUP);
  pinMode(BTN_AXIS,    INPUT_PULLUP);
  pinMode(BTN_SPEED,   INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Ready. Press Speed (A3) to begin at Slow.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  if (!sequenceActive) {
    checkSpeedButton();
  }
}

// ============================================================
//  SPEED BUTTON HANDLER (A3)
//  Short press → cycle speed state
//  Hold 3s → hard reset
// ============================================================
void checkSpeedButton() {
  if (digitalRead(BTN_SPEED) == LOW) {
    delay(50);
    if (digitalRead(BTN_SPEED) == LOW) {
      unsigned long pressTime = millis();

      // Wait for release or hold threshold
      while (digitalRead(BTN_SPEED) == LOW) {
        if (millis() - pressTime >= HOLD_RESET_MS) {
          hardReset();
          return;
        }
      }

      // Short press — cycle speed
      cycleSpeed();
    }
  }
}

// ============================================================
//  CYCLE SPEED
//  Slow → Medium → Fast → Pause → (back to Slow on next press)
//  First press always starts from Slow
// ============================================================
void cycleSpeed() {
  switch (speedState) {
    case SPD_SLOW:
      speedState = SPD_MEDIUM;
      Serial.println("Speed: MEDIUM");
      break;
    case SPD_MEDIUM:
      speedState = SPD_FAST;
      Serial.println("Speed: FAST");
      break;
    case SPD_FAST:
      speedState = SPD_PAUSE;
      Serial.println("Speed: PAUSE");
      pauseSequence();
      return;
    case SPD_PAUSE:
      speedState = SPD_SLOW;
      Serial.println("Speed: SLOW — restarting");
      startSequence();
      return;
  }

  // If sequence not active yet, start it
  if (!sequenceActive) {
    startSequence();
  }
}

// ============================================================
//  HARD RESET
// ============================================================
void hardReset() {
  while (digitalRead(BTN_SPEED) == LOW); // wait for release
  sequenceActive   = false;
  speedState       = SPD_SLOW;
  axisState        = AXIS_H;
  hRevState        = REV_FORWARD;
  vRevState        = REV_FORWARD;
  currentSweep     = 0;
  vStepsDone       = 0;
  hStepsCompleted  = 0;
  vStepsCompleted  = 0;
  sweepGoingRight  = true;
  pumpOff();
  disableMotors();
  Serial.println("HARD RESET. Press Speed to begin at Slow.");
}

// ============================================================
//  START SEQUENCE
// ============================================================
void startSequence() {
  sequenceActive  = true;
  currentSweep    = 0;
  vStepsDone      = 0;
  hStepsCompleted = 0;
  vStepsCompleted = 0;
  sweepGoingRight = true;
  axisState       = AXIS_H;
  hRevState       = REV_FORWARD;
  vRevState       = REV_FORWARD;
  enableMotors();
  Serial.println("Sequence started.");
  runSequence();
}

// ============================================================
//  PAUSE SEQUENCE
// ============================================================
void pauseSequence() {
  pumpOff();
  disableMotors();
  Serial.println("PAUSED. Press any button to resume.");

  // Wait for any button press
  while (true) {
    if (digitalRead(BTN_SPEED)   == LOW ||
        digitalRead(BTN_AXIS)    == LOW ||
        digitalRead(BTN_REVERSE) == LOW) {
      delay(50);
      if (digitalRead(BTN_SPEED)   == LOW ||
          digitalRead(BTN_AXIS)    == LOW ||
          digitalRead(BTN_REVERSE) == LOW) {

        // Check for hard reset hold on speed button
        if (digitalRead(BTN_SPEED) == LOW) {
          unsigned long pressTime = millis();
          while (digitalRead(BTN_SPEED) == LOW) {
            if (millis() - pressTime >= HOLD_RESET_MS) {
              hardReset();
              return;
            }
          }
        }

        while (digitalRead(BTN_SPEED)   == LOW ||
               digitalRead(BTN_AXIS)    == LOW ||
               digitalRead(BTN_REVERSE) == LOW);

        // Resume
        if (speedState == SPD_PAUSE) speedState = SPD_SLOW;
        enableMotors();
        Serial.println("Resuming...");
        runSequence();
        return;
      }
    }
  }
}

// ============================================================
//  RUN SEQUENCE
//  5 horizontal sweeps with 4 vertical steps between them
//  then home
// ============================================================
void runSequence() {
  while (sequenceActive && currentSweep < NUM_H_SWEEPS) {

    // --- HORIZONTAL SWEEP ---
    if (axisState == AXIS_H || axisState == AXIS_PAUSE) {
      long stepsLeft = H_STEPS - hStepsCompleted;
      if (stepsLeft > 0) {
        Serial.print("Sweep ");
        Serial.print(currentSweep + 1);
        Serial.println(hGoingRight() ? " → right" : " ← left");

        pumpOn();
        long done = moveMotor(H_PUL, H_DIR, stepsLeft,
                              hGoingRight() ? DIR_RIGHT : DIR_LEFT,
                              hDelay(), true);
        pumpOff();
        hStepsCompleted += done;

        if (hStepsCompleted < H_STEPS) return;  // paused/stopped mid-move
      }

      // Sweep complete
      hStepsCompleted = 0;
      sweepGoingRight = !sweepGoingRight;  // alternate next sweep direction
      currentSweep++;

      if (currentSweep >= NUM_H_SWEEPS) break;

      delay(300);
    }

    // --- VERTICAL STEP ---
    if (vStepsDone < NUM_V_STEPS) {
      if (axisState == AXIS_V || axisState == AXIS_H) {
        long stepsLeft = V_STEPS - vStepsCompleted;
        if (stepsLeft > 0) {
          Serial.println(vGoingDown() ? "↓ stepping down" : "↑ stepping up");
          long done = moveMotor(V_PUL, V_DIR, stepsLeft,
                                vGoingDown() ? DIR_DOWN : DIR_UP,
                                vDelay(), false);
          vStepsCompleted += done;

          if (vStepsCompleted < V_STEPS) return;  // paused mid-move
        }

        vStepsCompleted = 0;
        vStepsDone++;
        delay(300);
      }
    }
  }

  if (currentSweep >= NUM_H_SWEEPS) {
    // Sequence complete — home
    Serial.println("Sequence complete. Homing...");
    homeSequence();
  }
}

// ============================================================
//  HOME SEQUENCE
//  Go all the way left (since last sweep ends going right)
//  then go all the way up (reverse all vertical steps taken)
// ============================================================
void homeSequence() {
  pumpOff();

  // Go left full horizontal distance
  Serial.println("Homing: going left...");
  moveMotorRaw(H_PUL, H_DIR, H_STEPS, DIR_LEFT, H_FAST);
  delay(300);

  // Go up — reverse all vertical steps taken
  long totalVSteps = (long)NUM_V_STEPS * V_STEPS;
  Serial.println("Homing: going up...");
  moveMotorRaw(V_PUL, V_DIR, totalVSteps, DIR_UP, V_FAST);
  delay(300);

  disableMotors();
  sequenceActive  = false;
  currentSweep    = 0;
  vStepsDone      = 0;
  hStepsCompleted = 0;
  vStepsCompleted = 0;
  sweepGoingRight = true;
  speedState      = SPD_SLOW;

  Serial.println("Homed. Press Speed to run again.");
}

// ============================================================
//  MOVE MOTOR — checks all buttons every 10 steps
//  isHorizontal: true = check reverse for H axis
//  Returns steps actually completed
// ============================================================
long moveMotor(int pulPin, int dirPin, long steps, bool direction,
               unsigned int pulseDelayUs, bool isHorizontal) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(5);

  long         rampSteps = min(ACCEL_STEPS, steps / 2);
  unsigned int slowDelay = pulseDelayUs * 3;
  long         i         = 0;

  for (i = 0; i < steps; i++) {
    if (i % 10 == 0) {

      // SPEED BUTTON — cycle or hold reset
      if (digitalRead(BTN_SPEED) == LOW) {
        delay(50);
        if (digitalRead(BTN_SPEED) == LOW) {
          unsigned long pressTime = millis();
          while (digitalRead(BTN_SPEED) == LOW) {
            if (millis() - pressTime >= HOLD_RESET_MS) {
              hardReset();
              return i;
            }
          }
          // Short press — cycle speed
          cycleSpeed();
          if (speedState == SPD_PAUSE || !sequenceActive) return i;
        }
      }

      // AXIS BUTTON — cycle H → V → Pause
      if (digitalRead(BTN_AXIS) == LOW) {
        delay(50);
        if (digitalRead(BTN_AXIS) == LOW) {
          while (digitalRead(BTN_AXIS) == LOW);
          switch (axisState) {
            case AXIS_H:
              axisState = AXIS_V;
              Serial.println("Axis: VERTICAL");
              break;
            case AXIS_V:
              axisState = AXIS_PAUSE;
              Serial.println("Axis: PAUSE");
              pauseSequence();
              return i;
            case AXIS_PAUSE:
              axisState = AXIS_H;
              Serial.println("Axis: HORIZONTAL");
              break;
          }
          return i;  // exit move, runSequence picks up new axis
        }
      }

      // REVERSE BUTTON — cycle Forward → Backward → Pause for active axis
      if (digitalRead(BTN_REVERSE) == LOW) {
        delay(50);
        if (digitalRead(BTN_REVERSE) == LOW) {
          while (digitalRead(BTN_REVERSE) == LOW);
          RevState &rev = isHorizontal ? hRevState : vRevState;
          switch (rev) {
            case REV_FORWARD:
              rev = REV_BACKWARD;
              Serial.println(isHorizontal ? "H: BACKWARD" : "V: BACKWARD");
              // Reverse direction mid-move, travel back same steps
              direction = !direction;
              digitalWrite(dirPin, direction);
              delayMicroseconds(5);
              steps = i * 2;
              break;
            case REV_BACKWARD:
              rev = REV_PAUSE;
              Serial.println(isHorizontal ? "H: PAUSE" : "V: PAUSE");
              pauseSequence();
              return i;
            case REV_PAUSE:
              rev = REV_FORWARD;
              Serial.println(isHorizontal ? "H: FORWARD" : "V: FORWARD");
              break;
          }
        }
      }
    }

    // Trapezoidal acceleration
    unsigned int currentDelay;
    if (i < rampSteps) {
      currentDelay = slowDelay - (unsigned int)((slowDelay - pulseDelayUs) * i / rampSteps);
    } else if (i >= steps - rampSteps) {
      long stepsFromEnd = steps - i;
      currentDelay = slowDelay - (unsigned int)((slowDelay - pulseDelayUs) * stepsFromEnd / rampSteps);
    } else {
      currentDelay = pulseDelayUs;
    }

    digitalWrite(pulPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(currentDelay);
  }

  return i;
}

// ============================================================
//  MOVE MOTOR RAW — no button checks, used for homing only
// ============================================================
void moveMotorRaw(int pulPin, int dirPin, long steps, bool direction,
                  unsigned int pulseDelayUs) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(5);
  for (long i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(pulseDelayUs);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(pulseDelayUs);
  }
}

// ============================================================
//  ENABLE / DISABLE
// ============================================================
void enableMotors() {
  digitalWrite(H_EN, LOW);
  digitalWrite(V_EN, LOW);
  delayMicroseconds(200);
}

void disableMotors() {
  digitalWrite(H_EN, HIGH);
  digitalWrite(V_EN, HIGH);
}

// ============================================================
//  PUMP
// ============================================================
void pumpOn() {
  digitalWrite(PUMP_PIN, HIGH);
  Serial.println("Pump ON");
}

void pumpOff() {
  digitalWrite(PUMP_PIN, LOW);
  Serial.println("Pump OFF");
}
