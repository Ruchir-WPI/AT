// ============================================================
//  Back Scrubber Gantry Controller v6
//  16 inches horizontal, 4 inches vertical per step
//  4x microstepping, starts top-left
//
//  A3 (FAST)    → first press starts fast, second press stops,
//                 third press restarts from beginning
//  A2 (MEDIUM)  → same as fast but medium speed
//  A1 (AXIS)    → toggles active axis between horizontal/vertical
//  A0 (REVERSE) → reverses direction of ONLY the currently active axis
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
#define BTN_MEDIUM  A2
#define BTN_FAST    A3

// ---- MOTION PARAMETERS — 4x microstepping -------------------
const int   STEPS_PER_REV = 800;
const float PITCH_MM      = 8.0;
const float STEPS_PER_MM  = STEPS_PER_REV / PITCH_MM;  // 100 steps/mm

const float H_TRAVEL_MM   = 406.4;  // 16 inches
const float V_STEP_MM     = 101.6;  // 4 inches

const long  H_STEPS       = (long)(H_TRAVEL_MM * STEPS_PER_MM);  // 40,640
const long  V_STEPS       = (long)(V_STEP_MM   * STEPS_PER_MM);  // 10,160

const long  ACCEL_STEPS   = 100;

// Speed settings
const unsigned int H_MEDIUM = 232;
const unsigned int H_FAST   = 232;
const unsigned int V_MEDIUM = 2000;
const unsigned int V_FAST   = 1000;

// ---- DIRECTION STATE ----------------------------------------
// Each axis has its own independent direction, both start going
// toward the "far" end from top-left (right and down)
bool hGoingRight = true;   // horizontal: true = right, false = left
bool vGoingDown  = true;   // vertical: true = down, false = up

// ---- STATE --------------------------------------------------
enum SpeedMode { MEDIUM_SPEED, FAST_SPEED };
SpeedMode currentSpeed = FAST_SPEED;

bool sequenceActive  = false;
bool isPaused        = false;
bool doingHorizontal = true;  // which axis is currently active

// Track steps completed in current move for mid-move resume
long hStepsCompleted = 0;
long vStepsCompleted = 0;

unsigned int hDelay() { return (currentSpeed == MEDIUM_SPEED) ? H_MEDIUM : H_FAST; }
unsigned int vDelay() { return (currentSpeed == MEDIUM_SPEED) ? V_MEDIUM : V_FAST; }

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
  pinMode(BTN_MEDIUM,  INPUT_PULLUP);
  pinMode(BTN_FAST,    INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Ready. Press Medium or Fast to begin.");
}

// ============================================================
//  MAIN LOOP — only runs when sequence not active
// ============================================================
void loop() {
  if (!sequenceActive) {
    if (digitalRead(BTN_MEDIUM) == LOW) {
      delay(50);
      if (digitalRead(BTN_MEDIUM) == LOW) {
        while (digitalRead(BTN_MEDIUM) == LOW);
        currentSpeed = MEDIUM_SPEED;
        Serial.println("MEDIUM — Starting...");
        startSequence();
      }
    }
    if (digitalRead(BTN_FAST) == LOW) {
      delay(50);
      if (digitalRead(BTN_FAST) == LOW) {
        while (digitalRead(BTN_FAST) == LOW);
        currentSpeed = FAST_SPEED;
        Serial.println("FAST — Starting...");
        startSequence();
      }
    }
  }
}

// ============================================================
//  START SEQUENCE — always begins with horizontal
// ============================================================
void startSequence() {
  sequenceActive   = true;
  isPaused         = false;
  doingHorizontal  = true;
  hGoingRight      = true;   // reset to default directions
  vGoingDown       = true;
  hStepsCompleted  = 0;
  vStepsCompleted  = 0;
  enableMotors();
  runActiveAxis();
}

// ============================================================
//  RUN ACTIVE AXIS
//  Runs whichever axis is currently active continuously
//  bouncing back and forth until stopped or axis switched
// ============================================================
void runActiveAxis() {
  while (sequenceActive && !isPaused) {
    if (doingHorizontal) {
      // How many steps remain in current pass
      long stepsRemaining = H_STEPS - hStepsCompleted;
      if (stepsRemaining <= 0) {
        // Finished a full pass — flip direction and reset
        hGoingRight = !hGoingRight;
        hStepsCompleted = 0;
        stepsRemaining = H_STEPS;
        delay(300);
      }

      pumpOn();
      Serial.println(hGoingRight ? "H → right" : "H ← left");
      long stepsDone = moveMotor(H_PUL, H_DIR, stepsRemaining,
                                 hGoingRight ? HIGH : LOW,
                                 hDelay());
      hStepsCompleted += stepsDone;

      // If completed full pass, flip direction next iteration
      if (stepsDone >= stepsRemaining) {
        hGoingRight = !hGoingRight;
        hStepsCompleted = 0;
      }
      pumpOff();

    } else {
      // Vertical axis
      long stepsRemaining = V_STEPS - vStepsCompleted;
      if (stepsRemaining <= 0) {
        vGoingDown = !vGoingDown;
        vStepsCompleted = 0;
        stepsRemaining = V_STEPS;
        delay(300);
      }

      Serial.println(vGoingDown ? "V ↓ down" : "V ↑ up");
      long stepsDone = moveMotor(V_PUL, V_DIR, stepsRemaining,
                                 vGoingDown ? HIGH : LOW,
                                 vDelay());
      vStepsCompleted += stepsDone;

      if (stepsDone >= stepsRemaining) {
        vGoingDown = !vGoingDown;
        vStepsCompleted = 0;
      }
    }
  }
}

// ============================================================
//  MOVE MOTOR
//  Checks all buttons every 10 steps
//  Returns number of steps actually completed
// ============================================================
long moveMotor(int pulPin, int dirPin, long steps, bool direction,
               unsigned int pulseDelayUs) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(5);

  long         rampSteps = min(ACCEL_STEPS, steps / 2);
  unsigned int slowDelay = pulseDelayUs * 3;
  long         i         = 0;

  for (i = 0; i < steps; i++) {
    if (i % 10 == 0) {

      // FAST or MEDIUM — second press stops
      if (digitalRead(BTN_FAST) == LOW && currentSpeed == FAST_SPEED) {
        delay(50);
        if (digitalRead(BTN_FAST) == LOW) {
          while (digitalRead(BTN_FAST) == LOW);
          handleStop();
          return i;
        }
      }
      if (digitalRead(BTN_MEDIUM) == LOW && currentSpeed == MEDIUM_SPEED) {
        delay(50);
        if (digitalRead(BTN_MEDIUM) == LOW) {
          while (digitalRead(BTN_MEDIUM) == LOW);
          handleStop();
          return i;
        }
      }

      // AXIS — switch active axis, resume from where each left off
      if (digitalRead(BTN_AXIS) == LOW) {
        delay(50);
        if (digitalRead(BTN_AXIS) == LOW) {
          while (digitalRead(BTN_AXIS) == LOW);
          // Save progress on current axis
          if (doingHorizontal) hStepsCompleted += i;
          else                 vStepsCompleted += i;
          doingHorizontal = !doingHorizontal;
          Serial.println(doingHorizontal ? "Switched to HORIZONTAL" : "Switched to VERTICAL");
          return i;  // exit move, runActiveAxis will pick up new axis
        }
      }

      // REVERSE — flip only the currently active axis direction
      if (digitalRead(BTN_REVERSE) == LOW) {
        delay(50);
        if (digitalRead(BTN_REVERSE) == LOW) {
          while (digitalRead(BTN_REVERSE) == LOW);
          if (doingHorizontal) {
            hGoingRight = !hGoingRight;
            direction = hGoingRight ? HIGH : LOW;
            // Recalculate remaining steps to go back
            steps = i * 2;
            Serial.println(hGoingRight ? "H reversed → right" : "H reversed ← left");
          } else {
            vGoingDown = !vGoingDown;
            direction = vGoingDown ? HIGH : LOW;
            steps = i * 2;
            Serial.println(vGoingDown ? "V reversed ↓ down" : "V reversed ↑ up");
          }
          digitalWrite(dirPin, direction);
          delayMicroseconds(5);
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
//  HANDLE STOP
//  Stops everything, waits for Fast or Medium to restart
//  from the very beginning
// ============================================================
void handleStop() {
  sequenceActive = false;
  isPaused       = false;
  pumpOff();
  disableMotors();
  Serial.println("STOPPED. Press Fast or Medium to restart from beginning.");

  // Wait for fast or medium to restart
  bool restarted = false;
  while (!restarted) {
    if (digitalRead(BTN_FAST) == LOW) {
      delay(50);
      if (digitalRead(BTN_FAST) == LOW) {
        while (digitalRead(BTN_FAST) == LOW);
        currentSpeed = FAST_SPEED;
        restarted = true;
        Serial.println("FAST — Restarting...");
        startSequence();
      }
    }
    if (digitalRead(BTN_MEDIUM) == LOW) {
      delay(50);
      if (digitalRead(BTN_MEDIUM) == LOW) {
        while (digitalRead(BTN_MEDIUM) == LOW);
        currentSpeed = MEDIUM_SPEED;
        restarted = true;
        Serial.println("MEDIUM — Restarting...");
        startSequence();
      }
    }
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
