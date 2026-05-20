// ============================================================
//  Back Scrubber Gantry Controller — Full Version
//  2x ZK-SMC02 (PUL/DIR/EN) + Peristaltic Pump + Limit Switches
//
//  Buttons: Start, Slow, Medium, Fast, Stop
//  Limit switches: H-Left (NC), H-Right (NC), V-Top (NO), V-Bottom (NO)
// ============================================================

// ---- PIN ASSIGNMENTS ----------------------------------------
#define H_PUL       2
#define H_DIR       3
#define H_EN        4

#define V_PUL       5
#define V_DIR       6
#define V_EN        7

#define PUMP_PIN    13

#define BTN_START   8
#define BTN_SLOW    A0
#define BTN_MEDIUM  A1
#define BTN_FAST    A2
#define BTN_STOP    A3

// NC switches — normally LOW (pulled to GND through NC), goes HIGH when triggered
#define LIMIT_H_LEFT  9
#define LIMIT_H_RIGHT 10

// NO switches — normally HIGH (INPUT_PULLUP), goes LOW when triggered
#define LIMIT_V_TOP    11
#define LIMIT_V_BOTTOM 12

// ---- MOTION PARAMETERS --------------------------------------
const int   STEPS_PER_REV  = 3200;
const float PITCH_MM       = 8.0;
const float STEPS_PER_MM   = STEPS_PER_REV / PITCH_MM;  // 400 steps/mm

const float H_TRAVEL_MM    = 609.6;   // 24 inches
const float V_STEP_MM      = 101.6;   // 4 inches
const long  H_STEPS        = (long)(H_TRAVEL_MM * STEPS_PER_MM);  // 243,840
const long  V_STEP_STEPS   = (long)(V_STEP_MM   * STEPS_PER_MM);  // 40,640

const int   NUM_PASSES     = 6;
const long  ACCEL_STEPS    = 400;

// Speed presets — pulse delay in microseconds (lower = faster)
// Normal:  H = 15 RPM, V = 30 RPM
// Medium:  H = 22 RPM, V = 45 RPM (midpoint)
// Fast:    H = 30 RPM, V = 60 RPM

const unsigned int H_SLOW    = 1250;   // 15 RPM
const unsigned int H_MEDIUM  = 833;    // 22 RPM
const unsigned int H_FAST    = 625;    // 30 RPM

const unsigned int V_SLOW    = 625;    // 30 RPM
const unsigned int V_MEDIUM  = 416;    // 45 RPM
const unsigned int V_FAST    = 312;    // 60 RPM

const unsigned int HOME_DELAY = 312;   // homing always at fast speed

#define H_RIGHT  HIGH
#define H_LEFT   LOW
#define V_UP     HIGH
#define V_DOWN   LOW

// ---- STATE --------------------------------------------------
enum SpeedMode { SLOW, MEDIUM, FAST };
SpeedMode currentSpeed = SLOW;  // default to slow on boot

unsigned int hDelay() {
  if (currentSpeed == SLOW)   return H_SLOW;
  if (currentSpeed == MEDIUM) return H_MEDIUM;
  return H_FAST;
}

unsigned int vDelay() {
  if (currentSpeed == SLOW)   return V_SLOW;
  if (currentSpeed == MEDIUM) return V_MEDIUM;
  return V_FAST;
}

// Stop/pause state
volatile bool stopPressed    = false;
volatile bool isPaused       = false;
bool          sequenceActive = false;

// Track position in sequence for pause/resume
int  currentPass   = 0;
bool passGoRight   = true;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  // Disable motors FIRST before pinMode to prevent boot spin
  digitalWrite(H_EN, HIGH);
  digitalWrite(V_EN, HIGH);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(H_PUL,  OUTPUT);
  pinMode(H_DIR,  OUTPUT);
  pinMode(H_EN,   OUTPUT);
  pinMode(V_PUL,  OUTPUT);
  pinMode(V_DIR,  OUTPUT);
  pinMode(V_EN,   OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  // Buttons — all INPUT_PULLUP (active LOW)
  pinMode(BTN_START,  INPUT_PULLUP);
  pinMode(BTN_SLOW,   INPUT_PULLUP);
  pinMode(BTN_MEDIUM, INPUT_PULLUP);
  pinMode(BTN_FAST,   INPUT_PULLUP);
  pinMode(BTN_STOP,   INPUT_PULLUP);

  // Limit switches
  // NC switches: wire pulls pin LOW normally, opens (HIGH) when triggered
  // We use INPUT (no pullup) since NC wire holds it LOW
  pinMode(LIMIT_H_LEFT,  INPUT);
  pinMode(LIMIT_H_RIGHT, INPUT);

  // NO switches: INPUT_PULLUP holds HIGH, switch pulls LOW when triggered
  pinMode(LIMIT_V_TOP,    INPUT_PULLUP);
  pinMode(LIMIT_V_BOTTOM, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Ready. Select speed then press Start.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  // Speed buttons only work when sequence is NOT active
  if (!sequenceActive) {
    if (digitalRead(BTN_SLOW) == LOW) {
      delay(50);
      if (digitalRead(BTN_SLOW) == LOW) {
        currentSpeed = SLOW;
        Serial.println("Speed: SLOW");
        while (digitalRead(BTN_SLOW) == LOW);
      }
    }
    if (digitalRead(BTN_MEDIUM) == LOW) {
      delay(50);
      if (digitalRead(BTN_MEDIUM) == LOW) {
        currentSpeed = MEDIUM;
        Serial.println("Speed: MEDIUM");
        while (digitalRead(BTN_MEDIUM) == LOW);
      }
    }
    if (digitalRead(BTN_FAST) == LOW) {
      delay(50);
      if (digitalRead(BTN_FAST) == LOW) {
        currentSpeed = FAST;
        Serial.println("Speed: FAST");
        while (digitalRead(BTN_FAST) == LOW);
      }
    }

    // Start button
    if (digitalRead(BTN_START) == LOW) {
      delay(50);
      if (digitalRead(BTN_START) == LOW) {
        Serial.println("Starting sequence...");
        sequenceActive = true;
        stopPressed    = false;
        isPaused       = false;
        currentPass    = 0;
        passGoRight    = true;
        enableMotors();
        homingSequence();
        runScrubSequence();
        while (digitalRead(BTN_START) == LOW);
      }
    }
  }

  // Stop button always monitored
  if (digitalRead(BTN_STOP) == LOW) {
    delay(50);
    if (digitalRead(BTN_STOP) == LOW) {
      handleStop();
      while (digitalRead(BTN_STOP) == LOW);
    }
  }
}

// ============================================================
//  STOP HANDLER
//  First press → pause
//  Second press (stop again) → resume
//  Any other button after stop → return home, restart
// ============================================================
void handleStop() {
  if (!sequenceActive) return;

  if (!isPaused) {
    // First stop press — pause
    isPaused    = true;
    stopPressed = true;
    pumpOff();
    disableMotors();
    Serial.println("PAUSED. Press Stop again to resume, or Speed/Start to restart from home.");

    // Wait for next button press
    bool resolved = false;
    while (!resolved) {
      if (digitalRead(BTN_STOP) == LOW) {
        delay(50);
        if (digitalRead(BTN_STOP) == LOW) {
          // Resume
          isPaused    = false;
          stopPressed = false;
          enableMotors();
          Serial.println("Resuming...");
          resolved = true;
          while (digitalRead(BTN_STOP) == LOW);
        }
      }
      // Any other button → return home and restart
      if (digitalRead(BTN_START)  == LOW ||
          digitalRead(BTN_SLOW)   == LOW ||
          digitalRead(BTN_MEDIUM) == LOW ||
          digitalRead(BTN_FAST)   == LOW) {
        delay(50);
        Serial.println("Returning to home and restarting...");
        isPaused       = false;
        stopPressed    = false;
        currentPass    = 0;
        passGoRight    = true;
        enableMotors();
        homingSequence();
        resolved = true;
        // Consume button press
        while (digitalRead(BTN_START)  == LOW ||
               digitalRead(BTN_SLOW)   == LOW ||
               digitalRead(BTN_MEDIUM) == LOW ||
               digitalRead(BTN_FAST)   == LOW);
      }
    }
  }
}

// ============================================================
//  SCRUB SEQUENCE
// ============================================================
void runScrubSequence() {
  for (; currentPass < NUM_PASSES; currentPass++) {
    // Check if paused before each pass
    if (isPaused) return;

    passGoRight = (currentPass % 2 == 0);
    Serial.print("Pass ");
    Serial.print(currentPass + 1);
    Serial.println(passGoRight ? " → right" : " ← left");

    pumpOn();
    bool completed = moveMotor(H_PUL, H_DIR, H_STEPS,
                               passGoRight ? H_RIGHT : H_LEFT,
                               hDelay(), true);  // true = check limits
    pumpOff();

    if (!completed || isPaused) return;  // stopped mid-pass

    delay(300);

    // Vertical step (skip after last pass)
    if (currentPass < NUM_PASSES - 1) {
      Serial.println("Stepping up 4 inches...");
      completed = moveMotor(V_PUL, V_DIR, V_STEP_STEPS,
                            V_UP, vDelay(), true);
      if (!completed || isPaused) return;
      delay(300);
    }
  }

  // All passes done
  Serial.println("Sequence complete. Returning home.");
  pumpOff();
  homingSequence();
  disableMotors();
  sequenceActive = false;
  currentPass    = 0;
  Serial.println("Done. Ready for next cycle.");
}

// ============================================================
//  HOMING SEQUENCE
//  Drives to limit switches to find home (bottom-left)
// ============================================================
void homingSequence() {
  Serial.println("Homing horizontal...");

  // Home horizontal to LEFT
  digitalWrite(H_DIR, H_LEFT);
  long safety = 0;
  long maxH   = (long)(H_TRAVEL_MM * STEPS_PER_MM * 1.2);

  while (digitalRead(LIMIT_H_LEFT) == LOW && safety < maxH) {
    // NC switch: normally LOW, goes HIGH when triggered
    // So we drive until it reads HIGH
    digitalWrite(H_PUL, HIGH);
    delayMicroseconds(HOME_DELAY);
    digitalWrite(H_PUL, LOW);
    delayMicroseconds(HOME_DELAY);
    safety++;
  }
  delay(200);

  Serial.println("Homing vertical...");

  // Home vertical to BOTTOM
  digitalWrite(V_DIR, V_DOWN);
  safety      = 0;
  long maxV   = (long)(609.6 * STEPS_PER_MM * 1.2);

  while (digitalRead(LIMIT_V_BOTTOM) == HIGH && safety < maxV) {
    // NO switch: normally HIGH, goes LOW when triggered
    // So we drive until it reads LOW
    digitalWrite(V_PUL, HIGH);
    delayMicroseconds(HOME_DELAY);
    digitalWrite(V_PUL, LOW);
    delayMicroseconds(HOME_DELAY);
    safety++;
  }
  delay(200);

  Serial.println("Homing complete.");
}

// ============================================================
//  MOVE MOTOR
//  Returns true if completed fully, false if stopped early
//  checkLimits: true = stop if limit switch hit mid-move
// ============================================================
bool moveMotor(int pulPin, int dirPin, long steps, bool direction,
               unsigned int pulseDelayUs, bool checkLimits) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(5);

  long         rampSteps   = min(ACCEL_STEPS, steps / 2);
  unsigned int slowDelay   = pulseDelayUs * 3;

  for (long i = 0; i < steps; i++) {

    // Check stop button every 100 steps
    if (i % 100 == 0) {
      if (digitalRead(BTN_STOP) == LOW) {
        delay(50);
        if (digitalRead(BTN_STOP) == LOW) {
          handleStop();
          if (isPaused) return false;
        }
      }
    }

    // Check limit switches if requested
    if (checkLimits) {
      // NC horizontal limits — triggered when HIGH
      if (digitalRead(LIMIT_H_LEFT)  == HIGH && direction == H_LEFT)  return false;
      if (digitalRead(LIMIT_H_RIGHT) == HIGH && direction == H_RIGHT) return false;
      // NO vertical limits — triggered when LOW
      if (digitalRead(LIMIT_V_TOP)    == LOW && direction == V_UP)   return false;
      if (digitalRead(LIMIT_V_BOTTOM) == LOW && direction == V_DOWN) return false;
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

  return true;
}

// ============================================================
//  MOTOR ENABLE / DISABLE
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
//  PUMP CONTROL
// ============================================================
void pumpOn() {
  digitalWrite(PUMP_PIN, HIGH);
  Serial.println("Pump ON");
}

void pumpOff() {
  digitalWrite(PUMP_PIN, LOW);
  Serial.println("Pump OFF");
}
