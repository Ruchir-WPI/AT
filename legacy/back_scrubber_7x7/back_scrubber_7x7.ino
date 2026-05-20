// ============================================================
//  Back Scrubber — 7x7 inch test area
//  Starts top-right, 7 inches horizontal, 7 inches vertical
//  4x microstepping
// ============================================================

// ---- PIN ASSIGNMENTS ----------------------------------------
#define H_PUL       2
#define H_DIR       3
#define H_EN        4

#define V_PUL       5
#define V_DIR       6
#define V_EN        7

#define PUMP_PIN    13

#define BTN_SLOW    A0
#define BTN_MEDIUM  A1
#define BTN_FAST    A2
#define BTN_STOP    A3

// ---- MOTION PARAMETERS — 4x microstepping -------------------
const int   STEPS_PER_REV  = 800;
const float PITCH_MM       = 8.0;
const float STEPS_PER_MM   = STEPS_PER_REV / PITCH_MM;  // 100 steps/mm

const float H_TRAVEL_MM    = 177.8;   // 7 inches
const float V_STEP_MM      = 101.6;   // 4 inches per step
const float V_TOTAL_MM     = 177.8;   // 7 inches total vertical

const long  H_STEPS        = (long)(H_TRAVEL_MM * STEPS_PER_MM);  // 17,780
const long  V_STEP_STEPS   = (long)(V_STEP_MM   * STEPS_PER_MM);  // 10,160
const long  V_TOTAL_STEPS  = (long)(V_TOTAL_MM  * STEPS_PER_MM);  // 17,780

// 7 inches / 4 inch brush = 1 full pass + remainder, so 2 passes
const int   NUM_PASSES     = 2;
const long  ACCEL_STEPS    = 100;

// Speed presets (pulse delay in microseconds)
const unsigned int H_SLOW   = 1240;
const unsigned int H_MEDIUM = 928;
const unsigned int H_FAST   = 620;

const unsigned int V_SLOW   = 2500;  // 30 RPM
const unsigned int V_MEDIUM = 1666;  // 45 RPM
const unsigned int V_FAST   = 1250;  // 60 RPM

// Starts top-right, so first pass goes LEFT
#define H_RIGHT  LOW
#define H_LEFT   HIGH
#define V_DOWN   HIGH
#define V_UP     LOW

// ---- STATE --------------------------------------------------
enum SpeedMode { SLOW, MEDIUM, FAST };
SpeedMode currentSpeed = SLOW;

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

bool isPaused       = false;
bool sequenceActive = false;
bool stopFlag       = false;
int  currentPass    = 0;
bool passGoRight    = false;  // first pass goes left

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

  pinMode(BTN_SLOW,   INPUT_PULLUP);
  pinMode(BTN_MEDIUM, INPUT_PULLUP);
  pinMode(BTN_FAST,   INPUT_PULLUP);
  pinMode(BTN_STOP,   INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Ready. Press Slow, Medium, or Fast to begin.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  if (!sequenceActive) {
    if (digitalRead(BTN_SLOW) == LOW) {
      delay(50);
      if (digitalRead(BTN_SLOW) == LOW) {
        currentSpeed = SLOW;
        Serial.println("Speed: SLOW — Starting...");
        startSequence();
        while (digitalRead(BTN_SLOW) == LOW);
      }
    }
    if (digitalRead(BTN_MEDIUM) == LOW) {
      delay(50);
      if (digitalRead(BTN_MEDIUM) == LOW) {
        currentSpeed = MEDIUM;
        Serial.println("Speed: MEDIUM — Starting...");
        startSequence();
        while (digitalRead(BTN_MEDIUM) == LOW);
      }
    }
    if (digitalRead(BTN_FAST) == LOW) {
      delay(50);
      if (digitalRead(BTN_FAST) == LOW) {
        currentSpeed = FAST;
        Serial.println("Speed: FAST — Starting...");
        startSequence();
        while (digitalRead(BTN_FAST) == LOW);
      }
    }
  }
}

// ============================================================
//  START SEQUENCE
// ============================================================
void startSequence() {
  sequenceActive = true;
  isPaused       = false;
  stopFlag       = false;
  currentPass    = 0;
  passGoRight    = false;  // first pass goes left from top-right
  enableMotors();
  runScrubSequence();
}

// ============================================================
//  SCRUB SEQUENCE
// ============================================================
void runScrubSequence() {
  for (; currentPass < NUM_PASSES; currentPass++) {
    if (checkStop()) return;

    Serial.print("Pass ");
    Serial.print(currentPass + 1);
    Serial.println(passGoRight ? " -> right" : " <- left");

    pumpOn();
    bool completed = moveMotor(H_PUL, H_DIR, H_STEPS,
                               passGoRight ? H_RIGHT : H_LEFT,
                               hDelay());
    pumpOff();

    if (!completed) return;

    delay(300);

    if (currentPass < NUM_PASSES - 1) {
      Serial.println("Stepping down 4 inches...");
      completed = moveMotor(V_PUL, V_DIR, V_STEP_STEPS, V_DOWN, vDelay());
      if (!completed) return;
      delay(300);
    }

    passGoRight = !passGoRight;  // alternate direction each pass
  }

  Serial.println("Sequence complete.");
  pumpOff();
  disableMotors();
  sequenceActive = false;
  currentPass    = 0;
  Serial.println("Done. Press a speed button to run again.");
}

// ============================================================
//  CHECK STOP
// ============================================================
bool checkStop() {
  if (digitalRead(BTN_STOP) == LOW) {
    delay(50);
    if (digitalRead(BTN_STOP) == LOW) {
      while (digitalRead(BTN_STOP) == LOW);
      handleStop();
      return isPaused || !sequenceActive;
    }
  }
  return false;
}

// ============================================================
//  STOP HANDLER
// ============================================================
void handleStop() {
  isPaused = true;
  stopFlag = true;
  pumpOff();
  disableMotors();
  Serial.println("PAUSED. Press Stop to resume, or a speed button to restart.");

  bool resolved = false;
  while (!resolved) {
    if (digitalRead(BTN_STOP) == LOW) {
      delay(50);
      if (digitalRead(BTN_STOP) == LOW) {
        isPaused = false;
        stopFlag = false;
        enableMotors();
        Serial.println("Resuming...");
        resolved = true;
        while (digitalRead(BTN_STOP) == LOW);
        runScrubSequence();
      }
    }
    if (digitalRead(BTN_SLOW) == LOW) {
      delay(50);
      if (digitalRead(BTN_SLOW) == LOW) {
        currentSpeed = SLOW;
        resolved = true;
        while (digitalRead(BTN_SLOW) == LOW);
        restartSequence();
      }
    }
    if (digitalRead(BTN_MEDIUM) == LOW) {
      delay(50);
      if (digitalRead(BTN_MEDIUM) == LOW) {
        currentSpeed = MEDIUM;
        resolved = true;
        while (digitalRead(BTN_MEDIUM) == LOW);
        restartSequence();
      }
    }
    if (digitalRead(BTN_FAST) == LOW) {
      delay(50);
      if (digitalRead(BTN_FAST) == LOW) {
        currentSpeed = FAST;
        resolved = true;
        while (digitalRead(BTN_FAST) == LOW);
        restartSequence();
      }
    }
  }
}

// ============================================================
//  RESTART
// ============================================================
void restartSequence() {
  isPaused    = false;
  stopFlag    = false;
  currentPass = 0;
  passGoRight = false;  // always start going left from top-right
  enableMotors();
  Serial.println("Restarting...");
  runScrubSequence();
}

// ============================================================
//  MOVE MOTOR
// ============================================================
bool moveMotor(int pulPin, int dirPin, long steps, bool direction,
               unsigned int pulseDelayUs) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(5);

  long         rampSteps = min(ACCEL_STEPS, steps / 2);
  unsigned int slowDelay = pulseDelayUs * 3;

  for (long i = 0; i < steps; i++) {
    if (i % 10 == 0) {
      if (digitalRead(BTN_STOP) == LOW) {
        delay(50);
        if (digitalRead(BTN_STOP) == LOW) {
          while (digitalRead(BTN_STOP) == LOW);
          handleStop();
          return false;
        }
      }
    }

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
