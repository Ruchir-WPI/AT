// ============================================================
//  Back Scrubber Gantry Controller
//  16x7 inch test area, starts top-left
//  4x microstepping
//
//  Button behavior:
//  SLOW   → reverses horizontal direction immediately
//  MEDIUM → speed control (first press starts, second press pauses)
//  FAST   → fixed speed 310us (first press starts, second press pauses)
//  STOP   → when paused, switches axis (H→V or V→H) and resumes
//  Any button when paused via FAST/MEDIUM double press → resumes
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

const float H_TRAVEL_MM    = 406.4;   // 16 inches
const float V_STEP_MM      = 101.6;   // 4 inches per step
const float V_TOTAL_MM     = 177.8;   // 7 inches total vertical

const long  H_STEPS        = (long)(H_TRAVEL_MM * STEPS_PER_MM);  // 40,640
const long  V_STEP_STEPS   = (long)(V_STEP_MM   * STEPS_PER_MM);  // 10,160
const long  V_TOTAL_STEPS  = (long)(V_TOTAL_MM  * STEPS_PER_MM);  // 17,780

const int   NUM_PASSES     = 2;
const long  ACCEL_STEPS    = 100;

// Speed — only medium is variable, fast is fixed
const unsigned int H_MEDIUM = 232;    // 15s across 16 inches
const unsigned int H_FAST   = 310;    // fixed fast speed
const unsigned int V_MEDIUM = 2000;    // scaled proportionally
const unsigned int V_FAST   = 1000;    // scaled proportionally

#define H_RIGHT  HIGH
#define H_LEFT   LOW
#define V_DOWN   HIGH
#define V_UP     LOW

// ---- STATE --------------------------------------------------
enum SpeedMode { MEDIUM_SPEED, FAST_SPEED };
SpeedMode currentSpeed = FAST_SPEED;

unsigned int hDelay() {
  return (currentSpeed == MEDIUM_SPEED) ? H_MEDIUM : H_FAST;
}
unsigned int vDelay() {
  return (currentSpeed == MEDIUM_SPEED) ? V_MEDIUM : V_FAST;
}

bool isPaused        = false;
bool sequenceActive  = false;
bool stopFlag        = false;
bool doingHorizontal = true;   // track which axis is currently active
int  currentPass     = 0;
bool passGoRight     = false;  // starts top-left, first pass goes right

// For mid-move reversal
volatile bool reverseFlag = false;

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
  Serial.println("Ready. Press Medium or Fast to begin.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  if (!sequenceActive) {
    // MEDIUM — first press starts at medium speed
    if (digitalRead(BTN_MEDIUM) == LOW) {
      delay(50);
      if (digitalRead(BTN_MEDIUM) == LOW) {
        currentSpeed = MEDIUM_SPEED;
        Serial.println("Speed: MEDIUM — Starting...");
        while (digitalRead(BTN_MEDIUM) == LOW);
        startSequence();
      }
    }

    // FAST — first press starts at fast speed
    if (digitalRead(BTN_FAST) == LOW) {
      delay(50);
      if (digitalRead(BTN_FAST) == LOW) {
        currentSpeed = FAST_SPEED;
        Serial.println("Speed: FAST — Starting...");
        while (digitalRead(BTN_FAST) == LOW);
        startSequence();
      }
    }
  }
}

// ============================================================
//  START SEQUENCE
// ============================================================
void startSequence() {
  sequenceActive  = true;
  isPaused        = false;
  stopFlag        = false;
  reverseFlag     = false;
  currentPass     = 0;
  passGoRight     = false;  // top-left, first pass goes right
  doingHorizontal = true;
  enableMotors();
  runScrubSequence();
}

// ============================================================
//  SCRUB SEQUENCE
// ============================================================
void runScrubSequence() {
  for (; currentPass < NUM_PASSES; currentPass++) {
    if (checkButtons()) return;

    doingHorizontal = true;
    Serial.print("Pass ");
    Serial.print(currentPass + 1);
    Serial.println(passGoRight ? " -> right" : " <- left");

    pumpOn();
    bool completed = moveMotor(H_PUL, H_DIR, H_STEPS,
                               passGoRight ? H_RIGHT : H_LEFT,
                               hDelay(), true);
    pumpOff();

    if (!completed) return;

    delay(300);

    if (currentPass < NUM_PASSES - 1) {
      doingHorizontal = false;
      Serial.println("Stepping down 4 inches...");
      completed = moveMotor(V_PUL, V_DIR, V_STEP_STEPS, V_DOWN, vDelay(), false);
      if (!completed) return;
      delay(300);
    }

    passGoRight = !passGoRight;
  }

  Serial.println("Sequence complete.");
  pumpOff();
  disableMotors();
  sequenceActive = false;
  currentPass    = 0;
  Serial.println("Done. Press Medium or Fast to run again.");
}

// ============================================================
//  CHECK BUTTONS — called between major operations
//  Returns true if sequence should pause/stop
// ============================================================
bool checkButtons() {
  // MEDIUM second press — pause
  if (digitalRead(BTN_MEDIUM) == LOW && sequenceActive && currentSpeed == MEDIUM_SPEED) {
    delay(50);
    if (digitalRead(BTN_MEDIUM) == LOW) {
      while (digitalRead(BTN_MEDIUM) == LOW);
      handleDoublePause();
      return isPaused;
    }
  }
  // FAST second press — pause
  if (digitalRead(BTN_FAST) == LOW && sequenceActive && currentSpeed == FAST_SPEED) {
    delay(50);
    if (digitalRead(BTN_FAST) == LOW) {
      while (digitalRead(BTN_FAST) == LOW);
      handleDoublePause();
      return isPaused;
    }
  }
  return false;
}

// ============================================================
//  HANDLE DOUBLE PRESS PAUSE
//  Any button resumes
// ============================================================
void handleDoublePause() {
  isPaused = true;
  pumpOff();
  disableMotors();
  Serial.println("PAUSED. Press any button to resume.");

  while (isPaused) {
    if (digitalRead(BTN_SLOW)   == LOW ||
        digitalRead(BTN_MEDIUM) == LOW ||
        digitalRead(BTN_FAST)   == LOW ||
        digitalRead(BTN_STOP)   == LOW) {
      delay(50);
      if (digitalRead(BTN_SLOW)   == LOW ||
          digitalRead(BTN_MEDIUM) == LOW ||
          digitalRead(BTN_FAST)   == LOW ||
          digitalRead(BTN_STOP)   == LOW) {

        // If stop was pressed, switch axis
        if (digitalRead(BTN_STOP) == LOW) {
          doingHorizontal = !doingHorizontal;
          Serial.println(doingHorizontal ? "Switched to horizontal." : "Switched to vertical.");
        }

        isPaused = false;
        enableMotors();
        Serial.println("Resuming...");

        while (digitalRead(BTN_SLOW)   == LOW ||
               digitalRead(BTN_MEDIUM) == LOW ||
               digitalRead(BTN_FAST)   == LOW ||
               digitalRead(BTN_STOP)   == LOW);

        runScrubSequence();
      }
    }
  }
}

// ============================================================
//  MOVE MOTOR
//  isHorizontal: true = check slow button for reversal
//  Returns false if stopped mid-move
// ============================================================
bool moveMotor(int pulPin, int dirPin, long steps, bool direction,
               unsigned int pulseDelayUs, bool isHorizontal) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(5);

  long         rampSteps = min(ACCEL_STEPS, steps / 2);
  unsigned int slowDelay = pulseDelayUs * 3;
  bool         currentDir = direction;

  for (long i = 0; i < steps; i++) {
    // Check buttons every 10 steps
    if (i % 10 == 0) {
      // SLOW — reverse horizontal direction mid-move
      if (isHorizontal && digitalRead(BTN_SLOW) == LOW) {
        delay(50);
        if (digitalRead(BTN_SLOW) == LOW) {
          while (digitalRead(BTN_SLOW) == LOW);
          currentDir = !currentDir;
          digitalWrite(dirPin, currentDir);
          delayMicroseconds(5);
          Serial.println("Horizontal direction reversed.");
          // Flip remaining steps so it travels back the same distance
          steps = i * 2;
        }
      }

      // MEDIUM second press — pause
      if (digitalRead(BTN_MEDIUM) == LOW && currentSpeed == MEDIUM_SPEED) {
        delay(50);
        if (digitalRead(BTN_MEDIUM) == LOW) {
          while (digitalRead(BTN_MEDIUM) == LOW);
          handleDoublePause();
          return false;
        }
      }

      // FAST second press — pause
      if (digitalRead(BTN_FAST) == LOW && currentSpeed == FAST_SPEED) {
        delay(50);
        if (digitalRead(BTN_FAST) == LOW) {
          while (digitalRead(BTN_FAST) == LOW);
          handleDoublePause();
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
