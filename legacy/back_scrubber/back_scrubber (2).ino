// ============================================================
//  Back Scrubber Gantry Controller
//  2x ZK-SMC02 (PUL/DIR/EN interface) + Arduino
//
//  Axes:
//    HORIZONTAL  — brush travels 24 in (609.6 mm) side to side
//    VERTICAL    — gantry steps 4 in (101.6 mm) per row
//
//  Sequence:
//    Pass 1 → (right)
//    Step up 4 in
//    Pass 2 ← (left)
//    Step up 4 in
//    ... repeat for 6 passes (covers 24 in vertically)
//    Return to home (bottom-left)
// ============================================================

// ---- PIN ASSIGNMENTS ----------------------------------------
// Horizontal motor (driver 1 — bottom board extended pins)
#define H_PUL  2
#define H_DIR  3
#define H_EN   4

// Vertical motor (driver 2 — bottom board extended pins)
#define V_PUL  5
#define V_DIR  6
#define V_EN   7

// Optional: start button (momentary, pulls LOW)
#define START_BTN 8

// Optional: limit switches (NC = normally closed, pulls LOW when triggered)
#define LIMIT_H_LEFT  9
#define LIMIT_H_RIGHT 10
#define LIMIT_V_BOTTOM 11

// ---- MOTION PARAMETERS --------------------------------------
// Microstepping: 16x → 3200 steps/rev
// Lead screw pitch: 8 mm/rev
// Resolution: 8 mm / 3200 steps = 0.0025 mm/step

const int   STEPS_PER_REV      = 3200;     // 16x microstepping
const float PITCH_MM           = 8.0;      // mm per revolution
const float STEPS_PER_MM       = STEPS_PER_REV / PITCH_MM; // 400 steps/mm

// Travel distances
const float H_TRAVEL_MM        = 609.6;    // 24 inches horizontal
const float V_STEP_MM          = 101.6;    // 4 inches vertical per row
const float V_TOTAL_MM         = 609.6;    // 24 inches total vertical (but only 6 steps used)

// Computed step counts
const long  H_STEPS            = (long)(H_TRAVEL_MM * STEPS_PER_MM);  // 243,840
const long  V_STEP_STEPS       = (long)(V_STEP_MM   * STEPS_PER_MM);  // 40,640

// Number of horizontal passes
// 26 in vertical / 4 in brush = 6.5 → use 6 full passes + slight overlap
const int   NUM_PASSES         = 6;

// Speed settings
// Horizontal brush travel: 15 RPM → 2 mm/s → pulse delay = 1/(400 steps/mm * 2 mm/s) = 1250 us
// Vertical step:           30 RPM → 4 mm/s → pulse delay = 1/(400 steps/mm * 4 mm/s) =  625 us
// Homing (faster):         60 RPM → 8 mm/s → pulse delay =                               312 us

const unsigned int H_PULSE_DELAY_US   = 1250;  // microseconds between pulse edges (15 RPM)
const unsigned int V_PULSE_DELAY_US   =  625;  // microseconds between pulse edges (30 RPM)
const unsigned int HOME_PULSE_DELAY_US =  312;  // microseconds between pulse edges (60 RPM)

// Acceleration: number of steps to ramp up/down speed
// Prevents jerking at move start/end
const long  ACCEL_STEPS        = 400;      // ~1 mm ramp distance

// Direction logic (adjust if your motor spins the wrong way — just flip the bool)
#define H_RIGHT  HIGH
#define H_LEFT   LOW
#define V_UP     HIGH
#define V_DOWN   LOW

// ---- STATE --------------------------------------------------
bool hAtRight = false;   // track current horizontal position

// ---- FUNCTION PROTOTYPES ------------------------------------
void moveMotor(int pulPin, int dirPin, long steps, bool direction, unsigned int pulseDelayUs);
void homingSequence();
void runScrubSequence();
void enableMotors();
void disableMotors();

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(H_PUL, OUTPUT);
  pinMode(H_DIR, OUTPUT);
  pinMode(H_EN,  OUTPUT);
  pinMode(V_PUL, OUTPUT);
  pinMode(V_DIR, OUTPUT);
  pinMode(V_EN,  OUTPUT);

  // Start button
  pinMode(START_BTN, INPUT_PULLUP);

  // Limit switches (optional — comment out if not installed)
  pinMode(LIMIT_H_LEFT,   INPUT_PULLUP);
  pinMode(LIMIT_H_RIGHT,  INPUT_PULLUP);
  pinMode(LIMIT_V_BOTTOM, INPUT_PULLUP);

  // Motors disabled until needed
  disableMotors();

  Serial.println("Back scrubber ready. Press start button to begin.");
}

// =============================================================
//  MAIN LOOP
// =============================================================
void loop() {
  // Wait for start button press (active LOW)
  if (digitalRead(START_BTN) == LOW) {
    delay(50); // debounce
    if (digitalRead(START_BTN) == LOW) {
      Serial.println("Starting sequence...");
      enableMotors();
      homingSequence();       // go to known home position first
      runScrubSequence();     // execute all passes
      homingSequence();       // return to home when done
      disableMotors();
      Serial.println("Sequence complete.");
    }
  }
}

// =============================================================
//  HOMING SEQUENCE
//  Drives both axes toward limit switches to find home (bottom-left)
//  If you don't have limit switches, replace with fixed-step blind home
// =============================================================
void homingSequence() {
  Serial.println("Homing...");
  enableMotors();

  // ---- Home horizontal to LEFT ----
  // Drive left until limit switch triggers (or max steps reached)
  digitalWrite(H_DIR, H_LEFT);
  long safetyCount = 0;
  long maxHomingSteps = (long)(H_TRAVEL_MM * STEPS_PER_MM * 1.2); // 20% overtravel safety

  while (digitalRead(LIMIT_H_LEFT) == HIGH && safetyCount < maxHomingSteps) {
    digitalWrite(H_PUL, HIGH);
    delayMicroseconds(HOME_PULSE_DELAY_US);
    digitalWrite(H_PUL, LOW);
    delayMicroseconds(HOME_PULSE_DELAY_US);
    safetyCount++;
  }
  hAtRight = false;
  Serial.println("Horizontal homed.");

  delay(200);

  // ---- Home vertical to BOTTOM ----
  digitalWrite(V_DIR, V_DOWN);
  safetyCount = 0;
  long maxVHomingSteps = (long)(V_TOTAL_MM * STEPS_PER_MM * 1.2);

  while (digitalRead(LIMIT_V_BOTTOM) == HIGH && safetyCount < maxVHomingSteps) {
    digitalWrite(V_PUL, HIGH);
    delayMicroseconds(HOME_PULSE_DELAY_US);
    digitalWrite(V_PUL, LOW);
    delayMicroseconds(HOME_PULSE_DELAY_US);
    safetyCount++;
  }
  Serial.println("Vertical homed.");

  delay(200);
}

// =============================================================
//  SCRUB SEQUENCE
//  Alternating horizontal passes with vertical steps between them
// =============================================================
void runScrubSequence() {
  Serial.println("Starting scrub sequence.");

  for (int pass = 0; pass < NUM_PASSES; pass++) {

    // Determine direction: even passes go right, odd go left
    bool goRight = (pass % 2 == 0);

    Serial.print("Pass ");
    Serial.print(pass + 1);
    Serial.println(goRight ? " → (right)" : " ← (left)");

    // Execute horizontal pass
    moveMotor(H_PUL, H_DIR, H_STEPS, goRight ? H_RIGHT : H_LEFT, H_PULSE_DELAY_US);
    hAtRight = goRight;

    delay(300); // brief pause at end of pass

    // Step vertically up (except after the last pass)
    if (pass < NUM_PASSES - 1) {
      Serial.println("Stepping up 4 inches...");
      moveMotor(V_PUL, V_DIR, V_STEP_STEPS, V_UP, V_PULSE_DELAY_US);
      delay(300);
    }
  }

  Serial.println("All passes complete.");
}

// =============================================================
//  MOVE MOTOR — with trapezoidal acceleration
//  pulPin, dirPin : which motor
//  steps          : total steps to move
//  direction      : HIGH or LOW
//  pulseDelayUs   : target (cruise) pulse delay in microseconds
// =============================================================
void moveMotor(int pulPin, int dirPin, long steps, bool direction, unsigned int pulseDelayUs) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(5); // direction setup time

  // Clamp accel ramp to half total steps (avoid overshoot on short moves)
  long rampSteps = min(ACCEL_STEPS, steps / 2);

  // Slowest speed for accel start (3x cruise delay)
  unsigned int slowDelay = pulseDelayUs * 3;

  for (long i = 0; i < steps; i++) {
    unsigned int currentDelay;

    if (i < rampSteps) {
      // Accelerating: interpolate from slowDelay down to pulseDelayUs
      currentDelay = slowDelay - (unsigned int)((slowDelay - pulseDelayUs) * i / rampSteps);
    } else if (i >= steps - rampSteps) {
      // Decelerating: interpolate from pulseDelayUs back up to slowDelay
      long stepsFromEnd = steps - i;
      currentDelay = slowDelay - (unsigned int)((slowDelay - pulseDelayUs) * stepsFromEnd / rampSteps);
    } else {
      // Cruise
      currentDelay = pulseDelayUs;
    }

    // Optional: check limit switches mid-move for safety
    // if (digitalRead(LIMIT_H_LEFT) == LOW || digitalRead(LIMIT_H_RIGHT) == LOW) break;

    digitalWrite(pulPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(currentDelay);
  }
}

// =============================================================
//  ENABLE / DISABLE MOTORS
//  EN pin: LOW = enabled, HIGH = disabled (motors free to turn)
// =============================================================
void enableMotors() {
  digitalWrite(H_EN, LOW);
  digitalWrite(V_EN, LOW);
  delayMicroseconds(200); // allow drivers to wake up
}

void disableMotors() {
  digitalWrite(H_EN, HIGH);
  digitalWrite(V_EN, HIGH);
}
