// ============================================================
//  BACK SCRUBBER - Arduino UNO R4 Minima
//  Two NEMA 17 (42FH382.2-24A) motors via ZK-SMC02 drivers
//  Lead screw: T8 8mm/rev | Full step: 200 steps/rev
//  Resolution: 25 steps/mm
//  No limit switches - assumes carriage starts at top-left
//
//  PIN ASSIGNMENTS:
//    Pin 2  - EN           (both drivers shared, active LOW)
//    Pin 3  - Vertical DIR
//    Pin 4  - Vertical PUL
//    Pin 5  - Horizontal DIR
//    Pin 6  - Horizontal PUL
//    Pin 7  - Start button (momentary, other leg to GND)
//
//  DIRECTION CONSTANTS:
//    Adjust H_DIR_RIGHT / V_DIR_DOWN if a motor runs backwards
//    Swap HIGH <-> LOW for the affected axis only
//
//  MOTION GEOMETRY:
//    Horizontal travel : 16 inches = 406.4mm = 10,160 steps
//    Vertical travel   : 500mm               = 12,500 steps
//    Brush width       : 4 inches = ~102mm
//    Row step distance : 102mm               =  2,550 steps
//    Number of rows    : 5  (covers full 500mm)
//    Pattern           : Boustrophedon (lawnmower)
//                        Row 1: LEFT -> RIGHT
//                        Drop 102mm
//                        Row 2: RIGHT -> LEFT
//                        Drop 102mm
//                        ... repeat for 5 rows
//                        Return to start (top-left)
//
//  SPEED:
//    STEP_DELAY_US = 1000us = ~300 RPM = ~40mm/sec
//    Increase to slow down, decrease to speed up
//    Do not go below 300us or motor may stall
// ============================================================

// --- Pin definitions ---
#define EN_PIN 2;
#define V_DIR_PIN 3;
#define V_STEP_PIN 4;
#define H_DIR_PIN 5;
#define H_STEP_PIN 6;
#define BTN_PIN 7;

// --- Direction constants ---
// If a motor runs the wrong way, swap HIGH and LOW for that axis only
const int H_DIR_RIGHT = HIGH;
const int H_DIR_LEFT  = LOW;
const int V_DIR_DOWN  = HIGH;
const int V_DIR_UP    = LOW;

// --- Motion parameters ---
const long H_TOTAL_STEPS  = 10160;  // 406.4mm * 25 steps/mm
const long V_ROW_STEPS    =  2550;  // 102mm   * 25 steps/mm
const int  NUM_ROWS       =     5;  // ceil(500mm / 102mm)

const unsigned int STEP_DELAY_US = 1000; // microseconds per step (tune to adjust speed)

// --- State ---
bool sequenceRunning = false;

void setup() {
  pinMode(EN_PIN,     OUTPUT);
  pinMode(V_DIR_PIN,  OUTPUT);
  pinMode(V_STEP_PIN, OUTPUT);
  pinMode(H_DIR_PIN,  OUTPUT);
  pinMode(H_STEP_PIN, OUTPUT);
  pinMode(BTN_PIN,    INPUT_PULLUP);

  digitalWrite(EN_PIN, HIGH); // motors disabled at start

  Serial.begin(9600);
  Serial.println("Ready. Press button to start sequence.");
}

void loop() {
  if (digitalRead(BTN_PIN) == LOW && !sequenceRunning) {
    delay(50); // debounce
    if (digitalRead(BTN_PIN) == LOW) {
      sequenceRunning = true;
      runSequence();
      sequenceRunning = false;
    }
  }
}

void runSequence() {
  Serial.println("Sequence started.");
  enableMotors();

  bool goingRight = true;

  for (int row = 0; row < NUM_ROWS; row++) {
    Serial.print("Row ");
    Serial.println(row + 1);

    // Move horizontally across full width
    if (goingRight) {
      moveSteps(H_STEP_PIN, H_DIR_PIN, H_DIR_RIGHT, H_TOTAL_STEPS);
    } else {
      moveSteps(H_STEP_PIN, H_DIR_PIN, H_DIR_LEFT, H_TOTAL_STEPS);
    }

    // Drop down one brush-width, except after the last row
    if (row < NUM_ROWS - 1) {
      moveSteps(V_STEP_PIN, V_DIR_PIN, V_DIR_DOWN, V_ROW_STEPS);
    }

    goingRight = !goingRight;
  }

  // Return to top-left home position
  Serial.println("Returning to start...");
  moveSteps(H_STEP_PIN, H_DIR_PIN, H_DIR_LEFT,  H_TOTAL_STEPS);      // go back left
  moveSteps(V_STEP_PIN, V_DIR_PIN, V_DIR_UP,    V_ROW_STEPS * (NUM_ROWS - 1)); // go back up

  disableMotors();
  Serial.println("Sequence complete. Ready for next press.");
}

void moveSteps(int stepPin, int dirPin, int dir, long steps) {
  digitalWrite(dirPin, dir);
  delayMicroseconds(200); // settle direction signal before pulsing
  for (long i = 0; i < steps; i++) {
    stepOnce(stepPin);
  }
}

void stepOnce(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY_US / 2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US / 2);
}

void enableMotors() {
  digitalWrite(EN_PIN, LOW);
  delay(10);
}

void disableMotors() {
  digitalWrite(EN_PIN, HIGH);
}
