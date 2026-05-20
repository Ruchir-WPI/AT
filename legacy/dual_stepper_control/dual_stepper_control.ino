// ============================================================
// Dual Stepper Motor Control - Arduino Uno R4 Minima
// Motors: 17HE08-1004S + 17HS08-1004S (NEMA 17, 1A, 1.8deg)
// Drivers: ZK-SMC02 x2 (in Setup/External Control Mode)
// Battery: E-flite 3S 11.1V LiPo
// ============================================================
//
// WIRING:
//   Motor 1 (ZK-SMC02 #1):
//     PUL → D2
//     DIR → D3
//     EN  → D4
//     GND → Arduino GND (common ground!)
//
//   Motor 2 (ZK-SMC02 #2):
//     PUL → D5
//     DIR → D6
//     EN  → D7
//     GND → Arduino GND (common ground!)
//
//   Power:
//     Battery 11.1V → Screw terminal block → ZK-SMC02 #1 and #2 VCC/GND
//     Battery → 5V Buck Converter → Arduino 5V pin
//
// ZK-SMC02 SETUP:
//   - Set to "Setup Control Mode" (P03) to accept external PUL/DIR signals
//   - Set current limit potentiometer to 1.0A (motor rated current)
//   - Set subdivision via DIP switches (suggest 8x microstepping)
//
// LIBRARY REQUIRED:
//   AccelStepper by Mike McCaulay
//   Install via Arduino IDE: Sketch > Include Library > Manage Libraries
//   Search "AccelStepper" and install
// ============================================================

#include <AccelStepper.h>

// --- Pin Definitions ---
#define MOTOR1_PUL 2
#define MOTOR1_DIR 3
#define MOTOR1_EN 4

#define MOTOR2_PUL 5
#define MOTOR2_DIR 6
#define MOTOR2_EN 7

// --- Motor Configuration ---
// Steps per revolution = (360 / step_angle) * microstepping
// 1.8deg motor = 200 full steps. At 8x microstepping = 1600 steps/rev
#define STEPS_PER_REV 1600

// Target speed in RPM — set to 30 RPM as discussed
#define TARGET_RPM 30

// Convert RPM to steps per second for AccelStepper
// steps/sec = (RPM * steps_per_rev) / 60
#define STEPS_PER_SEC ((TARGET_RPM * STEPS_PER_REV) / 60.0)

// Acceleration in steps/sec^2 — smooth ramp up/down
#define ACCELERATION 500

// --- AccelStepper instances ---
// DRIVER mode: uses separate step and direction pins
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_PUL, MOTOR1_DIR);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_PUL, MOTOR2_DIR);

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Dual Stepper Control - Starting up...");

  // Configure enable pins (LOW = enabled on most drivers)
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);

  // Enable both drivers
  enableMotors();

  // --- Motor 1 configuration ---
  motor1.setMaxSpeed(STEPS_PER_SEC);
  motor1.setAcceleration(ACCELERATION);
  motor1.setCurrentPosition(0);

  // --- Motor 2 configuration ---
  motor2.setMaxSpeed(STEPS_PER_SEC);
  motor2.setAcceleration(ACCELERATION);
  motor2.setCurrentPosition(0);

  Serial.println("Motors ready.");
  Serial.print("Target speed: ");
  Serial.print(TARGET_RPM);
  Serial.println(" RPM");
  Serial.print("Steps/sec: ");
  Serial.println(STEPS_PER_SEC);
}

// ============================================================
// MAIN LOOP — Edit this section for your sequences
// ============================================================
void loop() {

  // --- EXAMPLE SEQUENCE ---
  // Motor 1: rotate forward 5 full revolutions
  // Motor 2: rotate backward 5 full revolutions
  // Then reverse both and repeat

  Serial.println("Running sequence: Forward 5 revolutions...");
  runBothMotors(5 * STEPS_PER_REV, -(5 * STEPS_PER_REV));
  waitForBothMotors();

  delay(1000);  // pause 1 second between sequences

  Serial.println("Running sequence: Reverse 5 revolutions...");
  runBothMotors(-(5 * STEPS_PER_REV), 5 * STEPS_PER_REV);
  waitForBothMotors();

  delay(1000);
}

// ============================================================
// HELPER FUNCTIONS
// ============================================================

// Move both motors to a target position (relative to current)
// motor1Steps: steps for motor 1 (positive = forward, negative = reverse)
// motor2Steps: steps for motor 2
void runBothMotors(long motor1Steps, long motor2Steps) {
  motor1.move(motor1Steps);
  motor2.move(motor2Steps);
}

// Block until both motors have reached their target position
// motor.run() MUST be called as fast as possible — no delays inside this
void waitForBothMotors() {
  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
  }
}

// Enable both motor drivers (LOW signal = enabled)
void enableMotors() {
  digitalWrite(MOTOR1_EN, LOW);
  digitalWrite(MOTOR2_EN, LOW);
  Serial.println("Motors enabled.");
}

// Disable both motor drivers — motors will freewheel
// Call this when done to reduce heat during idle
void disableMotors() {
  digitalWrite(MOTOR1_EN, HIGH);
  digitalWrite(MOTOR2_EN, HIGH);
  Serial.println("Motors disabled.");
}

// Set speed for motor 1 in RPM (can call mid-sequence)
void setMotor1RPM(float rpm) {
  float stepsPerSec = (rpm * STEPS_PER_REV) / 60.0;
  motor1.setMaxSpeed(stepsPerSec);
}

// Set speed for motor 2 in RPM (can call mid-sequence)
void setMotor2RPM(float rpm) {
  float stepsPerSec = (rpm * STEPS_PER_REV) / 60.0;
  motor2.setMaxSpeed(stepsPerSec);
}

// Spin motor 1 continuously in a direction (call repeatedly in loop)
// direction: 1 = forward, -1 = reverse
void spinMotor1Continuous(int direction) {
  motor1.setSpeed(direction * STEPS_PER_SEC);
  motor1.runSpeed();
}

// Spin motor 2 continuously in a direction (call repeatedly in loop)
void spinMotor2Continuous(int direction) {
  motor2.setSpeed(direction * STEPS_PER_SEC);
  motor2.runSpeed();
}
