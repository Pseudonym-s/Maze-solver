// ==================== BASIC LINE FOLLOWER ROBOT (LFR1) ====================
// Simplified version with essential features
// ==========================================================================

// ==================== CONFIGURATION ====================
#define NUM_SENSORS 8
#define LINE_THRESHOLD 250

// Speed Settings
#define BASE_SPEED 70
#define MAX_SPEED 100
#define TURN_SPEED 60

// PID Constants
#define KP 0.04
#define KI 0.0
#define KD 0.06

// ==================== PINOUT ====================
// Motor Driver Pins (TB6612FNG)
#define PWMA 5
#define AIN1 7
#define AIN2 6

#define PWMB 11
#define BIN1 9
#define BIN2 10

#define STBY 8

// Control Pins
#define START_BUTTON 4
#define LED 12

// ==================== SENSOR CONFIG ====================
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
const long weights[NUM_SENSORS] = {-700, -500, -300, -100, 100, 300, 500, 700};

// ==================== GLOBAL VARIABLES ====================
int sensorValues[NUM_SENSORS];
bool running = false;
float lastError = 0.0;
float integral = 0.0;

// ==================== MOTOR CONTROL ====================
void setMotor(int in1, int in2, int pwmPin, int speed) {
  int magnitude = constrain(abs(speed), 0, MAX_SPEED);
  bool forward = (speed >= 0);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwmPin, magnitude);
}

void setMotors(int left, int right) {
  digitalWrite(STBY, HIGH);
  setMotor(AIN1, AIN2, PWMA, left);
  setMotor(BIN1, BIN2, PWMB, right);
}

void stopMotors() {
  setMotors(0, 0);
  digitalWrite(STBY, LOW);
}

// ==================== SENSOR READING ====================
float readLinePosition() {
  long weightedSum = 0;
  int onLineCount = 0;

  // Read all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // Calculate weighted position
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > LINE_THRESHOLD) {
      weightedSum += weights[i];
      onLineCount++;
    }
  }

  // Return error (0 = centered, negative = left, positive = right)
  if (onLineCount == 0) {
    return lastError * 1000; // Lost line, use last known direction
  }

  return (float)weightedSum / (float)onLineCount;
}

// ==================== LINE FOLLOWING ====================
void followLine() {
  float error = readLinePosition();
  
  // PID calculation
  float proportional = error;
  integral += error;
  integral = constrain(integral, -5000, 5000);
  float derivative = error - lastError;
  
  float correction = (KP * proportional) + (KI * integral) + (KD * derivative);
  correction = constrain(correction, -MAX_SPEED, MAX_SPEED);
  
  lastError = error;
  
  // Calculate motor speeds
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;
  
  // Constrain and apply
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  setMotors(leftSpeed, rightSpeed);
}

// ==================== BUTTON CONTROL ====================
void checkButton() {
  static bool lastButtonState = HIGH;
  bool currentState = digitalRead(START_BUTTON);

  if (currentState == LOW && lastButtonState == HIGH) {
    running = !running;
    
    if (running) {
      digitalWrite(LED, HIGH);
      integral = 0;
      lastError = 0;
    } else {
      stopMotors();
      digitalWrite(LED, LOW);
    }
    
    delay(300); // Debounce
  }
  
  lastButtonState = currentState;
}

// ==================== SETUP ====================
void setup() {
  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Control pins
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  
  // Initialize
  digitalWrite(STBY, HIGH);
  digitalWrite(LED, LOW);
  
  Serial.begin(115200);
  Serial.println("LFR1 - Basic Line Follower Ready");
}

// ==================== MAIN LOOP ====================
void loop() {
  checkButton();
  
  if (running) {
    followLine();
  } else {
    stopMotors();
    delay(50);
  }
}
