// ==================== HYBRID LINE FOLLOWER ROBOT (8 SENSORS) ====================
// Features: State Machine + PID + Anti-Backtracking + Updated Pinout
// =================================================================================

// ==================== CONFIGURATION ====================
#define NUM_SENSORS 8
#define LINE_THRESHOLD 250

// Speed Settings
#define HIGH_SPEED 85            // was 135
#define BASE_SPEED  50             // was 120
#define CORNER_SPEED 60             // was 110
#define CORRECTION_SPEED 50         // was 60
#define PIVOT_SPEED   95            // was 85
#define INTERSECTION_TURN_SPEED 70  // was 85
#define MAX_SPEED 100               // was 140

// Tuning Parameters
#define BIAS_MODE 2      // 0 = no bias, 1 = left bias, 2 = right bias
#define BIAS_SWITCH_PIN -1 // -1 disables runtime toggle
#define BIAS_STRENGTH  4   // Strength of the bias

#define SPEED_SMOOTHING 0.012f   // Acceleration smoothing (0..1)
#define USE_SMOOTHING true     // Enable/disable smoothing

// Thresholds
#define CORNER_ERROR_THRESHOLD 2500     
#define DEADZONE_PID 150                
#define DERIVATIVE_LIMIT 3000           
#define BLINK_RATE 100
      

// ==================== UPDATED PINOUT ====================
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

// ==================== PID Constants ====================
float Kp_corner = 0.036f;
float Ki_corner = 0.00f;
float Kd_corner = 0.058f;

float Kp_straight = 0.0369f;
float Ki_straight = 0.00f;
float Kd_straight = 0.05f;

// ==================== SENSOR CONFIG (8 SENSORS) ====================
// Assumed Layout: Left [A0 A1 A2 A3 | A4 A5 A6 A7] Right
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
const long weights[NUM_SENSORS] = {-750, -650, -550, -250, 250, 550, 650, 750};

// ==================== ROBOT STATES ====================
enum RobotState { STATE_LOST = 0, STATE_LEFT = 1, STATE_RIGHT = 2, STATE_STRAIGHT = 3, STATE_INTERSECTION = 4 };
enum RobotMode  { MODE_STRAIGHT, MODE_CORNER, MODE_INTERSECTION, MODE_LOST };
enum TurnDirection { TURN_NONE = 0, TURN_LEFT = -1, TURN_RIGHT = 1 };

struct TurnRecord {
  TurnDirection direction;
  unsigned long timestamp;
};

// ==================== GLOBAL STATE ====================
int sensorValues[NUM_SENSORS];
bool running = false;
bool correcting = false;
float lastError = 0.0;
float integral = 0.0;
unsigned long lastBlinkTime = 0;
unsigned long lostStartTime = 0;

float currentLeftSpeed = 0.0;
float currentRightSpeed = 0.0;
bool runtimeLeftBias = false;
bool runtimeRightBias = false;


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

void setMotorsSmooth(int targetLeft, int targetRight) {
  if (USE_SMOOTHING) {
    currentLeftSpeed += (targetLeft - currentLeftSpeed) * SPEED_SMOOTHING;
    currentRightSpeed += (targetRight - currentRightSpeed) * SPEED_SMOOTHING;
    setMotors((int)currentLeftSpeed, (int)currentRightSpeed);
  } else {
    setMotors(targetLeft, targetRight);
  }
}

// ==================== LED CONTROL ====================
void updateLED() {
  if (!running) {
    digitalWrite(LED, LOW);
    return;
  }
  if (correcting) {
    if (millis() - lastBlinkTime >= BLINK_RATE) {
      digitalWrite(LED, !digitalRead(LED));
      lastBlinkTime = millis();
    }
  } else {
    digitalWrite(LED, HIGH);
  }
}

// ==================== SENSOR READING ====================
RobotState getRobotState() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[NUM_SENSORS - 1 - i] = analogRead(sensorPins[i]);
  }

  bool center_on =
    (sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD);

  bool left_side_on =
    (sensorValues[0] > LINE_THRESHOLD ||
     sensorValues[1] > LINE_THRESHOLD ||
     sensorValues[2] > LINE_THRESHOLD);

  bool right_side_on =
    (sensorValues[5] > LINE_THRESHOLD ||
     sensorValues[6] > LINE_THRESHOLD ||
     sensorValues[7] > LINE_THRESHOLD);

  // LOST
  if (!center_on && !left_side_on && !right_side_on) {
    return STATE_LOST;
  }

  // ANY intersection (90Â° guaranteed)
  if ((left_side_on && center_on) ||
      (right_side_on && center_on) ||
      (left_side_on && right_side_on)) {
    return STATE_INTERSECTION;
  }

  // Straight corrections (rare with straight-only tracks)
  if (left_side_on && !right_side_on) {
    lastError = -1;
    return STATE_LEFT;
  }

  if (right_side_on && !left_side_on) {
    lastError = 1;
    return STATE_RIGHT;
  }

  return STATE_STRAIGHT;
}

float readLinePosition(RobotMode &currentMode) {
  long weightedSum = 0;
  int onLineCount = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[NUM_SENSORS - 1 - i] = analogRead(sensorPins[i]);
  }

  bool wide_left_on = (sensorValues[0] > LINE_THRESHOLD || sensorValues[1] > LINE_THRESHOLD || sensorValues[2] > LINE_THRESHOLD);
  bool wide_right_on = (sensorValues[5] > LINE_THRESHOLD || sensorValues[6] > LINE_THRESHOLD || sensorValues[7] > LINE_THRESHOLD);

  if (wide_left_on && wide_right_on) {
    currentMode = MODE_INTERSECTION;
    return -5000.0;
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > LINE_THRESHOLD) {
      weightedSum += weights[i];
      onLineCount++;
    }
  }

  if (onLineCount == 0) {
    currentMode = MODE_LOST;
    return 99999;
  }

  float calculatedError = (float)weightedSum / (float)onLineCount;

  if (abs(calculatedError) > CORNER_ERROR_THRESHOLD) currentMode = MODE_CORNER;
  else currentMode = MODE_STRAIGHT;

  return calculatedError;
}

// ==================== MAIN LOGIC ====================
void followLine() {
  RobotState state = getRobotState();
  RobotMode mode = MODE_STRAIGHT;
  float pidError = readLinePosition(mode);

  correcting = true;
  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;

  if (BIAS_SWITCH_PIN >= 0) {
    int s = digitalRead(BIAS_SWITCH_PIN);
    runtimeLeftBias = (s == LOW);
    runtimeRightBias = (s == HIGH);
  }

  // 2. INTERSECTION
  if (state == STATE_INTERSECTION || mode == MODE_INTERSECTION) {
    leftSpeed = -INTERSECTION_TURN_SPEED;
    rightSpeed = INTERSECTION_TURN_SPEED;
    lastError = 1;
    integral = 0;
    setMotors(leftSpeed, rightSpeed);
    return;
  }
  

  // 1. LOST
  if (state == STATE_LOST || mode == MODE_LOST) {
    if (lostStartTime == 0) lostStartTime = millis();
    if (lastError >= 0) { leftSpeed = PIVOT_SPEED; rightSpeed = -PIVOT_SPEED; }
    else { leftSpeed = -PIVOT_SPEED; rightSpeed = PIVOT_SPEED; }
    integral = 0;
    setMotorsSmooth(leftSpeed, rightSpeed);
    return;
  }
  lostStartTime = 0;




  // 3. SHARP LEFT
if (state == STATE_LEFT) {
  leftSpeed = CORRECTION_SPEED;
  rightSpeed = BASE_SPEED;
  lastError = -1;
  correcting = true;
  integral = 0;
  setMotorsSmooth(leftSpeed, rightSpeed);
  return;
}


  // 4. SHARP RIGHT
if (state == STATE_RIGHT) {
  leftSpeed = BASE_SPEED;
  rightSpeed = CORRECTION_SPEED;
  lastError = 1;
  correcting = true;
  integral = 0;
  setMotorsSmooth(leftSpeed, rightSpeed);
  return;
}


  // 5. STRAIGHT PID
  if (state == STATE_STRAIGHT) {
    int baseSpeed;
    float correction = 0.0;
    if (abs(pidError) < DEADZONE_PID) pidError = 0.0;

    float Kp, Ki, Kd;
    if (abs(pidError) > CORNER_ERROR_THRESHOLD || mode == MODE_CORNER) {
      Kp = Kp_corner; Ki = Ki_corner; Kd = Kd_corner;
      baseSpeed = CORNER_SPEED;
    } else {
      Kp = Kp_straight; Ki = Ki_straight; Kd = Kd_straight;
      baseSpeed = HIGH_SPEED;
    }

    if (baseSpeed == CORNER_SPEED) {
      integral += pidError;
      integral = constrain(integral, -10000, 10000);
    } else { integral = 0; }

    float derivative = pidError - lastError;
    derivative = constrain(derivative, -DERIVATIVE_LIMIT, DERIVATIVE_LIMIT);

    correction = (Kp * pidError) + (Ki * integral) + (Kd * derivative);
    correction = constrain(correction, -MAX_SPEED, MAX_SPEED);
    lastError = pidError;

    int biasValue = 0;
    bool leftBiasActive = (BIAS_SWITCH_PIN >= 0) ? runtimeLeftBias : (BIAS_MODE == 1);
    bool rightBiasActive = (BIAS_SWITCH_PIN >= 0) ? runtimeRightBias : (BIAS_MODE == 2);

    int appliedBias = 0;
    if (leftBiasActive && abs(pidError) < (CORNER_ERROR_THRESHOLD / 2)) appliedBias = BIAS_STRENGTH;
    if (rightBiasActive && abs(pidError) < (CORNER_ERROR_THRESHOLD / 2)) appliedBias = -BIAS_STRENGTH;

    float leftF = (float)baseSpeed + correction + (float)appliedBias;
    float rightF = (float)baseSpeed - correction - (float)appliedBias;

    setMotorsSmooth((int)constrain(round(leftF), -MAX_SPEED, MAX_SPEED), (int)constrain(round(rightF), -MAX_SPEED, MAX_SPEED));
    return;
  }
}

// ==================== SETUP & LOOP ====================
void checkButtons() {
  static bool lastButtonState = HIGH;
  bool currentState = digitalRead(START_BUTTON);

  if (currentState == LOW && lastButtonState == HIGH) {
    running = !running;
    if (running) {
      digitalWrite(LED, HIGH);
      integral = 0; lastError = 0; currentLeftSpeed = 0; currentRightSpeed = 0;
    } else {
      setMotors(0, 0); digitalWrite(LED, LOW);
      currentLeftSpeed = 0; currentRightSpeed = 0;
    }
    delay(300);
  }
  lastButtonState = currentState;
}

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  for (int i = 0; i < NUM_SENSORS; i++) pinMode(sensorPins[i], INPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  if (BIAS_SWITCH_PIN >= 0) pinMode(BIAS_SWITCH_PIN, INPUT_PULLUP);

  
  Serial.begin(115200);
  Serial.println("Hybrid LFR (8 Sensors) Configured.");
}

void loop() {
  checkButtons();
  if (running) {
    followLine();
    updateLED();
  } else {
    digitalWrite(LED, LOW);
    delay(50);
  }
}