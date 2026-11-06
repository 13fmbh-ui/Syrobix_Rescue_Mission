const int SERVO_CENTER_ANGLE = 125;

#include <Servo.h>

#include "TCS34725Lite.h"

const uint8_t PIN_IR_LEFT   = 2;
const uint8_t PIN_IR_CENTER = 3;
const uint8_t PIN_IR_RIGHT  = 4;

const uint8_t PIN_ENA = 5;
const uint8_t PIN_ENB = 6;
const uint8_t PIN_IN1 = 7;
const uint8_t PIN_IN2 = 8;
const uint8_t PIN_IN3 = 9;
const uint8_t PIN_IN4 = 10;

const uint8_t PIN_SERVO = 13;
const uint8_t PIN_TRIG  = A4;
const uint8_t PIN_ECHO  = A5;

Servo obstacleServo;

const int OBSTACLE_THRESHOLD_CM = 12;

#define OBSTACLE_ENABLED 1

#define SERVO_DETACH_WHEN_IDLE 1

const unsigned long OBSTACLE_CHECK_INTERVAL_MS = 40;
const uint8_t OBSTACLE_CONFIRM_READS = 2;

unsigned long lastObstacleCheck = 0;
uint8_t obstacleHits = 0;

const uint8_t LINE_LEVEL = HIGH;
const uint8_t SENSOR_SAMPLES = 3;
const unsigned int SAMPLE_DELAY_US = 50;

const bool LEFT_MOTORS_REVERSED = false;
const bool RIGHT_MOTORS_REVERSED = false;
const float LEFT_TRIM = 1.00;
const float RIGHT_TRIM = 1.00;
const int MIN_PWM = 60;
const int MAX_PWM = 255;

const int BASE_SPEED = 140;
const int SPEED_DROP_PER_ERROR = 20;
const int SEARCH_SPEED = 170;

const float KP = 240.0;
const float KI = 0.0;
const float KD = 350.0;
const float INTEGRAL_LIMIT = 30.0;

const int ERROR_NEAR = 1;
const int ERROR_FAR = 2;

const unsigned long START_DELAY_MS = 1000;
const unsigned long LOOP_INTERVAL_MS = 5;
const unsigned long GAP_TIMEOUT_MS = 480;
const unsigned long SEARCH_TIMEOUT_MS = 5000;

const unsigned long SWEEP_STEP_MS = 300;

const int CREEP_SPEED = 60;

#define DEBUG 0

TCS34725Lite leftSensor(A0, A1);
TCS34725Lite rightSensor(A2, A3);

const uint8_t COLOR_ATIME = 0xF6;   
const uint8_t COLOR_GAIN  = 0x01;   

const uint16_t GREEN_MIN_CLEAR = 100;    
const float GREEN_MIN_RATIO    = 0.35;   
const float GREEN_MARGIN       = 0.05;   
const uint16_t COLOR_SAT_LIMIT = ((256 - COLOR_ATIME) * 1024UL * 9UL) / 10UL;  

const unsigned long GREEN_CHECK_INTERVAL_MS = 25;    
const unsigned long GREEN_COOLDOWN_MS = 700;         

const unsigned long GREEN_VERIFY_MS = 250;           
const unsigned long GREEN_VERIFY_STOP_MS = 60;       
const int GREEN_VERIFY_CREEP_SPEED = 20;             
                                                     
const uint8_t GREEN_VERIFY_MIN_HITS = 2;             
const unsigned long GREEN_FALSE_ALARM_COOLDOWN_MS = 300;  

const unsigned long BLACK_THEN_GREEN_WINDOW_MS = 400;

const uint16_t BLACK_MAX_CLEAR = 100;

#define USE_IR_FOR_BLACK 1

unsigned long lastBlackColor = 0;   
unsigned long lastBlackIr = 0;      

const unsigned long FORWARD_BEFORE_TURN_MS  = 150;  
const unsigned long FORWARD_BEFORE_UTURN_MS = 0;    
const unsigned long TURN_PAUSE_MS = 50;             
const int TURN_SPEED = 150;                         
const unsigned long TURN_90_MS  = 780;              
const unsigned long TURN_180_MS = 2000;             
const int UTURN_DIRECTION = 1;                      

#define TURN_CALIBRATION 0
#define CALIBRATION_MARKER MARKER_RIGHT
#define COLOR_DEBUG 0
#define COLOR_TEST 0

enum RobotState { FOLLOWING, CROSSING_GAP, SEARCHING, STOPPED,
                  GREEN_CHECKING, GREEN_ADVANCING, GREEN_TURNING, GREEN_UTURN,
                  AVOIDING_OBSTACLE };

RobotState state = FOLLOWING;
int lastError = 0;
int lastDirection = -1;
bool lastSeenCentered = true;
bool allBlack = false;
float integral = 0;
unsigned long lastLoopTime = 0;
unsigned long lostStart = 0;
unsigned long searchStart = 0;

const int MARKER_NONE  = 0;
const int MARKER_LEFT  = 1;
const int MARKER_RIGHT = 2;
const int MARKER_BOTH  = 3;
bool leftColorOk = false;
bool rightColorOk = false;
unsigned long lastGreenCheck = 0;
unsigned long greenIgnoreUntil = 0;
uint8_t greenLeftHits = 0;           
uint8_t greenRightHits = 0;
unsigned long greenStateStart = 0;   
unsigned long greenForwardMs = 0;    
RobotState greenNextState = GREEN_TURNING;  
int greenTurnDir = 1;                

bool readSensor(uint8_t pin) {
  uint8_t hits = 0;
  for (uint8_t i = 0; i < SENSOR_SAMPLES; i++) {
    if (digitalRead(pin) == LINE_LEVEL) hits++;
    delayMicroseconds(SAMPLE_DELAY_US);
  }
  return hits * 2 > SENSOR_SAMPLES;
}

bool readLineError(int &error) {
  bool left = readSensor(PIN_IR_LEFT);
  bool center = readSensor(PIN_IR_CENTER);
  bool right = readSensor(PIN_IR_RIGHT);

#if DEBUG
  Serial.print(left); Serial.print(center); Serial.print(right);
  Serial.print(" state="); Serial.println(state);
#endif

  allBlack = left && center && right;
  if (left && right) lastBlackIr = millis();   
  if (!left && !center && !right) return false;

  if (left && !center && !right) error = -ERROR_FAR;
  else if (left && center && !right) error = -ERROR_NEAR;
  else if (!left && center && right) error = ERROR_NEAR;
  else if (!left && !center && right) error = ERROR_FAR;
  else error = 0;

  return true;
}

void setMotor(uint8_t pinA, uint8_t pinB, uint8_t pinPwm, int speed, bool reversed, float trim) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (reversed) speed = -speed;

  int pwm = constrain((int)(abs(speed) * trim), 0, MAX_PWM);
  if (pwm > 0) pwm = map(pwm, 1, MAX_PWM, MIN_PWM, MAX_PWM);

  digitalWrite(pinA, speed > 0);
  digitalWrite(pinB, speed < 0);
  analogWrite(pinPwm, pwm);
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(PIN_IN1, PIN_IN2, PIN_ENA, leftSpeed, LEFT_MOTORS_REVERSED, LEFT_TRIM);
  setMotor(PIN_IN3, PIN_IN4, PIN_ENB, rightSpeed, RIGHT_MOTORS_REVERSED, RIGHT_TRIM);
}

int getDistanceCM() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 1500);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void moveServoSmooth(int targetAngle) {
  int currentAngle = obstacleServo.read();
  if (currentAngle < targetAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      obstacleServo.write(angle);
      delay(8);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      obstacleServo.write(angle);
      delay(8);
    }
  }
}

void handleObstacleAvoidance() {
  state = AVOIDING_OBSTACLE;
  drive(0, 0);

  if (!obstacleServo.attached()) {
    obstacleServo.attach(PIN_SERVO);
    obstacleServo.write(SERVO_CENTER_ANGLE);
    delay(100);
  }

  moveServoSmooth(15);
  delay(120);
  int distRight = getDistanceCM();

  moveServoSmooth(165);
  delay(120);
  int distLeft = getDistanceCM();

  moveServoSmooth(SERVO_CENTER_ANGLE);
  delay(100);

#if SERVO_DETACH_WHEN_IDLE
  obstacleServo.detach();
#endif

  }

void followLine(int error) {
  state = FOLLOWING;

  if (allBlack) {
    drive(CREEP_SPEED, CREEP_SPEED);
    lastSeenCentered = false;
    return;
  }

  if (error == 0) integral = 0;
  else integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  int derivative = error - lastError;
  int correction = (int)(KP * error + KI * integral + KD * derivative);
  int speed = BASE_SPEED - SPEED_DROP_PER_ERROR * abs(error);

  drive(speed + correction, speed - correction);

  lastError = error;
  if (error != 0) lastDirection = error;
  lastSeenCentered = (error == 0);
}

void handleLostLine(unsigned long now) {
  if (state == FOLLOWING) {
    integral = 0;
    lostStart = now;
    if (lastSeenCentered) {
      state = CROSSING_GAP;
    } else {
      state = SEARCHING;
      searchStart = now;
    }
  }

  if (state == CROSSING_GAP && now - lostStart >= GAP_TIMEOUT_MS) {
    state = SEARCHING;
    searchStart = now;
  }

  if (state == SEARCHING && SEARCH_TIMEOUT_MS > 0 && now - searchStart >= SEARCH_TIMEOUT_MS) {
    state = STOPPED;
  }

  switch (state) {
    case CROSSING_GAP:
      drive(BASE_SPEED, BASE_SPEED);
      break;
    case SEARCHING: {
  
      unsigned long t = now - searchStart;
      unsigned long dur = SWEEP_STEP_MS;
      int phase = 0;
      while (t >= dur) { t -= dur; phase++; dur += SWEEP_STEP_MS; }
      int dir = (phase % 2 == 0) ? lastDirection : -lastDirection;
      if (dir < 0) drive(-SEARCH_SPEED, SEARCH_SPEED);
      else drive(SEARCH_SPEED, -SEARCH_SPEED);
      break;
    }
    default:
      drive(0, 0);
      break;
  }
}

const int GC_GREEN = 0;
const int GC_OFFLINE = 1;        
const int GC_DARK = 2;           
const int GC_SATURATED = 3;      
const int GC_LOW_RATIO = 4;      
const int GC_NOT_OVER_RED = 5;   
const int GC_NOT_OVER_BLUE = 6;  

const char* greenCheckName(int code) {
  switch (code) {
    case GC_GREEN:         return "GREEN";
    case GC_OFFLINE:       return "OFFLINE (not found at startup)";
    case GC_DARK:          return "too dark";
    case GC_SATURATED:     return "SATURATED (lower gain)";
    case GC_LOW_RATIO:     return "not green: G ratio low";
    case GC_NOT_OVER_RED:  return "not green: G not above R";
    case GC_NOT_OVER_BLUE: return "not green: G not above B";
  }
  return "?";
}

int greenCheck(TCS34725Lite &sensor, bool ok, uint16_t raw[4]) {
  raw[0] = raw[1] = raw[2] = raw[3] = 0;
  if (!ok) return GC_OFFLINE;

  uint16_t r, g, b, c;
  sensor.getRawData(&r, &g, &b, &c);
  raw[0] = r; raw[1] = g; raw[2] = b; raw[3] = c;

  if (c < GREEN_MIN_CLEAR) return GC_DARK;
  if (c >= COLOR_SAT_LIMIT) return GC_SATURATED;

  float sum = (float)r + (float)g + (float)b;
  if (sum <= 0.0f) return GC_DARK;

  if ((float)g / sum <= GREEN_MIN_RATIO) return GC_LOW_RATIO;
  if (((float)g - (float)r) <= GREEN_MARGIN * (float)c) return GC_NOT_OVER_RED;
  if (((float)g - (float)b) <= GREEN_MARGIN * (float)c) return GC_NOT_OVER_BLUE;
  return GC_GREEN;
}

bool sensorSeesGreen(TCS34725Lite &sensor, bool ok, uint16_t raw[4]) {
  return greenCheck(sensor, ok, raw) == GC_GREEN;
}

#if COLOR_DEBUG || COLOR_TEST
void printGreenLine(const uint16_t l[4], int cl, const uint16_t r[4], int cr) {
  Serial.print("L r"); Serial.print(l[0]); Serial.print(" g"); Serial.print(l[1]);
  Serial.print(" b"); Serial.print(l[2]); Serial.print(" c"); Serial.print(l[3]);
  Serial.print(" : "); Serial.print(greenCheckName(cl));
  Serial.print("   ||   R r"); Serial.print(r[0]); Serial.print(" g"); Serial.print(r[1]);
  Serial.print(" b"); Serial.print(r[2]); Serial.print(" c"); Serial.print(r[3]);
  Serial.print(" : "); Serial.println(greenCheckName(cr));
}
#endif

void blinkColorStatus() {
  uint8_t n = (leftColorOk ? 0 : 1) + (rightColorOk ? 0 : 2);
  if (n == 0) return;
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(150);
    digitalWrite(LED_BUILTIN, LOW);  delay(150);
  }
}

int checkGreenMarker(unsigned long now) {
  if (now < greenIgnoreUntil) return MARKER_NONE;
  if (now - lastGreenCheck < GREEN_CHECK_INTERVAL_MS) return MARKER_NONE;
  lastGreenCheck = now;

  uint16_t l[4], r[4];
  int cl = greenCheck(leftSensor, leftColorOk, l);
  int cr = greenCheck(rightSensor, rightColorOk, r);
  bool gl = (cl == GC_GREEN);
  bool gr = (cr == GC_GREEN);

  if ((leftColorOk && l[3] < BLACK_MAX_CLEAR) || (rightColorOk && r[3] < BLACK_MAX_CLEAR)) lastBlackColor = now;

#if COLOR_DEBUG
  printGreenLine(l, cl, r, cr);
#endif

  if (gl || gr) {
    bool blackByColor = (now - lastBlackColor < BLACK_THEN_GREEN_WINDOW_MS);
#if USE_IR_FOR_BLACK
    bool blackByIr = (now - lastBlackIr < BLACK_THEN_GREEN_WINDOW_MS);
#else
    bool blackByIr = false;
#endif
    if (blackByColor || blackByIr) {
      lastBlackColor = now;
#if COLOR_DEBUG
      Serial.print("black->green: ignored (");
      Serial.print(blackByColor ? "color " : "");
      Serial.print(blackByIr ? "IR" : "");
      Serial.println(")");
#endif
      return MARKER_NONE;
    }
  }

  if (gl && gr) return MARKER_BOTH;
  if (gl) return MARKER_LEFT;
  if (gr) return MARKER_RIGHT;
  return MARKER_NONE;
}

bool inGreenManeuver() {
  return state == GREEN_CHECKING || state == GREEN_ADVANCING || state == GREEN_TURNING || state == GREEN_UTURN;
}

void startGreenManeuver(int marker, unsigned long now) {
  drive(0, 0);
  integral = 0;
  greenLeftHits  = (marker == MARKER_LEFT  || marker == MARKER_BOTH) ? 1 : 0;   
  greenRightHits = (marker == MARKER_RIGHT || marker == MARKER_BOTH) ? 1 : 0;
  state = GREEN_CHECKING;
  greenStateStart = now;
}

void chooseGreenTurn(int marker, unsigned long now) {
  if (marker == MARKER_LEFT) {
    greenTurnDir = -1;  greenNextState = GREEN_TURNING;  greenForwardMs = FORWARD_BEFORE_TURN_MS;
  } else if (marker == MARKER_RIGHT) {
    greenTurnDir = 1;   greenNextState = GREEN_TURNING;  greenForwardMs = FORWARD_BEFORE_TURN_MS;
  } else {  
    greenTurnDir = UTURN_DIRECTION;  greenNextState = GREEN_UTURN;  greenForwardMs = FORWARD_BEFORE_UTURN_MS;
  }

  integral = 0;
  state = (greenForwardMs > 0) ? GREEN_ADVANCING : greenNextState;
  greenStateStart = now;
}

void finishGreenCheck(int marker, unsigned long now) {
#if COLOR_DEBUG
  Serial.print("green check L="); Serial.print(greenLeftHits);
  Serial.print(" R="); Serial.print(greenRightHits);
  Serial.print(" -> ");
  Serial.println(marker == MARKER_BOTH ? "U-TURN" : (marker == MARKER_LEFT ? "LEFT" : (marker == MARKER_RIGHT ? "RIGHT" : "false alarm")));
#endif
  if (marker == MARKER_NONE) {
    drive(0, 0);
    state = FOLLOWING;
    integral = 0;
    greenIgnoreUntil = now + GREEN_FALSE_ALARM_COOLDOWN_MS;
  } else {
    chooseGreenTurn(marker, now);
  }
}

bool pivotForMs(unsigned long t, unsigned long ms) {
  if (t < TURN_PAUSE_MS) {
    drive(0, 0);
    return false;
  }
  if (t - TURN_PAUSE_MS < ms) {
    if (greenTurnDir > 0) drive(TURN_SPEED, -TURN_SPEED);
    else drive(-TURN_SPEED, TURN_SPEED);
    return false;
  }
  return true;
}

void finishGreenManeuver(unsigned long now) {
  drive(0, 0);
  state = FOLLOWING;
  integral = 0;
  lastError = 0;
  lastDirection = greenTurnDir;      
  lastSeenCentered = false;
  allBlack = false;
  greenIgnoreUntil = now + GREEN_COOLDOWN_MS;
  obstacleHits = 0;
}

void runGreenManeuver(unsigned long now) {
  unsigned long t = now - greenStateStart;

  switch (state) {
    case GREEN_CHECKING: {                
      if (t < GREEN_VERIFY_STOP_MS) drive(0, 0);                                    
      else drive(GREEN_VERIFY_CREEP_SPEED, GREEN_VERIFY_CREEP_SPEED);               

      uint16_t raw[4];
      if (sensorSeesGreen(leftSensor,  leftColorOk,  raw)) greenLeftHits++;
      if (sensorSeesGreen(rightSensor, rightColorOk, raw)) greenRightHits++;

      bool leftGreen  = greenLeftHits  >= GREEN_VERIFY_MIN_HITS;
      bool rightGreen = greenRightHits >= GREEN_VERIFY_MIN_HITS;

      if (leftGreen && rightGreen) {                       
        finishGreenCheck(MARKER_BOTH, now);
      } else if (t >= GREEN_VERIFY_MS) {                   
        if (leftGreen)       finishGreenCheck(MARKER_LEFT, now);
        else if (rightGreen) finishGreenCheck(MARKER_RIGHT, now);
        else                 finishGreenCheck(MARKER_NONE, now);
      }
      break;
    }

    case GREEN_ADVANCING:                 
      drive(BASE_SPEED, BASE_SPEED);
      if (t >= greenForwardMs) {
        state = greenNextState;
        greenStateStart = now;
      }
      break;

    case GREEN_TURNING:                   
      if (pivotForMs(t, TURN_90_MS)) finishGreenManeuver(now);
      break;

    case GREEN_UTURN:                     
      if (pivotForMs(t, TURN_180_MS)) finishGreenManeuver(now);
      break;

    default:
      break;
  }
}

void setup() {
#if DEBUG
  Serial.begin(9600);
#endif
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_CENTER, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

#if OBSTACLE_ENABLED
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  obstacleServo.attach(PIN_SERVO);
  obstacleServo.write(SERVO_CENTER_ANGLE);
  delay(500);                       
#if SERVO_DETACH_WHEN_IDLE
  obstacleServo.detach();           
#endif
#endif

#if COLOR_DEBUG || COLOR_TEST
  Serial.begin(9600);
#endif
  for (uint8_t i = 0; i < 3 && !leftColorOk; i++)  { leftColorOk  = leftSensor.begin();  if (!leftColorOk)  delay(10); }
  for (uint8_t i = 0; i < 3 && !rightColorOk; i++) { rightColorOk = rightSensor.begin(); if (!rightColorOk) delay(10); }
  if (leftColorOk)  { leftSensor.setIntegrationTime(COLOR_ATIME);  leftSensor.setGain(COLOR_GAIN); }
  if (rightColorOk) { rightSensor.setIntegrationTime(COLOR_ATIME); rightSensor.setGain(COLOR_GAIN); }
  delay(40);  
#if COLOR_DEBUG || COLOR_TEST
  Serial.print("left sensor: ");  Serial.println(leftColorOk ? "OK" : "NOT FOUND");
  Serial.print("right sensor: "); Serial.println(rightColorOk ? "OK" : "NOT FOUND");
#endif
  blinkColorStatus();

  drive(0, 0);
  delay(START_DELAY_MS);
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_INTERVAL_MS) return;
  lastLoopTime = now;

#if COLOR_TEST
  drive(0, 0);
  static unsigned long testLast = 0;
  if (now - testLast >= 250) {
    testLast = now;
    uint16_t tl[4], tr[4];
    int tcl = greenCheck(leftSensor, leftColorOk, tl);
    int tcr = greenCheck(rightSensor, rightColorOk, tr);
    printGreenLine(tl, tcl, tr, tcr);
  }
  return;
#endif

#if TURN_CALIBRATION
  static unsigned long calibrationLast = 0;
  if (inGreenManeuver()) {
    runGreenManeuver(now);
  } else {
    drive(0, 0);
    if (now - calibrationLast >= 3000) {
      calibrationLast = now;
      chooseGreenTurn(CALIBRATION_MARKER, now);
    }
  }
  return;
#endif

  if (inGreenManeuver()) {
    runGreenManeuver(now);
    return;
  }

  if (state != STOPPED) {
    int marker = checkGreenMarker(now);
    if (marker != MARKER_NONE) {
      startGreenManeuver(marker, now);
      return;
    }
  }

#if OBSTACLE_ENABLED
  if (state == FOLLOWING && now - lastObstacleCheck >= OBSTACLE_CHECK_INTERVAL_MS) {
    lastObstacleCheck = now;
    int distance = getDistanceCM();
    if (distance > 0 && distance <= OBSTACLE_THRESHOLD_CM) obstacleHits++;
    else obstacleHits = 0;

    if (obstacleHits >= OBSTACLE_CONFIRM_READS) {
      obstacleHits = 0;
      handleObstacleAvoidance();   
      return;
    }
  }
#endif

  int error;
  if (readLineError(error)) followLine(error);
  else handleLostLine(now);
}