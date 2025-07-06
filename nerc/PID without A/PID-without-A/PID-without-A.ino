#include <QTRSensors.h>

// Motor pins
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 10;
const int IN3 = 8;
const int IN4 = 9;

// IR sensor matrix reading setup
//const int irSensors[9] = { 22, 23, 24, 25, 26, 27, 28, 29, 30 };
//int matrix[3][3] = { 0 };
//const int voltagePin = 31;

// Line-following setup
QTRSensors qtr;
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

// PID constants
float Kp = 0.06, Ki = 0, Kd = 0.12;
int lastError = 0;
float integral = 0;
const int goal = 3500;

// Junction counting
const int leftIR = 38;
const int rightIR = 39;
int junctionCounter = 0;

// Matrix tracking
int row = 0, col = 0;
int nextRow = 0;
int nextCol = 0;

// Ultrasonic sensor pins
const int trigPin = 46;
const int echoPin = 47;

// Box counter
int boxesFound = 0;

// Movement pattern for the matrix
const int matrixPath[9][2] = {
  { 1, 1 }, { 1, 2 }, { 1, 3 }, { 2, 3 }, { 2, 2 }, { 2, 1 }, { 3, 1 }, { 3, 2 }, { 3, 3 }
};
int currentStep = 0;

bool detectBox();                                  // Prototype
void moveForward(int leftSpeed, int rightSpeed);   // Prototype
void stopMotors();                                 // Prototype
void moveBackward(int leftSpeed, int rightSpeed);  // Prototype
void turnRight();                                  // Prototype
void turnLeft();                                   // Prototype
void turnRightFor(int duration);                   // Prototype
void turnLeftFor(int duration);                    // Prototype

void setup() {
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A6, A7 }, sensorCount);
  //qtr.setEmitterPin(2);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  if (junctionCounter < 8) {
    followLine();
    checkJunction();
  } else {
    navigateMatrix();
  }
}

void followLine() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = goal - position;
  integral += error;
  int derivative = error - lastError;
  int motorSpeed = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = constrain(150 + motorSpeed, 0, 255);
  int rightSpeed = constrain(150 - motorSpeed, 0, 255);

  moveForward(leftSpeed, rightSpeed);
  lastError = error;
}

void checkJunction() {
  static bool lastLeft = LOW, lastRight = LOW;
  bool currentLeft = digitalRead(leftIR);
  bool currentRight = digitalRead(rightIR);

  if (currentLeft == HIGH && lastLeft == HIGH) {
    junctionCounter++;
    Serial.print("Junction: ");
    Serial.println(junctionCounter);
  }

  lastLeft = currentLeft;
  lastRight = currentRight;

  if (junctionCounter == 4 || junctionCounter == 7) {
    turnRight();
    delay(1000);  // Small delay after turning
  }

  if (junctionCounter == 8) {
    row = 1;
    col = 1;
    currentStep = 0;
  }
}

void navigateMatrix() {

  int targetRow = matrixPath[currentStep][0];
  int targetCol = matrixPath[currentStep][1];

  if (row == targetRow && col == targetCol) {

    if (detectBox()) {
      boxesFound++;
      delay(5000);  // Stop for 5 seconds when box is detected
    }

    currentStep = (currentStep + 1) % 9;
    row = matrixPath[currentStep][0];
    col = matrixPath[currentStep][1];
    // Turn right when moving from (1,3) to (2,3) or (2,1) to (3,1)
    if ((row == 1 && col == 3 && nextRow == 2) || (row == 2 && col == 3 && nextCol == 2)) {
      turnLeft();
    }

    // Turn left when moving from (2,3) to (2,2) or (3,3) to exit
    if ((row == 3 && col == 1 && nextRow == 3) || (row == 2 && col == 1 && nextRow == 3)) {
      turnRight();
    }
    nextRow = matrixPath[(currentStep + 1) % 9][0];
    nextCol = matrixPath[(currentStep + 1) % 9][1];

    row = nextRow;
    col = nextCol;



    if (boxesFound >= 3) {
      Serial.println("All boxes found! Exiting matrix.");
      stopMotors();
      if (row == 1 || row == 3) {
        moveForward(150, 150);
      } else if (row == 2) {
        turnRightFor(2000);  // 180-degree turn
      }
    }
  }
}

  bool detectBox() {
    long duration;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    float distance = duration * 0.034 / 2;

    return (distance <= 5);  // Detect box when 5 cm or closer
  }

  void moveForward(int leftSpeed, int rightSpeed) {
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  void moveBackward(int leftSpeed, int rightSpeed) {
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  void turnRight() {
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(2000);
  }

  void turnLeft() {
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(2000);
  }

  void turnRightFor(int duration) {
    turnRight();
    delay(duration);
    stopMotors();
  }

  void turnLeftFor(int duration) {
    turnLeft();
    delay(duration);
    stopMotors();
  }
