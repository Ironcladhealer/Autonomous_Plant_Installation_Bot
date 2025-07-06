#include <Arduino.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

//Initialize color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

//Servo Motors
Servo myServo1;
Servo myServo2;
//Servo myServo3;
//Servo myServo4;

// IR Sensors
#define LEFT_TURN_COUNTER_SENSOR 22
#define RIGHT_TURN_COUNTER_SENSOR 25
#define BOX_DETECT_IR 26

//Sonar Settings
/*#define TRIG_PIN 24


long duration;
float distance;
*/

// Create an instance of the QTRSensors class
QTRSensors qtr;

// Define the number of sensors
const int numSensors = 8;

// Junction Detection Variables
int junctionCount = 0;
bool junctionDetected = false;
int counterLeft;
int counterRight;
bool turnComplete = false;
int ti;
int tf;

// Array to store the sensor values
unsigned int sensorValues[numSensors];

// Threshold value to convert analog readings to binary (0 or 1)
const int threshold = 1000;  // Adjust this based on your sensor's behavior

// Motor control pins

const int ENA = 48;
const int IN1 = 46;
const int IN2 = 44;
const int ENB = 49;
const int IN3 = 47;
const int IN4 = 45;

// PID control variables
float Kp = 0.1;     // Proportional gain
float Ki = 0.00;    // Integral gain
float Kd = 0.0001;  // Derivative gain
int lastError = 0;
int integral = 0;

// Base motor speed
const int baseSpeed = 170;

bool boxIsDetected = false;
bool isProcessingBox = false;  //
unsigned long skipJunctionUntil = 0;


//Junction Logic
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 500;

int pos1 = 90;
int pos2 = 0;

void setup() {
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 9, 8, 7, 6, 5, 4, 3, 2, 1 }, numSensors);

  for (int i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(10);
  }

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LEFT_TURN_COUNTER_SENSOR, INPUT);
  pinMode(RIGHT_TURN_COUNTER_SENSOR, INPUT);

  myServo1.attach(12);
  myServo2.attach(10);
  //myServo3.attach(12);
  //myServo4.attach(13);

  Serial.println("Calibration complete.");
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;
  }

  tf = millis();
  ti = tf;
}

void loop() {
  myServo1.write(pos1);
  myServo2.write(pos2);
  counterLeft = digitalRead(LEFT_TURN_COUNTER_SENSOR);
  counterRight = digitalRead(RIGHT_TURN_COUNTER_SENSOR);
  detectJunction();

  // Only check for boxes in this specific junction range
  if (junctionCount > 9 && junctionCount < 22) {
    detectBoxColor();

    if (boxIsDetected) {
      isProcessingBox = true;
      stopMotors();
      delay(400);
      String color = getColor();  // Already stopped in detectBoxColor()
      actOnBoxColor(color);
      Serial.print("Detected Color: ");
      Serial.println(color);

      // Wait before continuing
      delay(500);
      isProcessingBox = false;
      return;  // Skip rest of loop
    }
  }

  // Only follow the line if no box was detected
  if (!boxIsDetected && !isProcessingBox) {
    lineFollow();
  }
}

void lineFollow() {
  qtr.read(sensorValues);

  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < numSensors; i++) {
    weightedSum += (long)sensorValues[i] * i * 1000;
    total += sensorValues[i];
  }

  if (total > 0) {
    int position = weightedSum / total;
    int center = ((numSensors - 1) * 1000) / 2;

    int error = position - center;
    integral += error;
    integral = constrain(integral, -500, 500);  // Anti-windup

    int motorSpeed = Kp * error + Ki * integral + Kd * (error - lastError);
    lastError = error;

    int leftMotorSpeed = baseSpeed - motorSpeed;
    int rightMotorSpeed = baseSpeed + motorSpeed;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    driveRobot(leftMotorSpeed, rightMotorSpeed);
  } else {
    moveForward();  // Or stop / search for line
  }

  delay(2);
}

void driveRobot(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, rightSpeed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void lineFollowBackward() {
  int leftIR = digitalRead(32);   // HIGH = sees line, LOW = no line
  int rightIR = digitalRead(31);  // HIGH = sees line, LOW = no line

  int leftSpeed = 150;
  int rightSpeed = 150;

  if (leftIR == HIGH && rightIR == HIGH) {
    // Both sensors see line → go straight backward
    analogWrite(ENA, leftSpeed);
    digitalWrite(IN1, LOW);  // Reverse
    digitalWrite(IN2, HIGH);

    analogWrite(ENB, rightSpeed);
    digitalWrite(IN3, HIGH);  // Reverse
    digitalWrite(IN4, LOW);
  } else if (leftIR == HIGH && rightIR == LOW) {
    // Line is on the left → turn left (pivot slightly right)
    analogWrite(ENA, leftSpeed - 80);  // slow left
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(ENB, rightSpeed + 80);  // fast right
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (leftIR == LOW && rightIR == HIGH) {
    // Line is on the right → turn right (pivot slightly left)
    analogWrite(ENA, leftSpeed + 80);  // fast left
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(ENB, rightSpeed - 80);  // slow right
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // Both sensors lost the line → stop or reverse slowly to search
    analogWrite(ENA, leftSpeed);
    digitalWrite(IN1, LOW);  // Reverse
    digitalWrite(IN2, HIGH);

    analogWrite(ENB, rightSpeed);
    digitalWrite(IN3, HIGH);  // Reverse
    digitalWrite(IN4, LOW);
  }

  delay(30);  // small delay to smooth behavior
}


void detectJunction() {
  // Replace your junction detection block with this:
  if (counterLeft == 1 && counterRight == 1 && !junctionDetected) {
    junctionCount++;
    Serial.print("Junction Count: ");
    Serial.println(junctionCount);
    junctionDetected = true;
    delay(400);
    junctionDetected = false;
  } else {
    junctionDetected = false;
  }

  //Taking Left Turns
  if (junctionCount == 4 || junctionCount == 8 || junctionCount == 17 || junctionCount == 19 || junctionCount == 22 || junctionCount == 42) {
    Serial.println("Turning left at junction");
    delay(50);
    takeLeftTurn();

    // 2-second turn
    turnComplete = true;
    junctionCount++;
  }

  //Taking 180 degree turn
  if (junctionCount == 26) {
    Serial.println("Turning 180 left at junction");
    delay(50);
    takeLeftFullTurn();

    // 2-second turn
    turnComplete = true;
    junctionCount++;
  }

  //Parking
  if (junctionCount == 43) {
    Serial.println("Reaching Parking");
    moveForward();
    delay(1000);
    while (1) {
      stopMotors();
    }
  }

  //Taking Right Turns
  if (junctionCount == 12 || junctionCount == 14 || junctionCount == 33 || junctionCount == 38) {
    Serial.println("Turning right at junction");
    delay(50);
    takeRightTurn();

    // 2-second turn
    // turnComplete = true;
    junctionCount++;
  }
}


void detectBoxColor() {
  if (digitalRead(BOX_DETECT_IR) == LOW) {
    stopMotors();
    delay(500);
    Serial.println("Box Detected");
    boxIsDetected = true;
  } else {
    boxIsDetected = false;
    //lineFollow();
  }
}

String getColor() {
  uint16_t r, g, b, c, colorTemp;
  delay(200);
  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);

  if (r > g && r > b) {
    Serial.println("red");
    return "red";
  }
  if (g > r && g > b) {
    delay(500);
    for (int i = 0; i < 10; i++) {
      if (r > g && r > b) {
        Serial.println("red");
        return "red";
        break;
      } else {
        //
      }
    }
    if (r > g && r > b) {
      Serial.println("red");
      return "red";
    }
    if (g > r && g > b) {
      if (r > g && r > b) {
        Serial.println("red");
        return "red";
      } else {
        Serial.println("green");
        return "green";
      }
    } else {
      Serial.println("green");
      return "green";
    }
  }
  if (b > r && b > g) {
    Serial.println("blue");
    return "blue";
  } else {
    Serial.println("No Color");
    return "NONE";
  }
}

void actOnBoxColor(String color) {

  // skipJunctionUntil = millis() + 3000;
  isProcessingBox = true;
  if (color == "red") {
    Serial.println("Lifting red box");
    moveBackwardLineFollowing(700);  // 1000 ms = 1 second


    takeLeftFullTurn();
    stopMotors();
    delay(50);

    for (int pos1 = 90; pos1 >= 0; pos1--) {
      myServo1.write(pos1);
      delay(20);
    }

    moveBackwardLineFollowing(1650);  // 1000 ms = 1 second


    for (int pos1 = 0; pos1 <= 90; pos1++) {
      myServo1.write(pos1);
      delay(20);
    }

    delay(200);

    takeLeftFullTurn();
    stopMotors();
    delay(50);

    moveForwardLineFollowing(100);
    stopMotors();
    delay(200);
    junctionCount = junctionCount - 3;
    boxIsDetected = false;
    isProcessingBox = false;

  } else if (color == "blue") {
    Serial.println("Lifting blue box");
    delay(2500);
    boxIsDetected = false;
    isProcessingBox = false;
  } else if (color == "green") {
    Serial.println("Lifting green box");

    moveBackwardLineFollowing(1200);  // 1000 ms = 1 second
    /*analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(500);*/
    stopMotors();
    delay(50);

    for (int pos2 = 0; pos2 <= 150; pos2++) {
      myServo2.write(pos2);
      delay(10);
    }

    moveForwardLineFollowing(1000);
    /*   analogWrite(ENA, 170);
    analogWrite(ENB, 170);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(800);*/
    stopMotors();
    delay(200);

    for (int pos2 = 150; pos2 >= 0; pos2--) {
      myServo2.write(pos2);
      delay(10);
    }
    delay(200);
    junctionCount = junctionCount - 2;
    boxIsDetected = false;
    isProcessingBox = false;
    /*for (int pos = 150; pos <= 30; pos--) {
      myServo2.write(pos);
      delay(10);
    }*/
  } else {
    Serial.println("Lifting nothing");
  }
}

void moveBackwardLineFollowing(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    lineFollowBackward();
  }
  stopMotors();
  delay(50);
}

void moveForwardLineFollowing(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    lineFollow();
  }
  stopMotors();
  delay(50);
}

void takeLeftTurn() {

  // Start turning left
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(400);

  // Resume line following after turn
}

void takeLeftFullTurn() {
  // Start turning left (both motors forward, creating a 180° rotation)
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(800);
}

void takeRightTurn() {

  // Start turning right
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(400);
}

void moveForward() {
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, 170);
  analogWrite(ENB, 170);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(500);
  return;
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}