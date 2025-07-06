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

QTRSensors qtr;

//Sonar Settings
#define TRIG_PIN 24
#define ECHO_PIN 26

long duration;
float distance;

// Define the number of sensors
const int numSensors = 8;

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
float Kp = 0.5;    // Proportional gain
float Ki = 0.00;   // Integral gain
float Kd = 0.001;  // Derivative gain
int lastError = 0;
int integral = 0;

// Base motor speed
const int baseSpeed = 190;

bool boxIsDetected = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 2, 3, 4, 5, 6, 7, 8, 9 }, numSensors);

  /*for (int i = 0; i < 250; i++) {
    qtr.calibrate();
    Serial.print(sensorValues[i]);
    Serial.print("\t");
    delay(50);
  }*/

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  myServo1.attach(11);
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
}

void loop() {
  // put your main code here, to run repeatedly:
  myServo1.write(90);
  myServo2.write(0);
  detectBoxColor();  // Check if a box is nearby

  if (!boxIsDetected) {
    // Follow line when no box is nearby
    moveForward();
  } else {
    // Get box color and act accordingly
    String color = getColor();
    actOnBoxColor(color);
    
    // After action, reset box detection to resume line following
    boxIsDetected = false;
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


void detectBoxColor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read pulse
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Detect box at ~1 cm
  if (distance >= 990.00) {
    Serial.println("Box Detected at ~1 cm!");
    boxIsDetected = true;
    String color = getColor();
    Serial.print("Detected Color: ");
    Serial.println(color);
  }
  else{
    boxIsDetected = false;
  }

  delay(500);
}

String getColor() {
  uint16_t r, g, b, c, colorTemp;
  delay(200);
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  //colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);

  if (r > g && r > b) {
    delay(500);
    for (int i = 0; i < 20; i++) {
      if (g > r && g > b) {
        Serial.println("green");
        return "green";
        break;
      } else {
        Serial.println("red");
        return "red";
      }
    }
  }
  if (g > r && g > b) {
    delay(500);
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
  if (color == "red") {
    Serial.println("Lifting red box");
    moveBackward();
    stopMotors();
    delay(50);
    takeLeftFullTurn();
    stopMotors();
    delay(50);
    moveBackward();
    stopMotors();
    delay(50);
    for (int pos = 90; pos >= 0; pos--) {
      myServo1.write(pos);
      delay(10);
    }
    analogWrite(ENA, 180);
    analogWrite(ENB, 180);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(100);
    stopMotors();
    delay(50);
    for (int pos = 0; pos <= 90; pos++) {
      myServo1.write(pos);
      delay(10);
    }
    takeLeftFullTurn();
  } else if (color == "blue") {
    Serial.println("Lifting blue box");
    delay(2500);
  } else if (color == "green") {
    Serial.println("Lifting green box");
    analogWrite(ENA, 180);
    analogWrite(ENB, 180);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(800);
    stopMotors();
    delay(50);
    for (int pos = 0; pos <= 150; pos++) {
      myServo2.write(pos);
      delay(10);
    }
    analogWrite(ENA, 240);
    analogWrite(ENB, 240);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(120);
    stopMotors();
    delay(200);
    for (int pos = 150; pos <= 30; pos--) {
      myServo2.write(pos);
      delay(10);
    }
  } else {
    Serial.println("Lifting nothing");
  }
}

void takeLeftTurn() {
  // Turn left for exactly 2 seconds
  Serial.println("Moving Left");
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(300);
  return;
  // Resume line following after turn
}

void takeLeftFullTurn() {
  // Turn left for exactly 2 seconds
  Serial.println("Moving Full Left");
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(900);
  return;
  // Resume line following after turn
}

void takeRightTurn() {
  // Turn left for exactly 2 seconds
  Serial.println("Moving Right");
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(500);
  return;
  // Resume line following after turn
}

void moveForward() {
  analogWrite(ENA, 190);
  analogWrite(ENB, 190);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveBackward() {
  Serial.println("Moving Backward");
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(800);
  return;
}

void stopMotors() {
  Serial.println("Stopping motors");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}