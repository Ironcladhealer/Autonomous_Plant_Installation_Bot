#include <QTRSensors.h>
// Line Following Bot with PID and Junction Handling

// Define motor control pins
#define ENA 9  // Enable pin for motor A
#define IN1 5  // Motor A control pin 1
#define IN2 4  // Motor A control pin 2
#define ENB 8 // Enable pin for motor B
#define IN3 7 // Motor B control pin 1
#define IN4 6  // Motor B control pin 2a

// Define QTR sensor and IR sensor pins
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

#define LEFT_IR 2      // Left IR sensor for junction detection
#define RIGHT_IR 13    // Right IR sensor for junction detection


// PID control variables
const double Kp = 0.072;  // Proportional gain
const float Ki = 0.0;   // Integral gain (set to 0 if not needed)
const double Kd = 0.14;  // Derivative gain
const int goal = 3500; //Goal (it may be 2500, depending upon QTR value)
const int baseSpeed = 255; //Max Speed

int lastError = 0;
int integral = 0;
int junctionCount = 0; // Keeps track of junctions detected


void setup() {
  
    Serial.begin(9600);
   qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    pinMode(LEFT_IR, INPUT);
    pinMode(RIGHT_IR, INPUT);
    
    calibrateQTR();
    delay(2000); // Wait before starting movement
}

void loop() {

//IR Sensor  
    int leftIR = digitalRead(LEFT_IR);
    int rightIR = digitalRead(RIGHT_IR);
//QTR Sensor
    unsigned int position = qtr.readLineBlack(sensorValues);
    int error = goal - position;

    // PID calculations
    integral += error;
    int derivative = error - lastError;
    int output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    lastError = error;

     // Base speed of motors
    int leftSpeed = baseSpeed + output;
    int rightSpeed = baseSpeed - output;
    
    leftSpeed = constrain(leftSpeed, 0, baseSpeed);
    rightSpeed = constrain(rightSpeed, 0, baseSpeed);

    moveForward(leftSpeed, rightSpeed);

    // Junction Detection
    if (leftIR == 1 && rightIR == 1) {  // If both IR sensors detect black
        junctionCount++;
        Serial.print("Junction Count: ");
        Serial.println(junctionCount);
        delay(1000);  // Debounce delay
    }
    
    // Junction-based decision making
    if (junctionCount == 5 || junctionCount == 7) {
        moveRight();
      delay(4000);
        } else if (junctionCount == 15) {
        moveLeft();
      delay(4000);
    }
}

void calibrateQTR() {
  delay(500);
  for (int i = 0; i <1000; i++){
    qtr.calibrate();
  }
}

void moveForward(int left, int right) {
    analogWrite(ENA, left);
    analogWrite(ENB, right);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void moveLeft() {
    Serial.println("Turning Left");
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(500);
    //stopMotors();
}

void moveRight() {
    Serial.println("Turning Right");
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(500);
    //stopMotors();
}

void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}