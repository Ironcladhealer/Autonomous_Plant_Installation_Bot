// Motor driver pins (L298N)
const int ENA = 5;  // Speed control for FL and BL
const int IN1 = 6;  // FL Direction
const int IN2 = 7;
const int IN3 = 8;  // BL Direction
const int IN4 = 9;
const int ENB = 10;

const int END = 29;
const int IN5 = 30; // FR Direction
const int IN6 = 32;
const int IN7 = 31; // BR Direction
const int IN8 = 34;
const int ENC = 33;


// Encoder pins
const int encoderFL = 2;
const int encoderFR = 19;
const int encoderBL = 3;
const int encoderBR = 18;

volatile long countFL = 0;
volatile long countFR = 0;
volatile long countBL = 0;
volatile long countBR = 0;

void setup() {
  Serial.begin(9600);

  // Motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // Encoder pins
  pinMode(encoderFL, INPUT);
  pinMode(encoderFR, INPUT);
  pinMode(encoderBL, INPUT);
  pinMode(encoderBR, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderFL), countFLPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderFR), countFRPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBL), countBLPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBR), countBRPulse, RISING);
}

// Forward
void moveForward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
    analogWrite(ENC, speed);
  analogWrite(END, speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
}

// Backward
void moveBackward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
    analogWrite(ENC, speed);
  analogWrite(END, speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
}

// Strafe Right
void strafeRight(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
    analogWrite(ENC, speed);
  analogWrite(END, speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
}

// Strafe Left
void strafeLeft(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
  analogWrite(END, speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
}

// Stop
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
  analogWrite(END, 0);
}

// Encoder Interrupts
void countFLPulse() { countFL++; }
void countFRPulse() { countFR++; }
void countBLPulse() { countBL++; }
void countBRPulse() { countBR++; }

void loop() {
  moveForward(230);
  delay(2000);
  stopMotors();
  delay(1000);

  moveBackward(230);
  delay(2000);
  stopMotors();
  delay(1000);

  strafeRight(230);
  delay(2000);
  stopMotors();
  delay(1000);

  strafeLeft(230);
  delay(2000);
  stopMotors();
  delay(1000);

  // Display encoder counts
  Serial.print("FL: "); Serial.print(countFL);
  Serial.print(" FR: "); Serial.print(countFR);
  Serial.print(" BL: "); Serial.print(countBL);
  Serial.print(" BR: "); Serial.println(countBR);
}
