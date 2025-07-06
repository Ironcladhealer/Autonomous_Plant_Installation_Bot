// IR Sensor Pins
#define LEFTMOST_SENSOR 4
#define LEFT_TURN_COUNTER_SENSOR 5
#define MID_SENSOR 6
#define RIGHTMOST_SENSOR 7
#define RIGHT_TURN_COUNTER_SENSOR 33

// Motor control pins
const int ENA = 8;
const int IN1 = 9;
const int IN2 = 10;
const int ENB = 13;
const int IN3 = 11;
const int IN4 = 12;

int junctionLeftCount = 0;
int junctionRightCount = 0;
int junctionCount = 0;
bool junctionLeftDetected = false;
bool junctionRightDetected = false;
bool junctionDetected = false;

bool matrixMode = true;

int right;
int mid;
int left;
int counter;
int counterLeft;
int counterRight;
bool turnRightCondition;

int row = 0;
int col = 0;

void setup() {
  // Sensor Pins
  pinMode(LEFTMOST_SENSOR, INPUT);
  pinMode(MID_SENSOR, INPUT);
  pinMode(RIGHTMOST_SENSOR, INPUT);
  pinMode(LEFT_TURN_COUNTER_SENSOR, INPUT);
  pinMode(RIGHT_TURN_COUNTER_SENSOR, INPUT);
  Serial.begin(9600);

  // Motor Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  left = digitalRead(LEFTMOST_SENSOR);
  mid = digitalRead(MID_SENSOR);
  right = digitalRead(RIGHTMOST_SENSOR);
  counterLeft = digitalRead(LEFT_TURN_COUNTER_SENSOR);
  counterRight = digitalRead(RIGHT_TURN_COUNTER_SENSOR);

  //Junction Left Calculation
  if (counterLeft == 1) {
    if (!junctionLeftDetected) {
      junctionLeftCount++;
      Serial.print("Junction Left Count: ");
      Serial.println(junctionLeftCount);
      junctionLeftDetected = false;
      delay(1000);  // Avoid double counting
    } else {
      junctionLeftDetected = false;  // Reset when not on a junction
    }
  }
  //Junction Right Calculation
  if (counterRight == 1) {
    if (!junctionRightDetected) {
      junctionRightCount++;
      // Serial.print("Junction Right Count: ");
      // Serial.println(junctionRightCount);
      junctionRightDetected = false;
      delay(500);  // Avoid double counting
    } else {
      junctionRightDetected = false;  // Reset when not on a junction
    }
  }

   //Special Turns before matrix
  if (junctionLeftCount == 4 || junctionLeftCount == 8) {
    Serial.println("Turning left");
    turnLeft();
    return;
  }
  //Stops motor
  if (junctionLeftCount == 5 || junctionLeftCount == 9) {
    Serial.println("Stopping left turn");
    moveForward();
  }

 if (junctionLeftCount > 9) {
  if (!matrixMode) {
    Serial.println("Switching to matrix mode!");
  }
  matrixMode = true;  // Activate matrix mode
  junctionLeftDetected = false;
}


  // Make sure updatePosition() runs continuously when matrixMode is active
  //if (matrixMode) {
    //updatePosition();
 // }

  /* //In matrix
  if (junctionRightCount == 10 || junctionRightCount == 14) {
    Serial.println("Turning Right");
    turnRight();
    return;  // Skip rest of the loop while turning right
  }
  //Stops motor
  if (junctionRightCount == 11 || junctionRightCount == 15) {
    Serial.println("Stopping Right turn");
    moveForward();
  }
*/
  //Line Follower and Junction Detection (White = 0 and Black = 1)
  if (left == 1 && mid == 1 && right == 1) {
    moveForward();
  }
  if (left == 0 && mid == 1 && right == 0) {
    moveForward();
  }
  if (left == 1 && mid == 0 && right == 0) {
    turnLeft();
  }
  if (left == 1 && mid == 1 && right == 0) {
    turnLeft();
  }
  if (left == 0 && mid == 0 && right == 1) {
    turnRight();
  }
  if (left == 0 && mid == 1 && right == 1) {
    turnRight();
  }
    if (left == 1 && mid == 1 && right == 0) {
    turnLeft();
  }
  if (left == 0 && mid == 0 && right == 0) {
    moveForward();
  }
}

void updatePosition() {
  if (counterLeft == 1 && counterRight == 1) {
    if (!junctionDetected) {
      junctionCount++;
      Serial.print("Junction Count: ");
      Serial.println(junctionCount);
      junctionDetected = true;  // Set to true so we don’t double-count
      delay(800);               // Avoid double-counting

      if (row == 0) {
        row = 1;
        col = 1;
      } else if (row == 1) {
        if (col < 3) {
          col++;  // Moving right
          moveForward();
        } else {
          row++;  // Reached (1,3), turn down
          Serial.println("Turning right to move down");
          turnRight();
          delay(2000);
        }
      } else if (row == 2) {
        if (col > 1) {
          col--;  // Moving left
          moveForward();
        } else {
          row++;  // Reached (2,1), turn down
          Serial.println("Turning left to move down");
          turnLeft();
          delay(2000);
        }
      } else if (row == 3) {
        if (col < 3) {
          col++;
          moveForward();
        } else {
          Serial.println("Reached (3,3), stopping");
          stopMotors();
        }
      }

      Serial.print("Current Position: (");
      Serial.print(row);
      Serial.print(",");
      Serial.print(col);
      Serial.println(")");
    }
  } else {
    // Reset junctionDetected only when the bot moves off the junction
    if (junctionDetected) {
      Serial.println("Left the junction");
    }
    junctionDetected = false;
  }

  // Line following when not on a junction
  if (!junctionDetected) {
    if (left == 1 && mid == 1 && right == 1) {
      moveForward();
    } else if (left == 0 && mid == 1 && right == 0) {
      moveForward();
    } else if (left == 1 && mid == 0 && right == 0) {
      turnLeft();
    } else if (left == 1 && mid == 1 && right == 0) {
      turnLeft();
    } else if (left == 0 && mid == 0 && right == 1) {
      turnRight();
    } else if (left == 0 && mid == 1 && right == 1) {
      turnRight();
    } else if (left == 0 && mid == 0 && right == 0) {
      moveForward();
    }
  }
}


// Motor Control Functions
void moveForward() {
  // Serial.println("Moving Forward!");
  analogWrite(ENA, 240);
  analogWrite(ENB, 240);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  // Serial.println("Moving Right!");
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  //Serial.println("Moving Left!");
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
