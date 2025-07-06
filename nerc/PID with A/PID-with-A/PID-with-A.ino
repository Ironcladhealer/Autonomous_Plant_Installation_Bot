#include <QTRSensors.h>

// Motor pins
const int ENA = 5;   // Speed control for left motor
const int IN1 = 6;   // Left motor direction
const int IN2 = 7;
const int ENB = 8;   // Speed control for right motor
const int IN3 = 9;   // Right motor direction
const int IN4 = 10;

// Encoder pins
const int encoderLeft = 2;
const int encoderRight = 3;

// IR sensor matrix reading setup
const int irSensors[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
int matrix[3][3];
const int voltagePin = 31;

// Line-following setup
QTRSensors qtr;
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

// PID constants
float Kp = 0.6, Ki = 0, Kd = 0.3;
int lastError = 0;
float integral = 0;
const int goal = 3500;

// Junction counting
const int leftIR = 32;
const int rightIR = 33;
int junctionCounter = 0;

// Target positions
const int MAX_TARGETS = 3;
int targets[MAX_TARGETS][2];
int targetCount = 0;

// Start position
int startRow = 0;
int startCol = 0;

// Node structure for A* algorithm
struct Node {
    int row;
    int col;
    int cost;
    int heuristic;

    int f() const {
        return cost + heuristic;
    }

    bool operator==(const Node& other) const {
        return row == other.row && col == other.col;
    }
};

// PathStep structure for storing the path
struct PathStep {
    int row, col;
};
PathStep path[10];
int pathLength = 0;

// Heuristic function (Manhattan distance)
int heuristic(int row, int col, int goalRow, int goalCol) {
    return abs(goalRow - row) + abs(goalCol - col);
}

void aStar(int startRow, int startCol, int goalRow, int goalCol) {
    Node start = {startRow, startCol, 0, heuristic(startRow, startCol, goalRow, goalCol)};
    Node goal = {goalRow, goalCol, 0, 0};

    Node openSet[10];
    int openCount = 0;
    openSet[openCount++] = start;

    pathLength = 0;

    while (openCount > 0) {
        int lowestIndex = 0;
        for (int i = 1; i < openCount; i++) {
            if (openSet[i].f() < openSet[lowestIndex].f()) lowestIndex = i;
        }

        Node current = openSet[lowestIndex];

        if (current == goal) {
            path[pathLength++] = {current.row, current.col};
            Serial.println("Goal reached!");
            return;
        }

        for (int i = lowestIndex; i < openCount - 1; i++) {
            openSet[i] = openSet[i + 1];
        }
        openCount--;

        int moves[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

        for (int i = 0; i < 4; i++) {
            int newRow = current.row + moves[i][0];
            int newCol = current.col + moves[i][1];

            if (newRow >= 0 && newRow < 3 && newCol >= 0 && newCol < 3) {
                path[pathLength++] = {newRow, newCol};
            }
        }
    }
}
void navigatePath() {
    for (int i = 0; i < pathLength; i++) {
        int newRow = path[i].row;
        int newCol = path[i].col;

        if (newRow < startRow) moveBackward(150, 150);  // Moving up
        else if (newRow > startRow) moveForward(150, 150);  // Moving down
        else if (newCol > startCol) turnRight();  // Moving right
        else if (newCol < startCol) turnLeft();  // Moving left

        startRow = newRow;
        startCol = newCol;
        delay(1000);
    }
}


void readMatrix() {
    digitalWrite(voltagePin, HIGH);
    delay(1000);

    int sensorIndex = 0;
    targetCount = 0;

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            matrix[row][col] = digitalRead(irSensors[sensorIndex]);
            if (matrix[row][col] == 1 && targetCount < MAX_TARGETS) {
                targets[targetCount][0] = row;
                targets[targetCount][1] = col;
                targetCount++;
            }
            sensorIndex++;
        }
    }

    digitalWrite(voltagePin, LOW);
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

void checkJunction() {
    static bool lastLeft = LOW, lastRight = LOW;
    bool currentLeft = digitalRead(leftIR);
    bool currentRight = digitalRead(rightIR);

    if (currentLeft == HIGH && lastLeft == HIGH) junctionCounter++;

    lastLeft = currentLeft;
    lastRight = currentRight;

    if (junctionCounter == 4) turnRight();
    else if (junctionCounter == 7) {
        turnRight();
        navigateToTargets();
    }
}

void navigateToTargets() {
    for (int i = 0; i < targetCount; i++) {
        int goalRow = targets[i][0];
        int goalCol = targets[i][1];
        aStar(startRow, startCol, goalRow, goalCol);
        startRow = goalRow;
        startCol = goalCol;
    }
    stopMotors();  // Stop after reaching all targets
    Serial.println("All positions reached!");
}

void setup() {
    Serial.begin(9600);
    pinMode(voltagePin, OUTPUT);
    pinMode(leftIR, INPUT);
    pinMode(rightIR, INPUT);

    for (int i = 0; i < 9; i++) pinMode(irSensors[i], INPUT);

    readMatrix();
}

void loop() {
    followLine();
    checkJunction();
}
