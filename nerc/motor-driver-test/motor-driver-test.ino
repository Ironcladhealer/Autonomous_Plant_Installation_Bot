#include <Arduino.h>

const int ENA = 48;
const int IN1 = 46;
const int IN2 = 44;
const int ENB = 49;
const int IN3 = 47;
const int IN4 = 45;

void setup(){
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop(){
  analogWrite(ENA, 240);
  analogWrite(ENB, 240);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}