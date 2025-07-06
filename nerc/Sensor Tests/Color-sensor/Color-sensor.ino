#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

//Initialize color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

#define TRIG_PIN 24
#define ECHO_PIN 26

long duration;
float distance;

void setup() {
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;
  }
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  detectBoxColor();
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
        Serial.println(r);
        Serial.println(g);
        Serial.println(b);
        delay(1000);
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
      Serial.println(r);
      Serial.println(g);
      Serial.println(b);
      delay(1000);
      return "red";
    }
    if (g > r && g > b) {
      if (r > g && r > b) {
        Serial.println("red");
        Serial.println(r);
        Serial.println(g);
        Serial.println(b);
        delay(1000);
        return "red";
      } else {
        Serial.println("green");
        Serial.println(r);
        Serial.println(g);
        Serial.println(b);
        delay(1000);
        return "green";
      }
    } else {
      Serial.println("green");
      Serial.println(r);
      Serial.println(g);
      Serial.println(b);
      delay(1000);
      return "green";
    }
  }
  if (b > r && b > g) {
    Serial.println("blue");
    Serial.println(r);
    Serial.println(g);
    Serial.println(b);
    delay(1000);
    return "blue";
  } else {
    Serial.println("No Color");
    return "NONE";
  }
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
  if (distance >= 1000.5) {
    Serial.println("Box Detected at ~1 cm!");

    String color = getColor();
    Serial.print("Detected Color: ");
    Serial.println(color);
  }

  delay(500);
}

