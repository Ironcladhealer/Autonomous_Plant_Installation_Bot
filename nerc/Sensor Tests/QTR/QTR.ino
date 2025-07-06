#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog(); // Use QTR-8A analog sensors
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);
  
  Serial.println("Calibrating...");
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(10);
  }
  Serial.println("Calibration complete");
}


void loop() {
  qtr.read(sensorValues);

  for (uint8_t i = 0; i < sensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(i < sensorCount - 1 ? "\t" : "\n");
  }

  delay(250);
}
