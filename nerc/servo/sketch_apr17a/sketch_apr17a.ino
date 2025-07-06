#include <Servo.h>

Servo myServo1;
//Servo myServo2; // Create servo object

void setup() {
  myServo1.attach(11); 
//  myServo2.attach(10);// Attach servo to pin D9
}

void loop() {
  //High points
 // myServo1.write(0);
 
  // Sweep from 0 to 180 degrees
  for (int pos = 0; pos <= 180; pos += 1) {
    myServo1.write(pos);
   // myServo2.write(pos);
    delay(15);
  }
  
delay(1000);  // Hold at 180
for (int pos = 180; pos <= 0; pos -= 1) {
    myServo1.write(pos);
   // myServo2.write(pos);
    delay(15);  // Wait for servo to reach the position
  }
delay(1000);
  // Sweep back from 180 to 0 degrees
  

 /*
  for (int pos = 0; pos <= 180; pos += 1) {
    myServo2.write(pos);
   // myServo2.write(pos);
    delay(15);
  }
  
delay(1000);  // Hold at 180
for (int pos = 180; pos <= 0; pos -= 1) {
    myServo2.write(pos);
   // myServo2.write(pos);
    delay(15);  // Wait for servo to reach the position
  }
delay(1000);
  /*
myServo2.write();
  // Sweep from 0 to 180 degrees
  for (int pos = 180; pos <= 100; pos -= 1) {
    myServo2.write(pos);
   // myServo2.write(pos);
    delay(15);  // Wait for servo to reach the position
  }

 delay(500);  // Hold at 180

  // Sweep back from 180 to 0 degrees
  for (int pos = 100; pos >= 180; pos += 1) {
    myServo2.write(pos);
   // myServo2.write(pos);
    delay(15);
  }
  dela(500);

/*
  delay(500);  // Hold at 0
    for (int pos = 0; pos <= 120; pos += 1) {
    myServo2.write(pos);
   // myServo2.write(pos);
    delay(15);  // Wait for servo to reach the position
  }

  delay(500);  // Hold at 180

  // Sweep back from 180 to 0 degrees
  for (int pos = 120; pos >= 0; pos -= 1) {
    myServo1.write(pos);
   // myServo2.write(pos);
    delay(15);
  }

  delay(500);  // Hold at 0
*/
}
