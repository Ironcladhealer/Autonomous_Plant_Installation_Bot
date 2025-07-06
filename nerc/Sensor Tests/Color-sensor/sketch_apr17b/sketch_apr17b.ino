#include <Servo.h>

Servo myServo1;
Servo myServo2;  // Create servo object

void setup() {
  myServo1.attach(4);
  myServo2.attach(3); // Attach servo to pin D9
}

void loop() {

//      myServo1.write(180);
 //   myServo2.write(180);
   // delay(1000);
  //      myServo1.write(0);
   // myServo2.write(0);
    //delay(1000);
  // Sweep from 0 to 180 degrees
  for (int pos = 90; pos <= 180; pos += 1) {
    myServo1.write(pos);
    myServo2.write(pos);// Wait for servo to reach the position
  }

  delay(500); // Hold at 180

  // Sweep back from 180 to 0 degrees
  for (int pos = 180; pos >= 90; pos -= 1) {
    myServo1.write(pos);
   myServo2.write(pos);
  }

  delay(500); // Hold at 0*/
}
