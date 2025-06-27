#include <Servo.h>


Servo myServo;
char command;


const int home = 180;       // Home position
const int offset = 50;     // Sweep range


void setup() {
 myServo.attach(9);
 myServo.write(home);     // Move servo to home position
 Serial.begin(9600);
 Serial.println("Send 'q', 'w'");
}


void loop() {
 if (Serial.available() > 0) {
   command = Serial.read();


   if (command == 'q') {
     // Sweep from home to home + 50 degrees
     for (int angle = home; angle <= home + offset; angle++) {
       myServo.write(angle);
       delay(10);
     }
   } else if (command == 'w') {
     // Sweep from home to home - 50 degrees
     for (int angle = home; angle >= home - offset; angle--) {
       myServo.write(angle);
       delay(10);
     }
     
 






   }
 }
}


