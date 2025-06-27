#include <Servo.h>


Servo myservo;  // create servo object to control a servo
char command;




int angle = 0;    // variable to store the servo position


void setup() {
  myservo.attach(6);  // attaches the servo on pin 6 to the servo object
  Serial.begin(9600);
  Serial.println("Send 'q'");
  myservo.write(300);   // Move to 0 degrees


}


void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();


    if (command == 'q') {
        Serial.println("Launch");
        myservo.write(900);    
        delay(1000);        // tell servo to go to position in variable 'angle'
        myservo.write(0);
        delay(1000);  
        myservo.write(900);    


    }
  }
}    