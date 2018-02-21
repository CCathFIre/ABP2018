/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
bool edge=false;
const int inPin = 9;

void setup() {
  pinMode(inPin, INPUT);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  delay(100);
  myservo.write(0);
}

void loop() {
  if(digitalRead(inPin) && !edge)){
    pos++;
    myservo.write(pos);
    edge = true;
    delay(25);
  }

  if(!digitalRead(inPin)){
    edge=false;
  }
}

