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
int rotation = 10;
int extension = 80;
int retraction = 0;
int potPin = A1;
float servoJamThreshold = 12;

void setup() {
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object
}

void print_pos(int pos) {
  Serial.println(pos);
}

void loop() {
  //myservo.write(10);
  //while(1);
  
  for (pos = retraction; pos <= extension; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    Serial.print(pos);
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(60);                       // waits 15ms for the servo to reach the position
    rotation = analogRead(potPin);
    Serial.print(", "); Serial.println(rotation);
    Serial.println(Check_Jam());
    Serial.println();
  }

  delay(1000);
  
  for (pos = extension; pos >= retraction; pos -= 10) { // goes from 180 degrees to 0 degrees
    Serial.print(pos);
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(60);                       // waits 15ms for the servo to reach the position
    rotation = analogRead(potPin);
    Serial.print(", "); Serial.println(rotation);
    Serial.println(Check_Jam());
    Serial.println();
  }

  delay(1000);
}

bool Check_Jam(){
  float realTheta = ((float)rotation)/12; //ALWAYS MAKE SURE TO CALIBRATE THIS!!!
  Serial.print(realTheta+retraction); Serial.print(", ");
  if(fabs(pos-realTheta-retraction) > servoJamThreshold)
    return true;
  else
    return false;
}

