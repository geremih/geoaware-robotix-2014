// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position
const int PIN_echo = 10, PIN_Trig = 11;

 
void setup() 
{ 
  Serial.begin(9600);
  Serial.println("starting");
  myservo.attach(8);  // attaches the servo on pin 9 to the servo object
  pinMode(PIN_Trig, OUTPUT);
  pinMode(PIN_echo, INPUT);
  
} 
 
 
void loop() 
{ 
  getDistance();
  delay(150);
}


long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


void getDistance( ){
  long duration, inches, cm;
  // The PING))) is triggered by a HIGH pulse of 10 microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(PIN_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_Trig, LOW);
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(PIN_echo, HIGH);
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
}

void goToPos( int pos){
  
  myservo.write(pos);              // tell servo to go to position in variable 'pos' 
  delay(100);                       // waits 15ms for the servo to reach the position 
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    switch (inChar) {
    case 'w':
      goToPos(90);
      break;
    case 'd':
      goToPos(0);
      break;
    case 'a':
      goToPos(180);
      break;
    }
  }
}


