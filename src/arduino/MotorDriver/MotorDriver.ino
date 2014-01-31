// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

AF_DCMotor rmotor(4);
AF_DCMotor lmotor(3);


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Starting Arduino");
  // turn on motor
  lmotor.setSpeed(200);
  rmotor.setSpeed(200);
  lmotor.run(RELEASE);
  rmotor.run(RELEASE);
}

void loop() {
  //This delay can be set accordingly
  //delay(1000);
}

//Set appropriate delay
// Option of starting motor by stepping speeds

void goForward(){
  uint8_t i;
  Serial.print("Forward");
  rmotor.run(FORWARD);
  lmotor.run(FORWARD);
  rmotor.setSpeed(255);
  lmotor.setSpeed(255);  

  delay(25);
  lmotor.run(RELEASE);
  rmotor.run(RELEASE);
  
}

void goBackward(){
  uint8_t i;
  Serial.print("Backward");

  rmotor.run(BACKWARD);
  lmotor.run(BACKWARD);
 

  rmotor.setSpeed(255);
  lmotor.setSpeed(255);  

  delay(25);
  lmotor.run(RELEASE);
  rmotor.run(RELEASE);
}

void goLeft(){

  uint8_t i;
  Serial.print("Left");

  rmotor.run(FORWARD);
  lmotor.run(BACKWARD);
  rmotor.setSpeed(255);
  lmotor.setSpeed(255);  

  delay(25);

  lmotor.run(RELEASE);
  rmotor.run(RELEASE);
  
}


void gradualLeft(){
  uint8_t i;
  Serial.print("Gradual Left");

  rmotor.run(FORWARD);
  lmotor.run(FORWARD);
  rmotor.setSpeed(255);
  lmotor.setSpeed(150);  

  delay(25);

  lmotor.run(RELEASE);
  rmotor.run(RELEASE);
  
  
}





void gradualRight(){
  uint8_t i;
  Serial.print("Gradual Right");

  rmotor.run(FORWARD);
  lmotor.run(FORWARD);
  rmotor.setSpeed(150);
  lmotor.setSpeed(255);  

  delay(25);

  lmotor.run(RELEASE);
  rmotor.run(RELEASE);
  
}



void goRight(){

  uint8_t i;
  Serial.print("Right");

  rmotor.run(BACKWARD);
  lmotor.run(FORWARD);
  rmotor.setSpeed(255);
  lmotor.setSpeed(255);  

  delay(25);
  lmotor.run(RELEASE);
  rmotor.run(RELEASE);
}




void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    switch (inChar) {
    case 'w':
      goForward();
      break;
    case 's':
      goBackward();
      break;
    case 'd':
      goRight();
      break;
    case 'z':
      gradualLeft();
      break;
    case 'c':
      gradualRight();
      break;
    case 'a':
      goLeft();
      break;

      
    }
  }
}


