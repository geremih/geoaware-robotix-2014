#ifndef LOCOMOTOR_H
#define LOCOMOTOR_H

#include <fstream>
#include<iostream>
#include<curses.h>

//#define LOCO_ARDUINO "/dev/ttyUSB0"
//#define DIST_ARDUINO "/dev/ttyACM0"

using namespace std;

class Locomotor
{  
 public:
  static std::ofstream loco_arduino_out;
  static std::ofstream dist_arduino_out;
  static std::ifstream dist_arduino_in;
  void goLeft(int amount = 1);
  void goRight(int amount =1 );
  void goForward(int amount = 1);
  void goBackward(int amount = 1);
  void goUTurn( int amount=1){
  void gradualLeft(int amount = 1);
  void gradualRight(int amount = 1);
  void switchToKeyboard();
  int getDistanceFront();
  int getDistanceLeft();
  int getDistanceRight();
  string currentPos;
  void facePassage(string passageDir);
  static Locomotor* getInstance();
  Locomotor();
  Locomotor(const Locomotor&);
  ~Locomotor();

  
 private:
  void writeToLoco(char c);
  void writeToDist( char c);
  int getDistance();
  void servoLeft();
  void servoRight();
  void servoFront();
  static bool instanceFlag;
  static Locomotor *single;
  static bool streamFlag;
};

#endif 
