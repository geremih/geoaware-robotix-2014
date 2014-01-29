#ifndef LOCOMOTOR_H
#define LOCOMOTOR_H

#include <fstream>
#include<iostream>
#include<curses.h>
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
  void switchToKeyboard();
  int getDistanceFront();
  int getDistanceLeft();
  int getDistanceRight();
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
