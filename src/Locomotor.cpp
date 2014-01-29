#include"Locomotor.h"
#include "GeoAware.h"

using namespace std;

bool Locomotor::instanceFlag = false;

bool Locomotor::streamFlag = false;

Locomotor* Locomotor::single = NULL;

ofstream Locomotor::loco_arduino;
ofstream Locomotor::dist_arduino;

Locomotor::Locomotor(){
  //Start Serial Communication
  cout<<"Constructing"<<endl;
}

void Locomotor::writeToDevice( char c){
  if(!streamFlag){
    Locomotor::loco_arduino.open(LOCO_ARDUINO , std::ios_base::app);
    Locomotor::dist_arduino.open(DIST_ARDUINO , std::ios_base::app);
    streamFlag = true;
  }
  Locomotor::loco_arduino<<c;
  loco_arduino.flush();
}

Locomotor * Locomotor::getInstance()
{
  if(! instanceFlag)
    {
      single = new Locomotor();
      cout<<"Created new instance"<<endl;
      instanceFlag = true;
      return single;
    }
  else
    {
      return single;
    }
}

Locomotor::~Locomotor(){
  cout<<"Destructing Locomotor"<<endl;
  loco_arduino.close();
  dist_arduino.close();
  instanceFlag = false;
}

void Locomotor::goLeft(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Left" << endl;
    writeToDevice('a');
  }
}

void Locomotor::goRight(int amount){
  for(int i =0 ; i< amount;i++){
    writeToDevice('d');
    cout<< "Go Right" << endl;
  }
}

void Locomotor::goForward(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Forward" << endl;
    writeToDevice('w');
  }
}

void Locomotor::goBackward(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Backward" << endl;
    writeToDevice('s');

  }
}

void Locomotor::switchToKeyboard(){
  
  int input;
  initscr();
  noecho();
  cbreak();
  while(true){
    input = getch();
    switch(input){
    case 'a':
      goLeft();
      break;
    case 'd':
      goRight();
      break;
    case 's':
      goBackward();
      break;
    case 'w':
      goForward();
      break;
    case 'x':
      return;
      
    }
  }
}


int Locomotor::getDistanceFront(){


}


