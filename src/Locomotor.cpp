#include"Locomotor.h"
#include "GeoAware.h"
#include<stdlib.h>
using namespace std;

bool Locomotor::instanceFlag = false;

bool Locomotor::streamFlag = false;

Locomotor* Locomotor::single = NULL;

std::ofstream Locomotor::loco_arduino_out;
std::ofstream Locomotor::dist_arduino_out;
std::ifstream Locomotor::dist_arduino_in;


Locomotor::Locomotor(){
  //Start Serial Communication
  cout<<"Constructing"<<endl;
  if(!streamFlag){
    Locomotor::loco_arduino_out.open(LOCO_ARDUINO , std::ios_base::app);
    Locomotor::dist_arduino_out.open(DIST_ARDUINO , std::ios_base::app);
    Locomotor::dist_arduino_in.open(DIST_ARDUINO , std::ios_base::app);
    streamFlag = true;
  }
}

void Locomotor::writeToLoco( char c){
  Locomotor::loco_arduino_out<<c;
  loco_arduino_out.flush();
}

void Locomotor::writeToDist( char c){
  Locomotor::dist_arduino_out<<c;
  dist_arduino_out.flush();
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
  loco_arduino_out.close();
  dist_arduino_out.close();
  dist_arduino_in.close();
  instanceFlag = false;
}

void Locomotor::goLeft(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Left" << endl;
    writeToLoco('a');
  }
}

void Locomotor::goRight(int amount){
  for(int i =0 ; i< amount;i++){
    writeToLoco('d');
    cout<< "Go Right" << endl;
  }
}

void Locomotor::goForward(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Forward" << endl;
    writeToLoco('w');
  }
}

void Locomotor::goBackward(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Backward" << endl;
    writeToLoco('s');
  }
}

int Locomotor::getDistance(){

  /*
    system("stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel
    -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke
    noflsh -ixon -crtscts");	//Activates the tty connection with the Arduino
  */
	string cm;	
	long int Time = time(NULL);

  //polling
  dist_arduino_in.clear();	//eof flag won't clear itself
  dist_arduino_out<<"p"<<endl;
  
  while(time(NULL)-Time < 1){}	//Wait one seconds for the Arduino  to start up

  dist_arduino_in >>cm;	//will set the          error flag if not ready, will get a number from the Arduino stream if ready
  cout << atoi(cm.c_str()) << endl;	//Output it to the cout         stream
  dist_arduino_in.clear();	//eof flag won't clear itself

  return(0);
}

void Locomotor::servoLeft(){

  writeToDist('a');
}

void Locomotor::servoRight(){

  writeToDist('d');
}

void Locomotor::servoFront(){

  writeToDist('w');
}

int Locomotor::getDistanceFront(){
  servoFront();
  return getDistance();

}
int Locomotor::getDistanceLeft(){
  servoLeft();
  return getDistance();
}

int Locomotor::getDistanceRight(){
  servoRight();
  return getDistance();
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


