#include"Locomotor.h"
#include "GeoAware.h"
#include<stdlib.h>
#include <string.h>
#define LANE_WIDTH 40
#define BOWSER_LENGTH 23


//in micro seconds 

#define MOVE_SLEEP_TIME 50000
#define SERVO_SLEEP_TIME 1000000
#define PING_WAIT_TIME 50000


#define GRAD_LEFT_TIME 1
#define GRAD_RIGHT_TIME 1
#define PATH_HISTORY_SIZE 50
//no of iterations
#define UTURN_AMOUNT 28

#define SERIALIDX_MIN  0
#define SERIALIDX_MAX  10


using namespace std;
using namespace cv;

bool Locomotor::instanceFlag = false;

bool Locomotor::streamFlag = false;

Locomotor* Locomotor::single = NULL;

char LOCO_ARDUINO[14] = "/dev/ttyUSB";
char  DIST_ARDUINO[14] = "/dev/ttyACM";

std::ofstream Locomotor::loco_arduino_out;
std::ofstream Locomotor::dist_arduino_out;
std::ifstream Locomotor::dist_arduino_in;


void Locomotor::addToPathHistory (string dir){
  if(path_history.size() > PATH_HISTORY_SIZE)
    path_history.pop_front();
  path_history.push_back( dir);
}

Locomotor::Locomotor(  string ACM ,  string USB){
  //Start Serial Communication
  cout<<"Constructing"<<endl;
  currentPos= "NONE";
  strcat(LOCO_ARDUINO, USB.c_str()) ;
  strcat(DIST_ARDUINO, ACM.c_str()) ;
  cout<<"CURRENTLY USING " <<  DIST_ARDUINO << " AND " << LOCO_ARDUINO<<endl;
  if(!streamFlag)
    {    
      Locomotor::loco_arduino_out.open(LOCO_ARDUINO , std::ios_base::app);
      Locomotor::dist_arduino_out.open(DIST_ARDUINO , std::ios_base::app);
      Locomotor::dist_arduino_in.open(DIST_ARDUINO , std::ios_base::app);
      streamFlag = true;
    }

  //arduino setup
  sleep(5);
}

void Locomotor::writeToLoco( char c){
  Locomotor::loco_arduino_out<<c;
  loco_arduino_out.flush();
}

void Locomotor::writeToDist( char c){
  Locomotor::dist_arduino_out<<c;
  dist_arduino_out.flush();
}


Locomotor * Locomotor::getInstance(string ACM , string USB)
{
  if(! instanceFlag)
    {
      single = new Locomotor(ACM , USB);
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
    addToPathHistory("LEFT");
    writeToLoco('a');
    usleep(MOVE_SLEEP_TIME); 
  }
}

void Locomotor::goRight(int amount){
  for(int i =0 ; i< amount;i++){
    writeToLoco('d');
    addToPathHistory("RIGHT");
    cout<< "Go Right" << endl;
    usleep(MOVE_SLEEP_TIME); 
      
  }
}

void Locomotor::goForward(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Forward" << endl;
    addToPathHistory("FORWARD");
    writeToLoco('w');
    usleep(MOVE_SLEEP_TIME); 
  }

}

void Locomotor::goBackward(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Backward" << endl;
    addToPathHistory("BACKWARD");
    writeToLoco('s');
    usleep(MOVE_SLEEP_TIME);
  }
}


void Locomotor::gradualLeft(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Grad Left" << endl;
    writeToLoco('z');
    usleep(MOVE_SLEEP_TIME);
  }

}




void Locomotor::gradualRight(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Grad Right" << endl;
    writeToLoco('c');
    usleep(MOVE_SLEEP_TIME);
  }
}

vector<string> Locomotor::windBack(int amt = 5)
{
  string dir;
  vector<string> retrace;
  int i = 0;
  while(path_history.size() > 0 && i < amt){
    i++;
    dir = path_history.back();
    path_history.pop_back();
    
 
    string opp_dir;

    if(dir == "LEFT")
      opp_dir = "RIGHT";

    if(dir == "RIGHT")
      opp_dir = "LEFT";

    if(dir == "FORWARD"){
      opp_dir = "BACKWARD";
     
    }
    retrace.push_back(opp_dir);
  }

  return retrace;

}


void Locomotor::goUTurn( int amount){
  goRight( UTURN_AMOUNT);

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
  
  usleep(PING_WAIT_TIME);
  std::ofstream file_out;
  file_out.open("Dist" , std::ios_base::app);
  dist_arduino_in >>cm;	//will set the          error flag if not ready, will get a number from the Arduino stream if ready
  file_out<< atoi( cm.c_str())<<endl;
  cout << atoi(cm.c_str()) << endl;	//Output it to the cout         stream

  dist_arduino_in.clear();	//eof flag won't clear itself
  file_out.close();
  return( atoi(cm.c_str()));
}

void Locomotor::servoLeft(){
  if( currentPos != "LEFT"){
    writeToDist('a');
    usleep(SERVO_SLEEP_TIME);
  }
  currentPos = "LEFT";
}

void Locomotor::servoRight(){
  if( currentPos != "RIGHT"){
    writeToDist('d');
    usleep(SERVO_SLEEP_TIME);
  }
  currentPos = "RIGHT";
}

void Locomotor::servoFront(){
  if( currentPos != "FRONT"){
    writeToDist('w');
    usleep(SERVO_SLEEP_TIME);
  }
  currentPos = "FRONT";
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


  servoRight();

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
    case 'z':
      gradualLeft();
      break;
    case 'c':
      gradualRight();
      break;
    case 'i':
      servoFront();
      break;
    case 'j':
      servoLeft();
      break;
    case 'l':
      servoRight();
      break;

    case 'p':
      getDistance();
      break;
    case 'x':
      return;
    case 'g':
      //facePassage("LEFT");
      break;
    case 'h':
      //facePassage("RIGHT");
      break;
    case 'u':
      goUTurn(UTURN_AMOUNT);
      break;
      
    }
  }
}


