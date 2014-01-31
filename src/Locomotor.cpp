#include"Locomotor.h"
#include "GeoAware.h"
#include<stdlib.h>

#define LANE_WIDTH 40
#define MOVE_SLEEP_TIME 10000
using namespace std;

using namespace cv;
bool Locomotor::instanceFlag = false;

bool Locomotor::streamFlag = false;

Locomotor* Locomotor::single = NULL;

std::ofstream Locomotor::loco_arduino_out;
std::ofstream Locomotor::dist_arduino_out;
std::ifstream Locomotor::dist_arduino_in;


Locomotor::Locomotor(){
  //Start Serial Communication
  cout<<"Constructing"<<endl;
  currentPos= "NONE";
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
  usleep(MOVE_SLEEP_TIME);
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

  usleep(MOVE_SLEEP_TIME);
}

void Locomotor::goBackward(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Go Backward" << endl;
    writeToLoco('s');
  }
  usleep(MOVE_SLEEP_TIME);
}


void Locomotor::gradualLeft(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Grad Left" << endl;
    writeToLoco('z');
  }
  usleep(MOVE_SLEEP_TIME);
 
  
}

void Locomotor::gradualRight(int amount){
  for(int i =0 ; i< amount;i++){
    cout<< "Grad Right" << endl;
    writeToLoco('c');
  }
  usleep(MOVE_SLEEP_TIME);
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
  if( currentPos != "LEFT")
    writeToDist('a');
  currentPos = "LEFT";
}

void Locomotor::servoRight(){
  if( currentPos != "RIGHT")
    writeToDist('d');
  currentPos = "RIGHT";
}

void Locomotor::servoFront(){
  if( currentPos != "FRONT")
    writeToDist('w');
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

void Locomotor::facePassage(string passageDir){

  int distance , curr_distance;
  int vote = 0;
  if (passageDir == "RIGHT"){

    while(vote < 3){
      curr_distance = getDistanceRight();
      if(curr_distance> 2.5 *LANE_WIDTH)
        vote++;
      goForward();
     
    }
    time_t init = time(NULL);

    while(time(NULL) - init < GRAD_RIGHT_TIME){
      gradualRight();

    }
  }

  
  
  


}

void Locomotor::switchToKeyboard(){
  
  int input;
  initscr();
  noecho();
  cbreak();

  // Open the default camera
  VideoCapture capture(0); 

  // Get the properties from the camera
  double width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
  double height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
 
  cout << "Camera properties\n";
  cout << "width = " << width << endl <<"height = "<< height << endl;
 
  // Create a matrix to keep the retrieved frame
  Mat frame;
 
  // Create a window to show the image
  namedWindow ("Capture", CV_WINDOW_AUTOSIZE);


  // Create the video writer
  VideoWriter video("capture.avi",capture.get(CV_CAP_PROP_FOURCC),  capture.get(CV_CAP_PROP_FPS), cvSize((int)width,(int)height) );


  while(true){

      capture >> frame;
 
      // Save frame to video
      video << frame;
 
      // Show image
      imshow("Capture", frame);

    
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
      
    }
  }
}


