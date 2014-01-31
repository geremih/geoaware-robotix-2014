#include "CamController.h"
#include "Locomotor.h"

#define IMSHOW 1
using namespace cv;

int main(int argc , char **argv){

  CamController cp;

  string argp = argv[1];
  VideoCapture capture(atoi(argp.c_str()));

  Mat frame;
  // if 2 successive frames detect same symbol, return it
  Locomotor * locomotor;
  time_t init = time(NULL);
  locomotor = Locomotor::getInstance();

  while(1){
    bool left ,right;

    capture>>frame;
    init = time(NULL);
    
    cp.processVideo( frame , "TUNNEL" , left ,right);
    
    cp.processVideo(frame , "LANE" , left , right);

    if( left == true){
      
      locomotor->goLeft(5);
    }
    else if (right == true){
      locomotor->goRight(5);
    }
    else{
      locomotor->goForward(5);
    }
    waitKey(100);
  }  

  return 0;
}



