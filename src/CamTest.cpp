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
  locomotor = Locomotor::getInstance();
  int i = 0;
  while(1){
    bool left ,right;

    for(int i=0;i<5;++i)
      capture>>frame;
    int backward =0;
    cp.processVideo( frame , "TUNNEL" , left ,right);
    //cp.processVideo(frame , "LANE" , left , right);

    if( left == true )
      {
	cout<<"Tunnel on Left"<<endl;
	//locomotor->goLeft(5);

      }
    
    else if (right == true )
      {
	cout<<"Tunnel on Right"<<endl;
	//locomotor->goRight(5);
      }
    else
      {
	cout<<"No tunnel"<<endl;
	//locomotor->goForward(5);
      }
    
    imshow("test",frame);
    waitKey(20);
  }  

  return 0;
}



