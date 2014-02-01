#ifndef CAMCONTROLLER_H
#define CAMCONTROLLER_H
#include "GeoAware.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>


using namespace std;
using namespace cv;
#define PI 3.1415926

class CamController{

 public:
  static string laneFollowDir(Mat frame); // return "LEFT" , "RIGHT" , "BACKWARD"
  static void isPassage( Mat frame ,bool& pLeft,bool& pRight); 
  static void processVideo(Mat image , string type , bool& pLeft , bool& pRight, bool& lane);
  static void  detectTunnel(vector<Vec6f> segments , bool& pLeft , bool& pRight);
};


#endif
