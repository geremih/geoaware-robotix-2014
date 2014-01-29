#ifndef CAMCONTROLLER_H
#define CAMCONTROLLER_H
#include "GeoAware.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>


using namespace std;
#define PI 3.1415926

class CamController{

 public:
  string laneFollowDir(vector<cv::Mat> frames); // return direction
  void isPassage(vector<cv::Mat> frames , bool& pLeft,bool& pRight);

  



};


#endif
