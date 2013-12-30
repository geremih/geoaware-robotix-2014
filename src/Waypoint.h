#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "GeoAware.h"

using namespace std;
using namespace cv;

class Waypoint
{
 public:
  cv::Point2f centroid;
  int start,end;
  int action;
  int idx;
  
  Waypoint();
  Waypoint(cv::Point2f ctr, int st, int en);
};

#endif
