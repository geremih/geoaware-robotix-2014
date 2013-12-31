#ifndef TJUNCTION_H
#define TJUNCTION_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include "Waypoint.h"

using namespace std;

class TJunction : public Waypoint
{

 public:
  bool UD,LR;
  
  TJunction();
  TJunction(cv::Point2f ctr,int st, bool ud, bool lr);
  TJunction(const TJunction&);
  const TJunction& operator = (const TJunction&);
  friend ostream& operator<< (ostream& stream, const TJunction& T);
};

#endif
