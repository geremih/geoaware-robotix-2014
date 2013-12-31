#ifndef LANDMARK_H
#define LANDMARK_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include "Waypoint.h"

using namespace std;

class Landmark : public Waypoint
{

 public:
  string shape;
  string color;
  
  Landmark();
  Landmark(cv::Point2f, string sh , string clr , int st, int en);
  Landmark(const Landmark&);
  const Landmark& operator = (const Landmark&);
  friend ostream& operator<< (ostream& stream, const Landmark& landmark);
};

#endif
