#ifndef LANDMARK_H
#define LANDMARK_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace std;

class Landmark
{

 public:

  cv::Point2f centroid;
  string shape;
  string color;
  int start,end;
  int action;

  Landmark(cv::Point2f, string sh , string clr , int st, int en);
  Landmark(const Landmark&);
  const Landmark& operator = (const Landmark&);
  friend ostream& operator<< (ostream& stream, const Landmark& landmark);
  void setAction(int); 
  void printInfo();

};

#endif
