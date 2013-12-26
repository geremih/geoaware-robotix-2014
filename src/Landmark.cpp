#include "GeoAware.h"

class Landmark
{
  cv::Point2f centroid;
  int shape;
  int color;
  int start,end;
  int action;

public:
  Landmark(cv::Point2f, int, int, int , int);
  void setAction(int);
  
};

Landmark::Landmark(cv::Point2f ctr, int sh, int clr, int st, int en)
{
  centroid = ctr;
  shape = sh;
  color = clr;
  start = st;
  end = en;
}

void Landmark::setAction(int a)
{
  action = a;
}
