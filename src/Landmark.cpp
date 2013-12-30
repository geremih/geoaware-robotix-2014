#include "Landmark.h"

using namespace std;

Landmark::Landmark()
{
  //calls base class constructor by default
  shape = "None";
  color = "None";
}

Landmark::Landmark(cv::Point2f ctr, string sh, string clr, int st, int en)
  :  Waypoint(ctr,st,en)
{
  shape = sh;
  color = clr;
}

Landmark::Landmark(const Landmark& L)
{
  centroid = L.centroid;
  shape = L.shape;
  color = L.color;
  start = L.start;
  end = L.end;
  idx = L.idx;
  action = L.action;
}

const Landmark &Landmark::operator = (const Landmark &L)
{
  centroid = L.centroid;
  shape = L.shape;
  color = L.color;
  start = L.start;
  end = L.end;
  idx = L.idx;
  action = L.action;
  return *this;
}

ostream& operator <<(ostream& stream, const Landmark& landmark)
{
  if(landmark.start)
    stream << "[START]" << endl;
  if(landmark.end)
    stream << "[END]" << endl;
  
  stream << "Index : " << landmark.idx << endl;
  stream << "Center : " << landmark.centroid << endl;
  stream << "Shape : " << landmark.shape << endl;
  stream << "Color : " << landmark.color << endl;
  
  return stream;
}
