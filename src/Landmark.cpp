#include "Landmark.h"

using namespace std;

Landmark::Landmark()
{
  centroid = cv::Point2f(0.f,0.f);
  shape = "None";
  color = "None";
  start = -1;
  end = -1;
  action = -1;
  idx = -1;
}

Landmark::Landmark(cv::Point2f ctr, string sh, string clr, int st, int en)
{
  centroid = ctr;
  shape = sh;
  color = clr;
  start = st;
  end = en;
  idx = -1;
  action = -1;
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
