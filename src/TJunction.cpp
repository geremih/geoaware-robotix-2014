#include "TJunction.h"

using namespace std;

TJunction::TJunction()
{
}

TJunction::TJunction(cv::Point2f ctr, int index)
  :  Waypoint(ctr,0,0, index)
{
}

const TJunction& TJunction::operator =(const TJunction& T)
{
  centroid = T.centroid;
  start = T.start;
  end = T.end;
  idx = T.idx;
  action = T.action;
  return *this;
}

ostream& operator <<(ostream& stream, const TJunction& T)
{
  if(T.start)
    stream << "[START]" << endl;
  if(T.end)
    stream << "[END]" << endl;
  
  stream << "Index : " << T.idx << endl;
  stream << "Center : " << T.centroid << endl;
  
  return stream;
}
