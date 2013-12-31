#include "TJunction.h"

using namespace std;

TJunction::TJunction()
{
  Waypoint();
  UD = false;
  LR = false;
}

TJunction::TJunction(cv::Point2f ctr, int st, bool ud, bool lr)
  :  Waypoint(ctr,st,0)
{
  UD = ud;
  LR = lr;
}

const TJunction& TJunction::operator =(const TJunction& T)
{
  centroid = T.centroid;
  start = T.start;
  end = T.end;
  idx = T.idx;
  action = T.action;
  UD = T.UD;
  LR = T.LR;
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
  stream << "UD : " << T.UD << endl;
  stream << "LR : " << T.LR << endl;
  
  return stream;
}
