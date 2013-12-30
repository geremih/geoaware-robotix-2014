#include "Waypoint.h"

using namespace std;

Waypoint::Waypoint()
{
  centroid = cv::Point2f(0.f,0.f);
  start = -1;
  end = -1;
  action = -1;
  idx = -1;
}

Waypoint::Waypoint(cv::Point2f ctr, int st, int en)
{
  centroid = ctr;
  start = st;
  end = en;
  action = -1;
  idx = -1;
}
