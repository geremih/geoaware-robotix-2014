#include "GeoAware.h"
#include "Landmark.h"
#include "Map.h"
#include "MapProcessor.h"

using namespace std;
using namespace cv;

int main(void)
{
  const string path = "../assets/map1.png";
  Map m(path);
  m.displayMap();
  
  MapProcessor mp(m);
  mp.printLandmarks();
  mp.displayConnections();

  cv::waitKey(0);
  return 0;
}
