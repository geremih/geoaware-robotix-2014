#include "GeoAware.h"
#include "Landmark.h"
#include "Map.h"

using namespace std;
using namespace cv;

int main(void)
{
  const string path = "../assets/map1.png";
  Map m(path);
  m.printLandmarks();
  m.displayMap();
  
  cv::waitKey(0);
  return 0;
}
