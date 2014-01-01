#include "GeoAware.h"
#include "Landmark.h"
#include "Map.h"
#include "MapProcessor.h"

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
  if(argc!=2)
    {
      cout << "Usage : ./GeoAware <path/to/map>" << endl;
      return 1;
    }
  
  const string path(argv[1]);
  cout << path << endl;
  Map m(path);
  //m.displayMap();
  
  MapProcessor mp(m);
  //mp.printLandmarks();
  mp.displayConnections();
  mp.displayRoute();
  mp.printRoute();

  cv::waitKey(0);
  return 0;
}
