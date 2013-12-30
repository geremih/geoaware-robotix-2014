#ifndef MAPPROCESSOR_H
#define MAPPROCESSOR_H

#include <iostream>
#include "GeoAware.h"
#include "Landmark.h"
#include "TJunction.h"
#include "Map.h"

using namespace std;
using namespace cv;

class MapProcessor
{
public:
  Map m;
  int size;
  int **adjMat;
  cv::Mat img_connections;
  std::vector<Waypoint*> route;
  
  MapProcessor();
  MapProcessor(Map inp_map);
  ~MapProcessor();
  void printLandmarks();
  void findRoute();
  void displayConnections();
  
 private:
  std::vector<Landmark> landmarks;
  
  void assignIndices();
  void drawIndices();
  void drawConnections();
  bool isConnected(Landmark l1, Landmark l2);
  int ** initAdjMat();
  void fillAdjMat();
  
  static string convertInt(int number);
};

#endif
