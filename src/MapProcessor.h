#ifndef MAPPROCESSOR_H
#define MAPPROCESSOR_H

#include <iostream>
#include <typeinfo>
#include <cstddef>
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
  int **adjMat; //only considering landmarks, not T-junctions
  cv::Mat img_connections;
  cv::Mat img_route;
  std::vector<Waypoint*> route;
  
  MapProcessor();
  MapProcessor(Map inp_map);
  ~MapProcessor();
  void printLandmarks();
  void findRoute();
  void displayConnections();
  void displayRoute();
  void printRoute();
  
 private:
  std::vector<Landmark> landmarks;
  
  void assignIndices();
  void drawIndices();
  void drawConnections();
  void drawRoute();
  bool isConnected(Landmark l1, Landmark l2);
  bool isConnected(cv::Point2f p1, cv::Point2f p2);
  int ** initAdjMat();
  void fillAdjMat();
  string getDir(string orientation_curr, string orientation_next);
  string getOrient(Waypoint *w1, Waypoint *w2);
  static string convertInt(int number);
};

#endif
