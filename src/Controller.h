#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "GeoAware.h"
#include "Locomotor.h"
#include "MapProcessor.h"
#include "LiveSymbolDetector.h"
#include "CamController.h"
#include <vector>

#define MAX_ATTEMPTS 5

#define AMT_LANE 3
#define AMT_TURN 10

#define LANE_FOLLOW_MIN 75
#define TAKE_TUNNEL 1
#define FOUND_SYMBOL 2
#define NOT_FOUND_SYMBOL 3
#define FOUND_TUNNEL_DONT_TAKE 4
#define FOUND_TUNNEL_AND_TAKE 5
#define NOT_FOUND_TUNNEL 6

class Controller
{
 public:
  int lastIndex;
  string orientation;
  bool pathFound;
  bool tunnelMode;
  string tunnelExitDir;
  std::vector<Landmark> path;
  Map m;
  MapProcessor mp;
  LiveSymbolDetector symbolDetector;
  Locomotor *locomotor;
  CamController *cam;
  Controller(string path , string ACM = "0" , string USB = "0");
  static Controller* getInstance(string mappath, string ACM = "0" , string USB = "0");
  ~Controller();
  void start();
  void facePassage(string passageDir);
  void turnCorner(string passageDir);
 private:

  int no_lane_tries;
  void mainLoop();
  void moveBot(string dir, int amt);
  bool processPassage(string passageDir);
  int detectSymbolController(string& shape, string& color , Point& centroid);
  void selectPath();
  void followLane(int amount , bool& foundLane , bool pmove ) ;
  static VideoCapture cap;
  static bool instanceFlag;
  static Controller* single;
};

#endif
