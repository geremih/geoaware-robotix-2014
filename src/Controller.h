#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "GeoAware.h"
#include "Locomotor.h"
#include "MapProcessor.h"
#include "LiveSymbolDetector.h"

#define STATE_PANIC -1
#define STATE_START 0
#define STATE_SEARCH_FOR_PATH 1
#define STATE_FOLLOW_LANE 2
#define STATE_TUNNEL_DILEMMA 3
#define STATE_ENTER_TUNNEL 4
#define STATE_COMPARE_SYMBOL 5
#define STATE_RIGHT_TURN 6
#define STATE_LEFT_TURN 7
#define STATE_END 10

#define EVT_LOCALIZED 99
#define EVT_DETECT_SYMBOL 98
#define EVT_DETECT_TUNNEL 97
#define EVT_TRUE 96
#define EVT_FALSE 95
#define EVT_EPS 94
#define EVT_END 93
#define EVT_NONE 92

#define MAX_ATTEMPTS 5

#define AMT_LANE 1
#define AMT_TURN 10

class Controller
{
 public:
  int currState;
  int lastIndex;
  string orientation;
  bool pathFound;
  std::vector<Landmark> path;
  Map m;
  MapProcessor mp;
  LiveSymbolDetector symbolDetector;
  Locomotor *locomotor;
  Controller(string path);
  static Controller* getInstance(MapProcessor inp_mp);
  ~Controller();
  void start();
  
 private:
  void mainLoop();
  int move(string dir, int amt);
  int getNextState(int curr_state, int event);
  int compareSymbol();
  void selectPath();
  int followLane();
  
  static bool instanceFlag;
  static Controller* single;
};

#endif
