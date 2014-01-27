#include "Controller.h"

Controller::Controller(string path):
  curr_index(0), curr_state(STATE_START), orientation("?"), pathFound(false), m(path), mp(m), symbolDetector()
{
  m.displayMap();
  m.printLandmarks();
}

int Controller::getNextState(int curr_state, int event)
{
  switch(curr_state)
    {
    case STATE_START:
      return STATE_FOLLOW_LANE;
    case STATE_FOLLOW_LANE:
      if(event==EVT_DETECT_TUNNEL)
	return STATE_TUNNEL_DILEMMA;
      else if(event == EVT_DETECT_SYMBOL)
	return STATE_COMPARE_SYMBOL;
      else if(event == EVT_END)
	return END;
    case STATE_TUNNEL_DILEMMA:
      if(action == EVT_TRUE)
	return STATE_ENTER_TUNNEL;
      else if(action == EVT_FALSE)
	return STATE_FOLLOW_LANE;
    case STATE_ENTER_TUNNEL:
      return STATE_FOLLOW_LANE;
    case STATE_COMPARE_SYMBOL:
      if(event == EVT_TRUE)
	return STATE_TURN;
      else
	return STATE_PANIC;
    case STATE_TURN:
      return STATE_FOLLOW_LANE;
    default:
      return STATE_PANIC;
    }
}

bool compareSymbol()
{
  // detect the symbol
  // compare with expected symbol
  // return accordingly
  string shape;
  string color;
  cv::Mat frame;
  
  VideoCapture cap(0);
  cap >> frame;
  
  symbolDetector.getSymbol(frame,shape,color);
  
}
