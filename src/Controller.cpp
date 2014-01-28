#include "Controller.h"

bool Controller::instanceFlag = false;

Controller* Controller::single = NULL;

Controller* Controller::getInstance(string path)
{
  if(! instanceFlag)
    {
      single = new Controller(path);
      cout<<"Created new controller instance"<<endl;
      instanceFlag = true;
      return single;
    }
  else
    {
      return single;
    }
}

Controller::Controller(string path):
  lastIndex(0), currState(STATE_START), orientation("?"), pathFound(false), m(path), mp(m), symbolDetector()
{
  m.displayMap();
  m.printLandmarks();
  locomotor = Locomotor::getInstance();
}

void Controller::start()
{
  mainLoop();
}

void Controller::mainLoop()
{
  int event = -1;
  
  while(true)
    {
      switch(currState)
	{
	case STATE_START:
	  break;
	case STATE_FOLLOW_LANE:
	  event = followLane();
	  break;
	case STATE_TUNNEL_DILEMMA:
	  event = tunnelDilemma();
	  break;
	case STATE_COMPARE_SYMBOL:
	  event = compareSymbol();
	  break;
	case STATE_LEFT_TURN:
	  event = move("LEFT", AMT_TURN);
	  break;
	case STATE_RIGHT_TURN:
	  event = move("RIGHT", AMT_TURN);
	  break;
	}
      currState = getNextState(currState,event);
    }
}

int Controller::move(string dir, int amt)
{
  if(dir == "RIGHT")
    locomotor->goRight(amt);
  else if(dir == "LEFT")
    locomotor->goLeft(amt);
  else if(dir == "FORWARD")
    locomotor->goForward(amt);
  else if(dir == "BACKWARD")
    locomotor->goBackward(amt);
  
  if(amt == AMT_TURN)
    sleep(2);

  return EVT_TRUE;
}

int Controller::getNextState(int curr_state, int event)
{
  if(event == EVT_NONE)
    return curr_state;

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

    case STATE_LEFT_TURN:
      return STATE_FOLLOW_LANE;
      
    case STATE_RIGHT_TURN:
      return STATE_FOLLOW_LANE;
      
    default:
      return STATE_PANIC;
    }
}

int Controller::compareSymbol()
{
  // detect the symbol
  // compare with expected symbol
  // return accordingly
  string shape;
  string color;
  cv::Mat frame;
  
  VideoCapture cap(0);
  
  int i = 0;
  while(i<MAX_ATTEMPTS)
    {
      cap >> frame;
      symbolDetector.getSymbol(frame,shape,color);
      if(shape == path[curr_index+1].shape && color == path[curr_index+1].color)
	return EVT_TRUE;
      ++i;
    }
  return EVT_FALSE;  
}

void Controller::selectPath()
{
  // follow the lane until a symbol is detected
  // based on symbol, select the path
  VideoCapture cap(0);
  cv::Mat frame;
  string shape, color;

  // ... add lane follow code here
  // ... stop at a corner
  
  while(!pathFound)
    {      
      cap >> frame;
      symbolDetector.getSymbol(frame,shape,color);
      
      // assumption : each path will have start symbol yellow at index 0
      // what if the landmark reached first is not part of any path ?
      for(int i = 0; i < mp.paths.size(); ++i)
	{
	  if(mp.paths[i].size() >=1 && mp.paths[i][1].shape == shape && mp.paths[i][1].color == color)
	    break;
	}
      if(i < mp.paths.size())
	{
	  path = mp.paths[i];
	  curr_index = 1;
	  //orientation = ...;
	  pathFound = true;
	}
    }  
}

int Controller::followLane()
{ 
  // add lane follow code here
  std::vector<cv::Mat> frames(MAX_ATTEMPTS);
  VideoCapture cap(0);
  int i;
  while(i<MAX_ATTEMPTS)
    cap >> frames[i++];
  
  // pass by ref
  // returns action (detected tunnel, turn to follow lane, detect corner, etc)
  int event = laneFollowDir(frames, dir);
  
  move(dir, AMT_LANE);
 
  // tunnel/symbol ?
}



int main(int argc, char *argv[])
{
  if(argc!=2)
    {
      cout << "Usage : ./GeoAware <path/to/map>" << endl;
      exit(1);
    }
  
  Controller *controller = Controller::getInstance(argv[1]);
  controller->start();
  return 0;
}
