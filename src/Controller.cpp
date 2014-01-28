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

void Controller::testLoop()
{
  bool flag;
  bool tunnelMode = false;
  string tunnelExitDir;
  string passageDir;
  int distance_front; //cm
  
  std::vector<Landmark> newpath;
  
  while(1)
    {
      distance_front = locomotor->getDistanceFront();
      
      if(distance_front > LANE_FOLLOW_MIN)
	followLane();
      else
	move("STRAIGHT",AMT_LANE);
      
      CamController::isPassage(pLeft,pRight);
      
      if(pLeft)
	{
	  passageDir = "LEFT";
	  newpath.clear();
	  // tunnel dilemma sends newpath consistent with lastIndex
	  if(mp.tunnelDilemma(passageDir,path,lastIndex,newpath,tunnelExitDir) == FOUND_TUNNEL)
	    {
	      tunnelMode = 1;
	      // traverse tunnel
	      // takepassage will align and turn to face the passage
	      takePassage(passageDir);
	      path = newpath;
	    }
	  else
	    {
	      // found a TJ
	      if(pathFound == false)
		{
		  //path = ... iterate paths
		  // update orientation during iteration
		  orientation = MapProcessor::getOrient(path[lastIndex],path[lastIndex+1]);
		  pathFound = true;
		}
	      orientation_next = MapProcessor::getOrient(path[lastIndex+1],route[lastIndex+2]);
	      if(orientation_next == "LEFT")
		takePassage(passageDir);
	      ++lastIndex;
	    }
	}
      if(pRight)
	{
	  passageDir = "RIGHT";
	  newpath.clear();
	  // tunnel dilemma sends newpath consistent with lastIndex
	  if(mp.tunnelDilemma(passageDir,path,lastIndex,newpath,tunnelExitDir) == FOUND_TUNNEL)
	    {
	      tunnelMode = 1;
	      // traverse tunnel
	      // takepassage will align and turn to face the passage
	      takePassage(passageDir);
	      path = newpath;
	    }
	  else
	    {
	      // found a TJ
	      if(pathFound == false)
		{
		  // TODO
		  //path = ... iterate paths
		  // update orientation during iteration
		  orientation = MapProcessor::getOrient(path[lastIndex],path[lastIndex+1]);
		  pathFound = true;
		}
	      orientation_next = MapProcessor::getOrient(path[lastIndex+1],path[lastIndex+2]);
	      if(orientation_next == "RIGHT")
		takePassage(passageDir);
	      ++lastIndex;
	    }
	}
      if(distance_front < LANE_FOLLOW_MIN)
	{
	  if(tunnelMode)
	    {
	      // exiting the tunnel
	      move(tunnelExitDir,AMT_TURN);
	      tunnelMode = false;
	    }
	  else
	    {
	      // reached a corner (in case of TJ, we would have already turned)
	      // if the next symbol is end
	      if(lastIndex == path.size() - 2)
		{
		  cout << "DONE!" << endl;
		  exit(0);
		}
	      
	      flag = compareSymbol();
	      if(flag == FOUND_SYMBOL)
		{
		  if(pathFound == false)
		    {
		      //path = ... iterate paths
		      // update orientation during iteration
		      orientation = MapProcessor::getOrient(path[lastIndex],path[lastIndex+1]);			      
		      pathFound = true;
		    }
		  else
		    {
		      // turn in reqd dir and upd orientation
		      orientation_next = getOrient(path[lastIndex+1],path[lastIndex+2]);
		      direction = getDir(orientation,orientation_next);
		      move(direction,AMT_TURN);
		      orientation = orientation_next;
		    }
		  lastIndex++;
		}
	    }
	}
    }
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

void Controller::move(string dir, int amt)
{
  if(dir == "RIGHT")
    locomotor->goRight(amt);
  else if(dir == "LEFT")
    locomotor->goLeft(amt);
  else if(dir == "FORWARD")
    locomotor->goForward(amt);
  else if(dir == "BACKWARD")
    locomotor->goBackward(amt);
  else if(dir == "UTURN")
    locomotor->goUTurn(amt);

  if(amt == AMT_TURN)
    sleep(2);
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
      if(event==EVT_CHECK_TUNNEL)
	return STATE_CHECK_TUNNEL;
      else if(event == EVT_END)
	return END;

    case STATE_CHECK_PASSAGE:
      if(event == EVT_TRUE)
	return STATE_CHECK_TUNNEL;
      if(event == EVT_FALSE)
	return STATE_COMPARE_SYMBOL;
      
    case STATE_TUNNEL_DILEMMA:
      if( == EVT_TRUE)
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
      if(shape == path[lastIndex+1].shape && color == path[lastIndex+1].color)
	return FOUND_SYMBOL;
      ++i;
    }
  return NOT_FOUND_SYMBOL;
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
  std::vector<cv::Mat> frames(MAX_ATTEMPTS);
  VideoCapture cap(0);
  int i = 0;
  while(i<MAX_ATTEMPTS)
    {
      cap >> frames[i++];
      cv::waitKey(33);
    }
  
  dir = CamController::laneFollowDir(frames);
  
  move(dir, AMT_LANE);
 
  return EVT_TUNNEL_DILEMMA;
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
