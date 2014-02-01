#include "Controller.h"

bool Controller::instanceFlag = false;

Controller* Controller::single = NULL;

VideoCapture  Controller::cap(0);

Controller* Controller::getInstance(string path)
{
  if(!instanceFlag)
    {
      single = new Controller(path);
      cout<<"Created new controller instance"<<endl;
      instanceFlag = true;
    }
  return single;
}

Controller::Controller(string path)
{
  locomotor = Locomotor::getInstance();
  //cam = CamController::getInstance();
}

void Controller::start()
{
  mainLoop();
}

// returns whether turned or not
bool Controller::processPassage(string passageDir)
{
  bool turned = false;
  int flag;
  string orientation_next;
  std::vector<Landmark> newpath;
  
  // tunnel dilemma sends newpath consistent with lastIndex
  // check params (pass by ref tunnelExitDir)
  //flag = mp.tunnelDilemma(passageDir,path,lastIndex,newpath, 0.5);
  flag = NOT_FOUND_TUNNEL;
  if(flag == FOUND_TUNNEL_AND_TAKE && pathFound)
    {
      // traverse tunnel
      tunnelMode = true;
      // facePassage will align and turn to face the passage (and enter slightly so that lane detection can easily pick up ?)
      locomotor->facePassage(passageDir);
      path = newpath;
      turned = true;
    }
  else if(flag == FOUND_TUNNEL_DONT_TAKE && pathFound)
    {
      moveBot("STRAIGHT",AMT_TURN);
    }
  else if(flag == NOT_FOUND_TUNNEL)
    {
      // found a T-Junction
      if(pathFound == false)
	{
	  // try to select the path we are on and update orientation
	  for(int i=0;i<mp.paths.size();++i)
	    {
	      if(mp.paths[i].size() > 1 && mp.paths[i][1].shape == "TJ") // also check for the reqd direction here, update that
		{
		  path = mp.paths[i];
		  orientation = MapProcessor::getOrient(path[lastIndex],path[lastIndex+1]);
		  break;
		}
	    }
	  pathFound = true;
	}
	      
      orientation_next = MapProcessor::getOrient(path[lastIndex+1],path[lastIndex+2]);
      string turn_dir = MapProcessor::getDir(orientation,orientation_next);
      // if orientations match, take left at T-Junction
      if(turn_dir == passageDir)
	{
	  locomotor->facePassage(passageDir);
	  turned = true;
	}
      ++lastIndex;
    }
  return turned;
}
  
void Controller::mainLoop()
{
  int flag;
  bool turned;
  string passageDir;
  string orientation_next, direction;
  //string shape, color;
  bool pLeft = false;
  bool pRight = false;
  int distance_front; //cm
  cv::Mat frame;

  cout << "in main loop" << endl;
  int i = 0;
  while(true)
    {
      distance_front = locomotor->getDistanceFront();
      cout << "DISTANCE_FRONT : " << distance_front << endl;
      if(distance_front > LANE_FOLLOW_MIN)
	followLane();
      else
	moveBot("STRAIGHT",AMT_LANE);
      
      if(distance_front < LANE_FOLLOW_MIN && i++ > 2)
	{
	  // reached end of corridor
	  // Cases :
	  // 1. end of tunnel
	  // 2. corner
	  // in case of a T-Junction we would have already turned to face it above
	  

	  // reached a corner
	  cout << "reached end, turning now" << endl;
	  locomotor->facePassage("LEFT");

	}

      
    }
}

void Controller::moveBot(string dir, int amt)
{
  cout << "moveBot : " << dir << ", " << amt << endl;
  if(dir == "RIGHT")
    locomotor->goRight(amt);
  else if(dir == "LEFT")
    locomotor->goLeft(amt);
  else if(dir == "FORWARD" || dir == "STRAIGHT")
    locomotor->goForward(amt);
  else if(dir == "BACKWARD")
    locomotor->goBackward(amt);
  else if(dir == "UTURN")
    locomotor->goUTurn(amt);
  
}

int Controller::detectSymbol(string& shape, string& color)
{
  cv::Mat frame;

  int i = 0;
  string prevShape,prevColor;
  string currShape,currColor;

  // if 2 successive frames detect same symbol, return it
  cap >> frame;
  symbolDetector.getSymbol(frame,prevShape,prevColor);
  while(i<MAX_ATTEMPTS - 1)
    {
      cap >> frame;
      symbolDetector.getSymbol(frame,currShape,currColor);
      if(currShape == prevShape && currColor == prevColor)
	{
	  shape = currShape;
	  color = currColor;
	  return FOUND_SYMBOL;
	}
      prevShape = currShape;
      prevColor = currColor;
      ++i;
      cv::waitKey(33);
    }

  return NOT_FOUND_SYMBOL;
}


void Controller::followLane()
{ 
  //std::vector<cv::Mat> frames(MAX_ATTEMPTS);
  cv::Mat frame;
  string dir;
  int i = 0;
  int nL = 0, nR = 0, nS = 0;
  for(int j = 0; j<5; j++)
    cap >> frame;

  while(i<3)
    {
      cap>>frame;
      bool left ,right;

    CamController::processVideo(frame , "LANE" , left , right);

    if( left == true )
      {
	nL++;

      }
    
    else if (right == true )
      {
	nR++;
      }
    else
      {
	nS++;
      }
    

    // if(CamController::laneFollowDir(frame) == "LEFT")
    //    ++nL;
    //  else if(CamController::laneFollowDir(frame) == "RIGHT")
    //    ++nR;
    //  else if(CamController::laneFollowDir(frame) == "STRAIGHT")
    //    ++nS;

    i++;
    }
  cv::waitKey(500);
  if(nL > nR && nL > nS)
    dir = "LEFT";
  else if(nR > nL && nR > nS)
    dir = "RIGHT";
  else
    dir = "STRAIGHT";
  cout << "\tFOLLOW LANE : moving " << dir << endl;
  moveBot(dir, AMT_LANE);
  moveBot("FORWARD" ,AMT_LANE);
}



int main(int argc, char *argv[])
{
  if(argc!=2)
    {
      cout << "Usage : ./GeoAware <path/to/map>" << endl;
      exit(1);
    }
  
  Controller *controller = Controller::getInstance(argv[1]);
  cout << "created controller, starting loop now " << endl;
  controller->start();
  return 0;
}
