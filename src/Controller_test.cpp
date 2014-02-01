#include "Controller.h"

bool Controller::instanceFlag = false;

Controller* Controller::single = NULL;

VideoCapture  Controller::cap(0);

Controller* Controller::getInstance(string path , string ACM , string USB)
{
  if(!instanceFlag)
    {
      single = new Controller(path, ACM , USB);
      cout<<"Created new controller instance"<<endl;
      instanceFlag = true;
    }
  return single;
}

Controller::Controller(string mappath, string ACM , string USB):
  lastIndex(0), tunnelMode(false), orientation("?"), pathFound(false), m(mappath), mp(m), symbolDetector()
{
  path = mp.paths[0];
  pathFound = true;
  orientation = MapProcessor::getOrient(path[0],path[1]);
  locomotor = Locomotor::getInstance(ACM , USB);
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
  int flag = -1;
  string orientation_next;
  std::vector<Landmark> newpath;
  
  flag = NOT_FOUND_TUNNEL;
  
  if(flag == FOUND_TUNNEL_AND_TAKE && pathFound)
    {
      // traverse tunnel
      tunnelMode = true;
      // facePassage will align and turn to face the passage (and enter slightly so that lane detection can easily pick up ?)
      facePassage(passageDir);
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
	  facePassage(passageDir);
	  turned = true;
	}
      ++lastIndex;
    }
  return turned;
}


#define LANE_WIDTH 40

void Controller::turnCorner(string passageDir){

  int distance , curr_distance;
  int vote = 0;
  if (passageDir == "RIGHT"){

    while(vote < 3){
      curr_distance = locomotor->getDistanceFront();
      if(curr_distance<  50)
        vote++;
      else if(vote>0)
        vote--;
      usleep(50000);
      followLane(1);
      moveBot("FORWARD" ,1 );
    }
    moveBot("BACKWARD" ,15 );
    locomotor->goRight(25);
  }
  else  if (passageDir == "LEFT"){

    while(vote < 3){
      curr_distance = locomotor->getDistanceFront();
      if(curr_distance < 50)
        vote++;
      else if(vote>0)
        vote--;
      usleep(50000);
      followLane(1);
      moveBot("FORWARD" ,1 );
    }
    moveBot("BACKWARD" ,15 );
    locomotor->goLeft(25);
  }



}
void Controller::facePassage(string passageDir){

  int distance , curr_distance;
  int vote = 0;
  cout << "facing passage on the " << passageDir << endl;
  if (passageDir == "RIGHT"){

    while(vote < 3){
      curr_distance = locomotor->getDistanceRight();
      if(curr_distance> LANE_WIDTH)
        vote++;
      else if(vote>0)
        vote--;
      usleep(50000);
      followLane(1);
      moveBot("FORWARD" ,1 );
    }
    moveBot("BACKWARD" ,15 );
    locomotor->gradualRight(300);
  }
  else  if (passageDir == "LEFT"){

    while(vote < 3){
      curr_distance = locomotor->getDistanceLeft();
      if(curr_distance> LANE_WIDTH)
        vote++;
      else if(vote>0)
        vote--;
      usleep(50000);
      followLane(1);
      moveBot("FORWARD" ,1 );
    }
    moveBot("BACKWARD" ,15 );
    locomotor->gradualLeft(300);
  }
  cout << "face passage done" << endl;
}


void Controller::mainLoop()
{
  int flag = -1;
  bool turned;
  string passageDir;
  string orientation_next, direction;
  string shape, color;
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
      cout << "current orientation : " << orientation << endl;

      Point centroid;
      flag = detectSymbolController(shape,color ,centroid);
      if(distance_front < LANE_FOLLOW_MIN && i++ > 2)
	{
	  i=0;
	  // reached a corner
	  cout << "\t\treached end, turning now" << endl;
	  if(lastIndex == path.size() -2)
	    {
	      cout << "DONE" << endl;
	      exit(1);
	    }
	  if(flag == FOUND_SYMBOL)
	    {
	      if(shape == path[lastIndex+1].shape && color == path[lastIndex+1].color)
		{
		  // turn in reqd dir and upd orientation
		  orientation_next = MapProcessor::getOrient(path[lastIndex+1],path[lastIndex+2]);
		  direction = MapProcessor::getDir(orientation,orientation_next);
		  cout << "found " << shape << ", " << color << " on the " << direction << endl;
		  facePassage(direction);
		  orientation = orientation_next;
		  lastIndex++;
		}
	      else
		{
		  cout << "Unexpected symbol : found " << shape << ", " << color << " |  expected " << path[lastIndex+1].shape << ", " << path[lastIndex+1].color << endl;
		  orientation_next = MapProcessor::getOrient(path[lastIndex+1],path[lastIndex+2]);
		  direction = MapProcessor::getDir(orientation,orientation_next);
		  cout << "facing passage on the " << direction << endl;
		  facePassage(direction);
		  cout << "face passage done" <<endl;
		  orientation = orientation_next;
		  lastIndex++;
		}
	    }
	  else
	    {
	      cout << "didnt detect any symbol" << endl;
	      orientation_next = MapProcessor::getOrient(path[lastIndex+1],path[lastIndex+2]);
	      direction = MapProcessor::getDir(orientation,orientation_next);
	      cout << "facing passage on the " << direction << endl;
	      facePassage(direction);
	      cout << "face passage done" <<endl;
	      orientation = orientation_next;
	      lastIndex++;
	    }
	  continue;
	}

      
      cout << "lastIndex = " << lastIndex << "path.size : " << path.size() << endl;
      if(lastIndex + 2 < path.size() && path[lastIndex+1].shape == "TJ")
	{
	  orientation_next = MapProcessor::getOrient(path[lastIndex+1],path[lastIndex+2]);
	  direction = MapProcessor::getDir(orientation,orientation_next);
	  cout << "orientation = " << orientation << " o_next = " << orientation_next << endl;
	  cout << "anticipating a TJ on the " << direction << endl;
	  facePassage(direction);
	  orientation = orientation_next;
	  ++lastIndex;
	}

      if(distance_front > LANE_FOLLOW_MIN)
	{
	  cout << "\t\tSTILL ROOM, FOLLOWING LANE" << endl;
	  followLane(3);
	  
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

int Controller::detectSymbolController(string& shape, string& color , Point & centroid)
{
  cv::Mat frame;
  int i = 0;
  string prevShape = "",prevColor = "";
  string currShape,currColor;

  // while(i<MAX_ATTEMPTS - 1)
  //   {
  // if 2 successive frames detect same symbol, return it
  for(int j=0;j<5;++j)
    cap >> frame;
  Rect roi(0,frame.rows/8,frame.cols-1,frame.rows - frame.rows/8);// set the ROI for the image
  frame = frame(roi); 
  centroid = symbolDetector.getSymbol(frame,currShape,currColor);
  cout << "centroid in controller, detect symbol : " << centroid << endl;

  if((currShape == "CIRCLE" || currShape == "TRIANGLE" || currShape == "SQUARE") && (currColor == "BLUE" || currColor == "RED" || currColor == "GREEN"))
    {
      shape = currShape;
      color = currColor;
      return FOUND_SYMBOL;
    }
  // prevShape = currShape;
  // prevColor = currColor;
  //cv::waitKey(33);
  // }

  return NOT_FOUND_SYMBOL;
}


#define CENTROID_OFFSET  120

void Controller::followLane(int amount)
{

  cv::Mat frame;
  bool left ,right;
  cap >> frame;
  
  string dir;
  int i = 0;
  int nL = 0, nR = 0, nS = 0;

  string shape , color;
  Point centroid;
  
  int flag = detectSymbolController(shape,color ,centroid);
  
  if( flag == FOUND_SYMBOL){
    cout << "found symbol to follow : " << centroid << endl;
    if( centroid.x - frame.cols/2 > CENTROID_OFFSET )
      {
	right = true;
	left = false;
      }
    else if (  frame.cols/2 - centroid.x > CENTROID_OFFSET)
      {
	left = true;
	right = false;	
      }
    else
      {
	right = false;
	left = false;
      }

    cv::circle(frame,centroid,3,CV_RGB(255,0,0) , 2, 8, 0 );
    cout << "aiming at : " << centroid << ", frame.cols/2 = " << frame.cols/2 << endl;
    imshow("frame",frame);
  }
  else {
    
    for(int j = 0; j<5; j++)
      cap >> frame;
    cout << "NO symbol to follow, using hough lines" << endl;
    CamController::processVideo(frame , "LANE" , left , right);

  }
  
  if( left == true )
    {
      dir = "LEFT";

    }
    
  else if (right == true )
    {
      dir = "RIGHT";
    }
  else
    {
      dir = "STRAIGHT";
    }
  cv::waitKey(20);
  cout << "\tFOLLOW LANE : moving " << dir << endl;
  moveBot(dir, AMT_LANE);
}



int main(int argc, char *argv[])
{
  if(argc< 2)
    {
      cout << "Usage : ./GeoAware <path/to/map>" << endl;
      exit(1);
    }

  Controller * controller;
  if(argc ==4)
    controller = Controller::getInstance(argv[1] , argv[2], argv[3]);
  else
    controller = Controller::getInstance(argv[1]);
  cout << "created controller, starting loop now " << endl;
  controller->start();
  return 0;
}
