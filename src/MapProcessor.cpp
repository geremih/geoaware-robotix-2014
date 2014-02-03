#include "MapProcessor.h"

#define DIST_MAX 999999

using namespace std;
using namespace cv;

MapProcessor::MapProcessor()
{
  cout << "Calling empty MapProcessor constructor" << endl;
}

MapProcessor::MapProcessor(Map inp_map)
  :  m(inp_map) 
{  

  landmarks = inp_map.landmarks;
  size = inp_map.landmarks.size();
  img_connections = inp_map.img_src.clone();
  img_route = inp_map.img_src.clone();
  adjMat = initAdjMat();

  assignIndices();
  drawIndices();
  fillAdjMat();
  shortestPath();
}

MapProcessor::~MapProcessor()
{
  for(int i=0;i<size;++i)
    delete adjMat[i];
  delete adjMat;

  for(int i=0;i<route.size();++i)
    {
      if(route[i]!=NULL)
        delete route[i];
    }
}

void MapProcessor::findRoute()
{
  Waypoint *wp = new Landmark(landmarks[0]);
  Landmark *l;
  route.push_back(wp);
  
  int *used = new int[size];
  int i,j,idx;
  int nTJ = 0; //no. of T-Junctions found

  for(i=0;i<size;++i)
    used[i] = 0;
  used[0] = 1;
  
  // guaranteed that "route.back()" will be a landmark
  // since a T-Junction will be followed by a landmark
  while(1)
    {
      wp = route.back();
      l = dynamic_cast<Landmark*>(wp);
      
      idx = l->idx;
      
      if(l->color == "BROWN")
        break;

      for(i=0;i<size;++i)
        {
          //this is the  next landmark in the route
          if(adjMat[idx][i]==1 && used[i]==0)
            {
              wp = new Landmark(landmarks[i]);
              route.push_back(wp);
              used[i] = 1;
              break;
            }
        }
      
      //next in sequence already found above, so go to next iteration
      if(i<size)
        continue;
      
      //next in sequence is a T-Junction
      //check all unused landmarks to see which can form a T-Junction with idx
      for(i=0;i<size;++i)
        {
          if(used[i] == 1)
            continue;
	  
          cv::Point2f p_idx(landmarks[idx].centroid);
          cv::Point2f p_i(landmarks[i].centroid);
	  
          cv::Point2f p1(p_idx.x,p_i.y);
          cv::Point2f p2(p_i.x,p_idx.y);
	  
          if(isConnected(p_idx,p1) && isConnected(p1,p_i))
            {
              wp = new TJunction(p1,size + nTJ);
              ++nTJ;
              route.push_back(wp);
	      
              wp = new Landmark(landmarks[i]);
              used[i] = 1;
              route.push_back(wp);

              break;
            }
          if(isConnected(p_idx,p2) && isConnected(p2,p_i))
            {
              wp = new TJunction(p2,size + nTJ);
              ++nTJ;
              route.push_back(wp);
	      
              wp = new Landmark(landmarks[i]);
              used[i] = 1;
              route.push_back(wp);

              break;
            }
        }
    }
  delete used;
}

void MapProcessor::displayRoute()
{
  findRoute();
  //drawRoute();
  //cv::imshow("route",img_route);
}

void MapProcessor::drawRoute()
{
  int i,j;
  for(i=0;i<route.size()-1;++i)
    cv::line(img_route,route[i]->centroid,route[i+1]->centroid,CV_RGB(150,50,150),2);
}

string MapProcessor::getOrient(Waypoint *w1, Waypoint *w2)
{
  if(!w1 || !w2)
    return "None";
  string orientation;
  cv::Point2f p1(w1->centroid);
  cv::Point2f p2(w2->centroid);
  
  if(p2.y - p1.y > EPSILON)
    orientation = "SOUTH";
  else if(p1.y - p2.y > EPSILON)
    orientation = "NORTH";
  else if(p2.x - p1.x > EPSILON)
    orientation = "EAST";
  else if(p1.x - p2.x > EPSILON)
    orientation = "WEST";
  else
    orientation = "?";
  return orientation;
}


string MapProcessor::getOrient(Landmark l1, Landmark l2)
{
  string orientation;
  cv::Point2f p1(l1.centroid);
  cv::Point2f p2(l2.centroid);
  
  if(p2.y - p1.y > EPSILON)
    orientation = "SOUTH";
  else if(p1.y - p2.y > EPSILON)
    orientation = "NORTH";
  else if(p2.x - p1.x > EPSILON)
    orientation = "EAST";
  else if(p1.x - p2.x > EPSILON)
    orientation = "WEST";
  else
    orientation = "?";
  return orientation;
}

string MapProcessor::getDir(string orientation_curr, string orientation_next)
{
  string direction;
  if(orientation_curr == "NORTH")
    {
      if(orientation_next == "NORTH")
        direction = "STRAIGHT";
      else if(orientation_next == "SOUTH")
        direction = "UTURN";
      else if(orientation_next == "EAST")
        direction = "RIGHT";
      else if(orientation_next == "WEST")
        direction = "LEFT";
      else
        direction =  "?";
    }
  else if(orientation_curr == "SOUTH")
    {
      if(orientation_next == "NORTH")
        direction = "UTURN";
      else if(orientation_next == "SOUTH")
        direction = "STRAIGHT";
      else if(orientation_next == "EAST")
        direction = "LEFT";
      else if(orientation_next == "WEST")
        direction = "RIGHT";
      else
        direction = "?";
    }
  else if(orientation_curr == "EAST")
    {
      if(orientation_next == "NORTH")
        direction = "LEFT";
      else if(orientation_next == "SOUTH")
        direction = "RIGHT";
      else if(orientation_next == "EAST")
        direction = "STRAIGHT";
      else if(orientation_next == "WEST")
        direction = "UTURN";
      else
        direction = "?";
    }
  else if(orientation_curr == "WEST")
    {
      if(orientation_next == "NORTH")
        direction = "RIGHT";
      else if(orientation_next == "SOUTH")
        direction = "LEFT";
      else if(orientation_next == "EAST")
        direction = "UTURN";
      else if(orientation_next == "WEST")
        direction = "STRAIGHT";
      else
        direction = "?";
    }
  else
    direction = "?";
  
  return direction;
}

void MapProcessor::printRoute()
{
  Waypoint *wp,*wp1;
  Landmark *l;
  TJunction *tj;
  int i;
  
  string orientation_curr;
  string orientation_next;
  string direction;
  
  cout << "ROUTE :" << endl << endl;
  cout << "1. STRAIGHT from [START]" << endl << endl;
  for(i=1;i<route.size()-1;++i)
    {
      wp = route[i];
      l = dynamic_cast<Landmark*>(wp);
      if(l!=NULL)
        {
          orientation_curr = getOrient(route[i-1],route[i]);
          orientation_next = getOrient(route[i],route[i+1]);
          direction = getDir(orientation_curr,orientation_next);
	  
          cout << i+1 << ". " << direction << " at Landmark :" << endl;
          cout << *l << endl;
        }
      else
        {
          orientation_curr = getOrient(route[i-1],route[i]);
          orientation_next = getOrient(route[i],route[i+1]);
          direction = getDir(orientation_curr,orientation_next);

          tj = dynamic_cast<TJunction*>(wp);
          cout << i+1 << ". " << direction << " at T-Junction :" << endl;
          cout << *tj << endl;
        }
    }
  cout << i+1 << ". Reached [END]" << endl;
}

void MapProcessor::displayConnections()
{
  drawConnections();
  cv::imshow("connections",img_connections);
}

void MapProcessor::drawConnections()
{
  if(!adjMat)
    return;

  int i,j;
  for(i=0;i<landmarks.size();++i)
    {
      for(j=0;j<i;++j)
        {
          if(adjMat[i][j]==1)
            cv::line(img_connections,landmarks[i].centroid,landmarks[j].centroid,CV_RGB(100,100,100),3);
        }
    }
}

void MapProcessor::assignIndices()
{
  int i;
  Landmark l;
  for(i=0;i<landmarks.size();++i)
    {
      if(landmarks[i].start)
        {
          l = landmarks[0];
          landmarks[0] = landmarks[i];
          landmarks[i] = l;
          break;
        }
    }
  
  for(i=0;i<landmarks.size();++i)
    {
      if(landmarks[i].end)
        {
          l = landmarks[landmarks.size()-1];
          landmarks[landmarks.size()-1] = landmarks[i];
          landmarks[i] = l;
          break;
        }
    }

  for(i=0;i<landmarks.size();++i)
    landmarks[i].idx = i;
}

void MapProcessor::drawIndices()
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;
  
  for(int i=0; i<landmarks.size();++i)
    {
      string label = convertInt(i);
      cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
      cv::Point pt = landmarks[i].centroid;
      cv::rectangle(img_connections, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
      cv::putText(img_connections, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
    }
}

bool MapProcessor::isConnected(Landmark l1, Landmark l2)
{
  // cout << "is connected Landmark: " << l1.centroid << " and " << l2.centroid << endl;
  LineIterator it(m.img_arena, l1.centroid, l2.centroid, 8);
  for(int i = 0; i < it.count; ++i, ++it)
    {
      Vec3b intensity = m.img_tjs.at<Vec3b>(it.pos());
      int blue = (int)intensity.val[0];
      int green = (int)intensity.val[1];
      int red = (int)intensity.val[2];
      if(blue==0 && green==0 && red==0)
	{
	  // cout << "false" << endl;
	  return false;
	}
      // int intensity = static_cast<int>(m.img_arena.at<uchar>(it.pos()));
      // if(intensity<=10)
      // 	return false;
    }
  // cout << "true" << endl;
  return true;
}


bool MapProcessor::isConnectedTJpts(Point2f p1, Point2f p2)
{
  // cout << "is connected TJ pts: " << p1 << " and " << p2 << endl;
  Mat temp = m.img_tjs.clone();
  circle(temp,p1,11, Scalar(255),-1,8,0);
  circle(temp,p2,11, Scalar(255),-1,8,0);
  LineIterator it(temp, p1 , p2, 8);
  for(int i = 0; i < it.count; ++i, ++it)
    {
      Vec3b intensity = temp.at<Vec3b>(it.pos());
      int blue = (int)intensity.val[0];
      int green = (int)intensity.val[1];
      int red = (int)intensity.val[2];
      if(blue==0 && green==0 && red==0)
	{
	  // cout << "false " << endl;
	  return false;
	}
      // int intensity = static_cast<int>(m.img_arena.at<uchar>(it.pos()));
      // if(intensity<=10)
      // 	return false;
    }
  // cout << "true " << endl;
  return true;
}

bool MapProcessor::isConnected(cv::Point2f p1, cv::Point2f p2)
{
  // cout << "is connected pts: " << p1 << " and " << p2 << endl;
  LineIterator it(m.img_arena, p1, p2, 8);
  for(int i = 0; i < it.count; ++i, ++it)
    {
      Vec3b intensity = m.img_arena.at<Vec3b>(it.pos());
      int blue = (int)intensity.val[0];
      int green = (int)intensity.val[1];
      int red = (int)intensity.val[2];
      if(blue==0 && green==0 && red==0)
        {
	  // cout << "false" << endl;
	  return false;
	}
    }
  // cout << "true" << endl;
  return true;
}

int ** MapProcessor::initAdjMat()
{
  int **adjMat = new int*[size];
  for(int i=0;i<size;++i)
    adjMat[i] = new int[size];
  return adjMat;
}

void MapProcessor::fillAdjMat()
{
  if(!adjMat)
    return;
  
  int i,j;
  for(i=0;i<landmarks.size();++i)
    {
      for(j=0;j<i;++j)
        {
          if(isConnectedTJpts(landmarks[i].centroid, landmarks[j].centroid))
            {
              adjMat[i][j] = 1;
              adjMat[j][i] = 1;
            }
          else
            {
              adjMat[i][j] = 0;
              adjMat[j][i] = 0;
            }
        }
    }
  for(i=0;i<landmarks.size();++i)
    adjMat[i][i] = 0;
}

void MapProcessor::printLandmarks()
{
  for(int i=0;i<landmarks.size();++i)
    cout << landmarks[i] << endl;
}

string MapProcessor::convertInt(int number)
{
  stringstream ss;//create a stringstream
  ss << number;//add number to the stream
  return ss.str();//return a string with the contents of the stream
}

void MapProcessor::fillAllAdjMat (){
  Landmark l;
  int total_nodes = landmarks.size() + m.TJs.size();
  cadjMat = std::vector<std::vector<int> >(total_nodes, vector<int>(total_nodes,0));
        
        
  for(int i =0; i< landmarks.size() ; i++){

    //Copy initial adjmatric
    for(int j =0 ; j<landmarks.size() ; j++)
      {
        cadjMat[i][j] = adjMat[i][j];
      }

    //add tjunctions to adjmatrix
    for(int j =0; j< m.TJs.size() ; j++)
      {
        if(isConnectedTJpts(m.TJs[j].centroid , landmarks[i].centroid)) {
          cadjMat[i][landmarks.size() + j] = 1;
          cadjMat[landmarks.size() + j][i] = 1;
        }
      }
    //add landmarks to clandmarks
    clandmarks.push_back( landmarks[i]);
  }


        
  for(int i =0 ; i< m.TJs.size(); i++){
    for(int j =0 ; j< m.TJs.size(); j++){
      if(isConnectedTJpts(m.TJs[j].centroid , m.TJs[i].centroid)){
        cadjMat[landmarks.size() +i][landmarks.size() + j] = 1;
        cadjMat[landmarks.size() + j][landmarks.size() + i] = 1;
      }
    }
    //addJunctions to clandmarks
    l = m.TJs[i];
    l.idx = landmarks.size() + i;
    clandmarks.push_back( l);
  }
        
}
                


bool MapProcessor::landmarkComp( Landmark l1 , Landmark l2){
  return l1.idx < l2.idx;
}



double MapProcessor::getShortestDistanceandPath(  deque<Landmark> hexagon_list ,deque<Landmark>& min_hexagon_list){

  double  curr_distance , min_distance;
  //TODO: nmake this double_max ideally
  min_distance = DIST_MAX;
                        
  do {
    curr_distance = 0;
                                
    for(int j = 0 ; j < hexagon_list.size()-1 ; j++)
      {
        curr_distance += distances[hexagon_list[j].idx][hexagon_list[j+1].idx];
      }
    if( curr_distance < min_distance){
      min_distance = curr_distance;
      min_hexagon_list = hexagon_list;
    }
  } while ( std::next_permutation(hexagon_list.begin() +1 ,hexagon_list.end() -1 , MapProcessor::landmarkComp)); //the first one is start and last is end. so not permuting them

  return min_distance;
}



void MapProcessor::shortestPath(){

  fillAllAdjMat();
  //clandmarks is landmarks with the waypoints included
  populateAllPairDistance( clandmarks );
  
  cout<< "clandmarks is"<< endl;
  for(int i = 0 ; i < clandmarks.size(); i++){
    cout<< clandmarks[i].idx <<" "<< clandmarks[i].color << " "<< clandmarks[i].shape << endl;

  }
  
  //stores all hexagons, start and points
  std::deque<Landmark> hexagon_list;
  std::deque<Landmark> min_distance_hexagon_list;
  //stores all the start landmarks
  std::vector<Landmark> start_list;
  //index of the last landmark
  Landmark end_landmark;



  //populating the hexagon_list vector
  for (int i =0; i < clandmarks.size() ; i++)
    {
      if( clandmarks[i].shape == "HEXAGON" )
        hexagon_list.push_back( clandmarks[i]);
      else if( clandmarks[i].start )
        start_list.push_back(clandmarks[i]);
      else if (clandmarks[i].end){
        end_landmark = clandmarks[i];

      }
    }

  if(start_list.size() == 0){
    cout<<"Shushman, you idiot, where are the start symbols you promised?";
    exit(1);
      
  }
  hexagon_list.push_back(end_landmark);
        
  paths = std::vector< std::vector<Landmark> > (start_list.size());
  //helpers to find out the perfect path

  double min_distance , curr_distance;
  //Find the correct path for each different start_list
  for(int i=0;i < start_list.size() ; i++){
    
    hexagon_list.push_front(start_list[i]);
    
    min_distance = getShortestDistanceandPath( hexagon_list  , min_distance_hexagon_list);
    
    std::vector<Landmark> temp_path;
    
    paths[i].push_back( min_distance_hexagon_list[0]);
    
    for(int j=0; j< min_distance_hexagon_list.size() -1 ; j++)
      {
                                        
        //getPath only returns the intermediate path
        temp_path =  getPath(min_distance_hexagon_list[j].idx , min_distance_hexagon_list[j+1].idx );
        paths[i].insert( paths[i].end() , temp_path.begin() , temp_path.end());
        paths[i].push_back( min_distance_hexagon_list[j+1]);
                                        
      }
    //remove the start element
    hexagon_list.pop_front();
  }

  for(int i =0 ; i< paths.size(); i++)
    {
      drawPath( paths[i]);
    }
    cv::waitKey(0);
}


//void MapProcessor::getPathandDistance( deque<Landmark> hexagon_list , )
void MapProcessor::populateAllPairDistance( std::vector<Landmark> vertices ){

  distances = std::vector<std::vector<int> > (vertices.size(), vector<int>(vertices.size(),DIST_MAX));
  next=  std::vector<std::vector<int> >(vertices.size(), vector<int>(vertices.size(),-1));
  // initialize diagonal
  for(int i=0; i < vertices.size(); i++)
    distances[i][i]=0;

  for( int i =0 ; i < vertices.size() ; i++ ){
    for( int j =0 ; j < vertices.size() ; j++ ){
      if( cadjMat[i][j] != 0)
        distances[j][i] = distances[i][j]  = distance( vertices[i].centroid , vertices[j].centroid);
                                
    }
  }

  // Floyd-Warshall
  // Add nodes between (first 1 then 2, 3 till n) and look if
  // distance is shorter
  for(int k = 0; k < vertices.size(); k++)
    for(int i = 0; i < vertices.size(); i++)
      for(int j = 0; j < vertices.size(); j++)
        if(distances[i][j]>distances[i][k]+distances[k][j]){
          distances[i][j]=distances[i][k]+distances[k][j];
          next[i][j]= k;
        }
                                 
	
  // Print out final distance matrix
  /*  cout<< "The distance matrix is "<<endl;
  for(int i = 0; i < vertices.size(); i++){
    for(int j = 0; j < vertices.size(); j++)
      cout << distances[i][j] << " ";
    cout << endl;
  }

  cout<<"The next matrix is " <<endl;
  for(int i = 0; i < vertices.size(); i++){
    for(int j = 0; j < vertices.size(); j++)
      cout << next[i][j] << " ";
    cout << endl;
  }
  */

}

std::vector<Landmark> MapProcessor::getPath(int i, int  j){

  //the path returned only contains the intermediate points to be visited
  std::vector<Landmark> pathi , pathj , path;

  // TODO: this should be made something that says its not possible to reach
  if (distances[i][j] == DIST_MAX){
    // cout<< i << " and " << j << " are not connected" <<endl;
    return path;

  }
  
  int intermediate = next[i][j];
  if (intermediate ==  -1){
    /*
    cout<< i << " and " << j << " are directly connected" <<endl;
    cout<<"Returning"<<endl;
    */
    return path;
  }
  else {
    pathi = getPath(i, intermediate);
    pathj = getPath(intermediate, j);
    //concatenate the paths
    path.reserve(pathi.size() + pathj.size() + 1);
    path.insert( path.end(), pathi.begin(), pathi.end() );
    path.push_back(clandmarks[intermediate]);
    path.insert( path.end(), pathj.begin(), pathj.end() );
    return path;
  }
        
}

double MapProcessor::distance( cv::Point2f a , cv::Point2f b){
  return cv::norm(cv::Mat(a),cv::Mat(b));
}

void MapProcessor::drawPath( vector<Landmark> path)
{

  int i,j;
  Scalar clr = CV_RGB(rand()%255 , rand()%255 , rand()%255);
  cout << "PATH : " ;
  for(i=0;i<path.size()-1;++i)
    {
      cv::line(img_route,path[i].centroid,path[i+1].centroid,clr,2);
      cout << path[i];
      cout << "---------------" << endl;
    }
  cout << endl;
  cv::imshow("path",img_route);
}


bool MapProcessor::tunnelDilemma(  string direction , vector<Landmark>& path_taken , int last_landmark ,  vector<Landmark>& new_path , float approx = .5){

  /* orienation should be of the form LEFT or RIGHT*/
  Landmark previous = path_taken[last_landmark];
  Landmark next = path_taken[last_landmark +1];
  string curr_orientation = getOrient(previous , next);
  Point start = Point( (1- approx)* previous.centroid.x + approx*next.centroid.x ,  (1- approx)* previous.centroid.y + approx*next.centroid.y);
  Point end;

  /*
    WHAT IS THE DIFFENCE BETWEEEN m.img_src , m.img_arena ?
  */
  if(direction == "LEFT"){

    if(curr_orientation == "NORTH"){
      end = Point(0 , start.y );
    }
    else if(curr_orientation == "SOUTH"){
      end = Point(m.img_src.cols ,start.y);
    }
    else     if(curr_orientation == "EAST"){
      end = Point( start.x , 0);
    }
    else     if(curr_orientation == "WEST"){
      end = Point(start.x , m.img_src.rows);
    }
    
  }
  else if(direction == "RIGHT")
    {
      if(curr_orientation == "NORTH"){
        end = Point(m.img_src.cols , start.y);
      }
    else if(curr_orientation == "SOUTH"){

      end = Point(0 , start.y );
    }
    else     if(curr_orientation == "EAST"){
      end = Point(start.x , m.img_src.rows);
    }
    else     if(curr_orientation == "WEST"){
      end = Point( start.x , 0);
    }
  }
  else
    {
      cout<<"What the fuck are you passing as arguments?"<<endl;
    }


  LineIterator it(m.img_src, start, end, 8);
  bool inWall = false;
  Point exit;
  for(int i = 0; i < it.count; ++i, ++it)
    {
      Vec3b intensity = m.img_src.at<Vec3b>(it.pos());
      int blue = (int)intensity.val[0];
      int green = (int)intensity.val[1];
      int red = (int)intensity.val[2];
      if((blue>250 && green>250 && red> 250)){
        if(inWall){
          exit = it.pos();
          break;
        }
      }
        
      if(blue<=BLACK_THRESHOLD && green<=BLACK_THRESHOLD && red<=BLACK_THRESHOLD) //white or black
        {
          inWall = true;
        }
    }


  double distance_left = 0;

  for(int i =last_landmark+1 ; i<path_taken.size()-1;i++){
    distance_left += distance(path_taken[i].centroid , path_taken[i+1].centroid);
  }

  cout<<"The distance to be traversed in the current path is " << distance_left <<endl;


  deque<Landmark> hexagon_list;
  for(int i = 0; i < clandmarks.size() ;i++)
    {
      if( clandmarks[i].shape == "HEXAGON" && clandmarks[i].visited == false)
        hexagon_list.push_back( clandmarks[i]);
    }

  double curr_distance;
  deque< Landmark> min_hexagon_list;
  bool takeTunnel = false;
  vector<Landmark>  temp_path;
  hexagon_list.push_back(path_taken[path_taken.size()-1]);
  for(int i = 0; i < clandmarks.size() ;i++)
    {
      if(isConnected( exit, clandmarks[i].centroid ) ){
        hexagon_list.push_front( clandmarks[i]);
        curr_distance = getShortestDistanceandPath( hexagon_list , min_hexagon_list);
        hexagon_list.pop_front();
        if(curr_distance < distance_left){
          takeTunnel = true;
          new_path.clear();
          new_path.push_back( min_hexagon_list[0]);
          for(int j=0; j< min_hexagon_list.size() -1 ; j++)
            {
              //getPath only returns the intermediate path
              temp_path =  getPath(min_hexagon_list[j].idx , min_hexagon_list[j+1].idx );
              new_path.insert( new_path.end() , temp_path.begin() , temp_path.end());
              new_path.push_back( min_hexagon_list[j+1]);
            }
        }
      }
    }

  

}
  
      
  

  


