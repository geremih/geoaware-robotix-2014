#include "Map.h"

using namespace std;
using namespace cv;

Map::Map()
{
  cout << "Calling empty Map constructor" << endl;
}

Map::Map(const string path)
{
  img_src = cv::imread(path);
  if(img_src.empty())
    {
      cout << "Invalid input map! Exiting. " << endl;
      exit(EXIT_FAILURE);
    }

  // cv::Mat kernel = Mat::ones(Size(3, 3), CV_8U);
  // cv::Mat eroded = img_src.clone();
  // cv::erode(img_src,eroded,kernel);
  // cv::Mat dilated = eroded.clone();
  // cv::dilate(eroded,dilated,kernel);
  // cv::imshow("src", img_src);
  // cv::imshow("eroded", eroded);
  // cv::imshow("dilated",dilated);
  // cv::waitKey(0);
  // img_src = dilated;
  addBorders(0,0,0);
  img_waypoints = img_src.clone();
  img_landmarks = obtainLandmarksImg();
  img_arena = obtainArenaImg();
  corners = getCorners(img_arena);
  landmarks = getLandmarks();
  img_tjs = img_arena.clone();
  TJs = getTJs();
  displayMap();
}

Map::Map(const Map& M)
{
  img_src = M.img_src.clone();
  img_landmarks = M.img_landmarks.clone();
  img_arena = M.img_arena.clone();
  landmarks = M.landmarks;
  img_tjs = M.img_tjs;
  TJs = M.TJs;
}

//adds 2 px wide border around the image to take care of boundary contours
void Map::addBorders(int blue, int green, int red)
{
  int i,j;
  i = img_src.rows - 1;
  for(j=0;j<img_src.cols;++j)
    {
      img_src.at<Vec3b>(0,j)[0] = blue;
      img_src.at<Vec3b>(0,j)[1] = green;
      img_src.at<Vec3b>(0,j)[2] = red;

      img_src.at<Vec3b>(1,j)[0] = blue;
      img_src.at<Vec3b>(1,j)[1] = green;
      img_src.at<Vec3b>(1,j)[2] = red;

      img_src.at<Vec3b>(i-1,j)[0] = blue;
      img_src.at<Vec3b>(i-1,j)[1] = green;
      img_src.at<Vec3b>(i-1,j)[2] = red;
      
      img_src.at<Vec3b>(i,j)[0] = blue;
      img_src.at<Vec3b>(i,j)[1] = green;
      img_src.at<Vec3b>(i,j)[2] = red;
    }
  
  j = img_src.cols - 1;
  for(i=0;i<img_src.rows;++i)
    {
      img_src.at<Vec3b>(i,0)[0] = blue;
      img_src.at<Vec3b>(i,0)[1] = green;
      img_src.at<Vec3b>(i,0)[2] = red;

      img_src.at<Vec3b>(i,1)[0] = blue;
      img_src.at<Vec3b>(i,1)[1] = green;
      img_src.at<Vec3b>(i,1)[2] = red;

      img_src.at<Vec3b>(i,j-1)[0] = blue;
      img_src.at<Vec3b>(i,j-1)[1] = green;
      img_src.at<Vec3b>(i,j-1)[2] = red;
      
      img_src.at<Vec3b>(i,j)[0] = blue;
      img_src.at<Vec3b>(i,j)[1] = green;
      img_src.at<Vec3b>(i,j)[2] = red;
    }
}

cv::Mat Map::obtainLandmarksImg()
{
  cv::Mat landmarks( img_src.rows, img_src.cols, CV_8UC3, CV_RGB(0,0,0));
  for(int i=0; i<img_src.rows; i++)
    {
      for(int j=0; j<img_src.cols; j++)
	{
	  //BGR
	  Vec3b intensity = img_src.at<Vec3b>(i,j);
	  int blue = (int)intensity.val[0];
	  int green = (int)intensity.val[1];
	  int red = (int)intensity.val[2];
	  
	  if((blue==255 && green==255 && red==255) || (blue<=BLACK_THRESHOLD && green<=BLACK_THRESHOLD && red<=BLACK_THRESHOLD)) //white or black
	    {
	      landmarks.at<Vec3b>(i,j)[0] = 0;
	      landmarks.at<Vec3b>(i, j)[1] = 0;
	      landmarks.at<Vec3b>(i, j)[2] = 0;
	    }
	  else //landmark or start/end
	    {
	      landmarks.at<Vec3b>(i, j)[0] = blue;
	      landmarks.at<Vec3b>(i, j)[1] = green;
	      landmarks.at<Vec3b>(i, j)[2] = red;
	    }
	}
    }
  return landmarks; 
}

cv::Mat Map::obtainArenaImg()
{
  cv::Mat arena(img_src.rows,img_src.cols, CV_8UC3, CV_RGB(0,0,0));
  for(int i=0;i<img_src.rows;++i)
    {
      for(int j=0;j<img_src.cols;++j)
	{
	  Vec3b intensity = img_src.at<Vec3b>(i,j);
	  int blue = (int)intensity.val[0];
	  int green = (int)intensity.val[1];
	  int red = (int)intensity.val[2];
	  
	  if(!(blue==0 && green==0 && red==0))
	    {
	      arena.at<Vec3b>(i, j)[0] = 255;
	      arena.at<Vec3b>(i, j)[1] = 255;
	      arena.at<Vec3b>(i, j)[2] = 255;
	    }
	}      
    }
  return arena;
}

void Map::removeDuplicateContours(std::vector<std::vector<cv::Point> >& contours_dupl, std::vector<std::vector<cv::Point> >& contours)
{
  int i,j;
  int X,Y,x,y;
  cv::Rect R,r;
  for(i=0;i<contours_dupl.size();++i)
    {
      R = cv::boundingRect(contours_dupl[i]);
      X = R.x;
      Y = R.y;
      for(j=0;j<contours.size();++j)
	{
	  r = cv::boundingRect(contours[j]);
	  x = r.x;
	  y = r.y;
	  if(abs(X-x)<EPSILON && abs(Y-y)<EPSILON)
	    break;
	}
      if(j==contours.size())
	  contours.push_back(contours_dupl[i]);
    }
}

// remove small trivial edges from the shape
void Map::cleanEdges(std::vector<cv::Point>& approx, std::vector<cv::Point>& actual)
{
  // Number of vertices of polygonal curve (small edges yet to be removed)
  int vtc = approx.size();
  
  if(vtc>=3 && vtc<=6)
    {
      int j = 0;
      int s = approx.size();
      while(j<s)
	{
	  Point2f a(approx[j].x,approx[j].y);
	  Point2f b(approx[(j+1) % s].x,approx[(j+1) % s].y);
	  double res = cv::norm(a-b);
	  if(res>=EPSILON) //keep only large edges
	    actual.push_back(approx[j]);
	  ++j;
	}
    }
  else
    actual = approx;
}

void Map::printLandmarks()
{
  cout << "\tLandmarks : " << endl << endl;
  for(int i=0;i<landmarks.size();++i)
    cout << landmarks[i] << endl << endl;
  cout << endl << "\tT Junctions : " << endl << endl;
  for(int i=0;i<TJs.size();++i)
    cout << TJs[i] << endl;
}

void Map::addLandmark(std::vector<Landmark>& landmarks, std::vector<cv::Point>& symbol)
{
  cv::Point2f center(0.f,0.f);
  float radius = 0.f;
  cout << "adding symbol " << symbol << endl;
  cv::minEnclosingCircle(symbol,center,radius);
  //cv::circle(img_landmarks, center, radius, CV_RGB(100,200,255), 2);
  
  int vtc = symbol.size();
  vtc = (vtc > HEXAGON) ? CIRCLE : vtc;
  string shape = vtxToShape(vtc);

  Vec3b intensity = img_landmarks.at<Vec3b>((int)center.y,(int)center.x);
  int blue = (int)intensity.val[0];
  int green = (int)intensity.val[1];
  int red = (int)intensity.val[2];
  
  string color = getColor(blue,green,red);
  int start = 0, end = 0;
  if(color=="YELLOW")
    start = 1;
  if(color=="BROWN")
    end = 1;
  Landmark l(center,shape,color,start,end);
  landmarks.push_back(l); 
}

std::vector<Landmark> Map::getLandmarks()
{
  // Convert to grayscale
  cv::Mat img_gray;
  cv::cvtColor(img_landmarks, img_gray, CV_BGR2GRAY);
  
  // Use Canny instead of threshold to catch squares with gradient shading
  cv::Mat img_bw;
  cv::Canny(img_gray, img_bw, 0, 50, 5);
  cv::imshow("edges",img_bw);
  cv::Mat img_dst = img_landmarks.clone();
  
  // Find contours
  std::vector<std::vector<cv::Point> > contours_dupl;
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(img_bw.clone(), contours_dupl, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  
  removeDuplicateContours(contours_dupl,contours);
  
  // approx holds an individual contour found by approxPolyDP
  std::vector<cv::Point> approx;
  
  // actual removes the small edges from approx so that the intended shape is obtained
  std::vector<cv::Point> actual;

  // Vector of all the landmarks
  std::vector<Landmark> landmarks;

  //cout << "Number of contours found : " << contours.size() << endl;

  //Get all the landmarks from the contours
  for (int i = 0; i < contours.size(); i++)
    {      
      cv::drawContours(img_landmarks,contours,i,CV_RGB(255,255,255),2);
      
      // Approximate contour with accuracy proportional to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

      // Skip small or non-convex objects
      /*if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
	{
	  cout << "Skipping contour " << i << endl;
	  continue;
	  }*/
      
      cleanEdges(approx,actual);
      if(actual.size() < 3 || cv::contourArea(approx) < MIN_AREA)
	{
	  actual.clear();
	  continue;
	}	  
      //cout << "approx.size = " << approx.size() << ", actual.size = " << actual.size() << endl;
      int vtc = actual.size();
      if(vtc==3)
	setLabel(img_dst, string("TRI").append(convertInt(i)), contours[i]);
      else if (vtc == 4)
	setLabel(img_dst, string("RECT").append(convertInt(i)), contours[i]);
      else if (vtc == 5)
	setLabel(img_dst, string("PENTA").append(convertInt(i)), contours[i]);
      else if (vtc == 6)
	setLabel(img_dst, string("HEXA").append(convertInt(i)), contours[i]);	  
      else
	{
	  // Detect and label circles
	  double area = cv::contourArea(contours[i]);
	  cv::Rect r = cv::boundingRect(contours[i]);
	  float radius = r.width / 2;
	  
	  if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
	    setLabel(img_dst, string("CIR").append(convertInt(i)), contours[i]);
	}
      
      addLandmark(landmarks,actual);
      actual.clear();
    
    }
  return landmarks;
}

void Map::displayMap()
{
  drawTJs();
  cv::imshow("src",img_src);
  cv::imshow("arena",img_arena);
  cv::imshow("landmarks",img_landmarks);
  cv::imshow("waypoints",img_waypoints);
  cv::imshow("img tjs",img_tjs);
}

const Map& Map::operator = (const Map& m)
{
  img_src = m.img_src;
  img_landmarks = m.img_landmarks;
  img_arena = m.img_arena;
  landmarks = m.landmarks;
  
  return *this;
}
// STATIC FUNCTIONS :

string Map::vtxToShape(int vtc)
{
  if (vtc == CIRCLE)
    return "CIRCLE";
  if(vtc == TRIANGLE)
    return "TRIANGLE";
  if(vtc == SQUARE)
    return "SQUARE";
  if(vtc == PENTAGON)
    return "PENTAGON";
  if(vtc == HEXAGON)
    return "HEXAGON";
  return "None";
}

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
double Map::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void Map::setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;
  
  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);
  
  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

bool Map::isPrimaryColor(int blue, int green, int red)
{
  return ((blue-green>PRIMARY_THRESHOLD && blue-red>PRIMARY_THRESHOLD) || (green-blue>PRIMARY_THRESHOLD && green-red>PRIMARY_THRESHOLD) || (red-blue>PRIMARY_THRESHOLD && red-green>PRIMARY_THRESHOLD));
}

string Map::getColor(int blue, int green, int red)
{
  string color;
  if(isPrimaryColor(blue,green,red))
    {
      if(blue>green && blue>red)
	color = "BLUE";
      else if(green>blue && green>red)
	color = "GREEN";
      else
	color = "RED";
    }
  else
    {
      if(red>200 && green>200)
	color = "YELLOW";
      else
	color = "BROWN";
    }
  return color;
}

string Map::convertInt(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}


void Map::drawTJs()
{
  for(int i=0;i<TJs.size();++i)
    {
      circle(img_waypoints,TJs[i].centroid,7,Scalar(0),-1,8,0);
      circle(img_tjs,TJs[i].centroid,10,Scalar(0),-1,8,0);
    }
}

void Map::removeDuplicateTJpts(std::vector<cv::Point>& TJs_dupl, std::vector<cv::Point>& TJs_unique)
{
  int i,j;
  int X,Y,x,y;
  for(i=0;i<TJs_dupl.size();++i)
    {
      X = TJs_dupl[i].x;
      Y = TJs_dupl[i].y;
      for(j=0;j<TJs_unique.size();++j)
	{
	  x = TJs_unique[j].x;
	  y = TJs_unique[j].y;
	  if(abs(X-x)<EPSILON_TJ && abs(Y-y)<EPSILON_TJ)
	    break;
	}
      if(j==TJs_unique.size())
	  TJs_unique.push_back(TJs_dupl[i]);
    }
}


int Map::getIndex(std::vector<int> vec, int val)
{
  int i;
  for(i=0;i<vec.size();++i)
    {
      if(vec[i] == val)
	return i;
    }
  return -1;
}

// color of line p1-p2
// 1 if all white
// 0 if all black
// -1 otherwise
int Map::lineColor(cv::Mat img, cv::Point2f p1, cv::Point2f p2)
{
  LineIterator it(img, p1, p2, 8);
  int nB = 0;
  int nW = 0;
  int flag = -1;
  for(int i = 0; i < it.count; ++i, ++it)
    {
      Vec3b intensity = img.at<Vec3b>(it.pos());
      int blue = (int)intensity.val[0];
      int green = (int)intensity.val[1];
      int red = (int)intensity.val[2];
      if(blue==0 && green==0 && red==0)
	++nB;
      else
	++nW;
    }
  int nT = nW + nB;
  if((double)nB/nT < 0.01) // all white
    flag = 1;
  if((double)nW/nT < 0.01) // all black
    flag = 0;
  return flag;
}

// adds corner to 'corners' vector if not already present
bool Map::addCorner(std::vector<cv::Point>& corners, cv::Point pt)
{
  for(int i=0;i<corners.size();++i)
    {
      if(abs(corners[i].x - pt.x) < EPSILON && abs(corners[i].y - pt.y) < EPSILON)
	return false;
    }
  corners.push_back(pt);
  return true;
}

// stores retreived corners in the member 'corners' vector
std::vector<cv::Point> Map::getCorners(cv::Mat img_arena)
{
  std::vector<cv::Point> corners;
  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( img_arena.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  int thresh = 85;
  int max_thresh = 255;

  cv::Mat img_arena_gray;
  cv::cvtColor( img_arena, img_arena_gray, CV_BGR2GRAY );
 
  /// Detecting corners
  cv::cornerHarris( img_arena_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  cv::normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  cv::convertScaleAbs( dst_norm, dst_norm_scaled );
  
  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
    { for( int i = 0; i < dst_norm.cols; i++ )
	{
	  if( (int) dst_norm.at<float>(j,i) > thresh )
	    {
	      if(addCorner(corners,cv::Point(i,j)))
		cv::circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
	    }
	}
    }
  
  /// Showing the result
  cv::imshow( "corners", dst_norm_scaled );
  return corners;
}


std::vector<Landmark> Map::getTJs()
{
  int i,j,k,n;
  int width;
  cv::Point p,p1,p2,p3,p4,p5,p6;
  
  // TMat[i][j] = 1 if corners i and j give rise to TJ
  std::vector<std::vector<int> > TMat(corners.size(), std::vector<int>(corners.size(),0));
  // adjMat[i][j] = 1 if corner i is connected to corner j
  std::vector<std::vector<int> > adjMat(corners.size(), std::vector<int>(corners.size(),0));
 
  // sequence of linked corner indices
  std::vector<int> seq;
  std::vector<int> used(corners.size(),0);
  int curr = 0;
  int next = 0;

  int idx_i, idx_j;
  char dir_i1,dir_i2;
  char dir_j1,dir_j2;
  char dirT;
  
  std::vector<cv::Point> TJpts_dupl;
  std::vector<cv::Point> TJpts_unique;
  
  std::vector<Landmark> TJunctions;
  
  n = corners.size();
  // create adjMat and TMat
  for(i=0;i<n;++i)
    {
      for(j=0;j<i;++j)
	{
	  // vertical line
	  if(abs(corners[i].x - corners[j].x)<EPSILON)
	    {
	      p1 = (corners[i].y < corners[j].y) ? corners[i] : corners[j];
	      p2 = (p1==corners[i]) ? corners[j] : corners[i];
	      
	      p3 = cv::Point(p1.x-2,p1.y+2);
	      p4 = cv::Point(p1.x+2,p1.y+2);
	      p5 = cv::Point(p2.x-2,p2.y-2);
	      p6 = cv::Point(p2.x+2,p2.y-2);

	      // both white
	      if(lineColor(img_arena,p3,p5) + lineColor(img_arena,p4,p6) == 2)
		{
		  TMat[i][j] = TMat[j][i] = 1;
		  //cout << "found" << endl;
		}
	      // one white, one black
	      if(lineColor(img_arena,p3,p5) + lineColor(img_arena,p4,p6) == 1)
		adjMat[i][j] = adjMat[j][i] = 1;
	    }
	  
	  // horizontal line
	  else if(abs(corners[i].y- corners[j].y)<EPSILON)
	    {
	      p1 = (corners[i].x < corners[j].x) ? corners[i] : corners[j];
	      p2 = (p1==corners[i]) ? corners[j] : corners[i];

	      p3 = cv::Point(p1.x+2,p1.y-2);
	      p4 = cv::Point(p1.x+2,p1.y+2);
	      p5 = cv::Point(p2.x-2,p2.y-2);
	      p6 = cv::Point(p2.x-2,p2.y+2);

	      if(lineColor(img_arena,p3,p5) + lineColor(img_arena,p4,p6) == 2)
		{
		  TMat[i][j] = TMat[j][i] = 1;
		  //cout << "found" << endl;
		}
	      if(lineColor(img_arena,p3,p5) + lineColor(img_arena,p4,p6) == 1)
		adjMat[i][j] = adjMat[j][i] = 1;
	    }
	  else
	    continue;
	}
    }

  // get sequence of adjacent corner indices
  curr = next = 0;
  do
    {
      curr = next;
      seq.push_back(curr);
      used[curr] = 1;
      for(i=0;i<corners.size();++i)
	{
	  if(adjMat[curr][i] == 1 && used[i]==0)
	    {
	      next = i;
	      break;
	    }
	}
    }
  while(curr!=next);
  
  // obtain the vector of TJunctions
  for(i=0;i<n;++i)
    {
      for(j=0;j<i;++j)
	{
	  if(TMat[i][j]==1)
	    {
	      idx_i = getIndex(seq,i);
	      idx_j = getIndex(seq,j);
	      
	      p = corners[i];

	      // get corners adjacent to corner i
	      p1 = corners[seq[(idx_i+1)%n]];
	      p2 = corners[seq[idx_i>0? idx_i-1 : n-1]];
	      
	      if(abs(p1.x - p.x) < EPSILON)
		dir_i1 = p1.y < p.y ? 'U' : 'D';
	      else if(abs(p1.y - p.y) < EPSILON)
		dir_i1 = p1.x < p.x ? 'L' : 'R';
	      if(abs(p2.x - p.x) < EPSILON)
		dir_i2 = p2.y < p.y ? 'U' : 'D';
	      else if(abs(p2.y - p.y) < EPSILON)
		dir_i2 = p2.x < p.x ? 'L' : 'R';
		
	      p = corners[j];
	      p1 = corners[seq[(idx_j+1)%n]];
	      p2 = corners[seq[idx_j>0? idx_j-1 : n-1]];
	      if(abs(p1.x - p.x) < EPSILON)
		dir_j1 = p1.y < p.y ? 'U' : 'D';
	      else if(abs(p1.y - p.y) < EPSILON)
		dir_j1 = p1.x < p.x ? 'L' : 'R';
	      if(abs(p2.x - p.x) < EPSILON)
		dir_j2 = p2.y < p.y ? 'U' : 'D';
	      else if(abs(p2.y - p.y) < EPSILON)
		dir_j2 = p2.x < p.x ? 'L' : 'R';

	      // in case of a TJ, there will be exactly one matching pair (in form of a T)
	      int match;	    
	      if(dir_i1 == dir_j1 || dir_i1 == dir_j2)
		match = dir_i1;
	      else
		match = dir_i2;

	      switch(match)
		{
		case 'U' : dirT = 'D';
		  break;
		case 'L' : dirT = 'R';
		  break;
		case 'R' : dirT = 'L';
		  break;
		case 'D' : dirT = 'U';
		  break;
		}

	      cv::Point tj;
	      switch(dirT)
		{
		case 'U' : width = abs(corners[i].x - corners[j].x);
		  tj = cv::Point(0.5*(corners[i].x + corners[j].x),corners[i].y - width/2);
		  break;
		case 'L' : width = abs(corners[i].y - corners[j].y);
		  tj = cv::Point(corners[i].x - width/2,0.5*(corners[i].y + corners[j].y));
		  break;
		case 'R' : width = abs(corners[i].y - corners[j].y);
		  tj = cv::Point(corners[i].x + width/2,0.5*(corners[i].y + corners[j].y));
		  break;
		case 'D' : width = abs(corners[i].x - corners[j].x);
		  tj = cv::Point(0.5*(corners[i].x + corners[j].x),corners[i].y + width/2);
		  break;
		}
	      TJpts_dupl.push_back(tj);
	    }
	}
    }
  
  removeDuplicateTJpts(TJpts_dupl,TJpts_unique);
  
  for(i=0;i<TJpts_unique.size();++i)
      TJunctions.push_back(Landmark(TJpts_unique[i] , "TJ" , "None" , 0 , 0 ));
  
  return TJunctions;
}
