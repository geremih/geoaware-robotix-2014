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
  addBorders(0,0,0);
  img_landmarks = obtainLandmarksImg();
  img_arena = obtainArenaImg();
  landmarks = getLandmarks();
}

Map::Map(const Map& M)
{
  img_src = M.img_src.clone();
  img_landmarks = M.img_landmarks.clone();
  img_arena = M.img_arena.clone();
  landmarks = M.landmarks;
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
  for(int i=0;i<landmarks.size();++i)
    cout << landmarks[i] << endl;
}

void Map::addLandmark(std::vector<Landmark>& landmarks, std::vector<cv::Point>& symbol)
{
  cv::Point2f center(0.f,0.f);
  float radius = 0.f;
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

  cout << "Number of contours found : " << contours.size() << endl;

  //Get all the landmarks from the contours
  for (int i = 0; i < contours.size(); i++)
    {      
      cv::drawContours(img_landmarks,contours,i,CV_RGB(255,255,255),2);
      
      // Approximate contour with accuracy proportional to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, false);

      // Skip small or non-convex objects
      /*if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
	{
	  cout << "Skipping contour " << i << endl;
	  continue;
	  }*/
      
      cleanEdges(approx,actual);
      
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
  cv::imshow("src",img_src);
  cv::imshow("arena",img_arena);
  cv::imshow("landmarks",img_landmarks);
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
