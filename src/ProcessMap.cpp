#include "GeoAware.h"
#include "Landmark.h"

using namespace cv;
using namespace std;

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
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
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
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

bool isPrimaryColor(int blue, int green, int red)
{
  return ((blue-green>PRIMARY_THRESHOLD && blue-red>PRIMARY_THRESHOLD) || (green-blue>PRIMARY_THRESHOLD && green-red>PRIMARY_THRESHOLD) || (red-blue>PRIMARY_THRESHOLD && red-green>PRIMARY_THRESHOLD));
}

string getColor(int blue, int green, int red)
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
string convertInt(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

void removeDuplicateContours(std::vector<std::vector<cv::Point> >& contours_dupl, std::vector<std::vector<cv::Point> >& contours)
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

cv::Mat obtainLandmarks(cv::Mat src)
{
  cv::Mat landmarks( src.rows, src.cols, CV_8UC3, CV_RGB(0,0,0));
  for(int i=0; i<src.rows; i++)
    {
      for(int j=0; j<src.cols; j++)
	{
	  //BGR
	  Vec3b intensity = src.at<Vec3b>(i,j);
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

cv::Mat obtainArena(cv::Mat src)
{
  cv::Mat arena(src.rows,src.cols, CV_8UC3, CV_RGB(0,0,0));
  for(int i=0;i<src.rows;++i)
    {
      for(int j=0;j<src.cols;++j)
	{
	  Vec3b intensity = src.at<Vec3b>(i,j);
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

void addLandmark(std::vector<Landmark>& landmarks, std::vector<cv::Point>& symbol, cv::Mat img_landmarks)
{
  cv::Point2f center(0.f,0.f);
  float radius = 0.f;
  cv::minEnclosingCircle(symbol,center,radius);
  //circle( img_landmarks, center, (int)radius, CV_RGB(100,100,200), 2, 8, 0 );
  
  int vtc = symbol.size();
  vtc = (vtc > HEXAGON) ? CIRCLE : vtc;
  string shape = vtxToShape[vtc];

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

void printLandmarks(std::vector<Landmark>& landmarks)
{
  for(int i=0;i<landmarks.size();++i)
    cout << landmarks[i] << endl;
}

void initVtxToShape()
{
  vtxToShape[CIRCLE] = "CIRCLE";
  vtxToShape[TRIANGLE] = "TRIANGLE";
  vtxToShape[SQUARE] = "SQUARE";
  vtxToShape[PENTAGON] = "PENTAGON";
  vtxToShape[HEXAGON] = "HEXAGON";
}

void drawConnections(cv::Mat img_connections, std::vector<Landmark>& landmarks, int **adjMat)
{
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

void assignIndices(std::vector<Landmark>& landmarks)
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

void displayIndices(cv::Mat img_connections, std::vector<Landmark>& landmarks)
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

bool isConnected(Landmark l1, Landmark l2, cv::Mat img_arena)
{
  LineIterator it(img_arena, l1.centroid, l2.centroid, 8);
 
  for(int i = 0; i < it.count; i++, ++it)
  {
    Vec3b intensity = img_arena.at<Vec3b>(it.pos());
    int blue = (int)intensity.val[0];
    int green = (int)intensity.val[1];
    int red = (int)intensity.val[2];
    if(blue==0 && green==0 && red==0)
      return false;
  }
  return true;
}

int ** initAdjMat(int s)
{
  int ** adjMat = new int*[s];
  for(int i=0;i<s;++i)
    adjMat[i] = new int[s];
  return adjMat;
}

void fillAdjMat(int **adjMat, std::vector<Landmark>& landmarks, cv::Mat img_arena)
{
  if(!adjMat)
    return;
  int i,j;
  for(i=0;i<landmarks.size();++i)
    {
      for(j=0;j<i;++j)
	{
	  if(isConnected(landmarks[i], landmarks[j], img_arena))
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
}

int main()
{
  initVtxToShape();

  srand (time(NULL));
  
  const string path = "../assets/map3.png";
  cv::Mat img_src = cv::imread(path);
  if (img_src.empty())
    return -1;
  
  // Isolate the landmarks
  cv::Mat img_landmarks = obtainLandmarks(img_src);
  
  // Get the traversible part of the arena
  cv::Mat img_arena = obtainArena(img_src);
  
  // Convert to grayscale
  cv::Mat img_gray;
  cv::cvtColor(img_landmarks, img_gray, CV_BGR2GRAY);
  
  // Use Canny instead of threshold to catch squares with gradient shading
  cv::Mat img_bw;
  cv::Canny(img_gray, img_bw, 0, 50, 5);

  // img_dst holds the final image with labels
  cv::Mat img_dst = img_landmarks.clone();
  
  // Find contours
  std::vector<std::vector<cv::Point> > contours_dupl;
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(img_bw.clone(), contours_dupl, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  removeDuplicateContours(contours_dupl,contours);
  
  // approx holds an individual contour found by approxPolyDP
  std::vector<cv::Point> approx;
  
  // actual removes the small edges from approx so that actual shape is obtained
  std::vector<cv::Point> actual;
 
  // Vector of all the landmarks
  std::vector<Landmark> landmarks;
  
  cout << "Number of contours found : " << contours.size() << endl;
  
  for (int i = 0; i < contours.size(); i++)
    {
      // Approximate contour with accuracy proportional to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

      // Skip small or non-convex objects
      if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
	continue;
     
      //cout << "Processing contour " << i << ":" << endl;
      
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
	  vtc = actual.size();
	  if(vtc==3)
	    setLabel(img_dst, string("TRI").append(convertInt(i)), contours[i]);
	  if (vtc == 4)
	    setLabel(img_dst, string("RECT").append(convertInt(i)), contours[i]);
	  if (vtc == 5)
	    setLabel(img_dst, string("PENTA").append(convertInt(i)), contours[i]);
	  if (vtc == 6)
	    setLabel(img_dst, string("HEXA").append(convertInt(i)), contours[i]);	  
	}
      else
	{
	  actual = approx;

	  // Detect and label circles
	  double area = cv::contourArea(contours[i]);
	  cv::Rect r = cv::boundingRect(contours[i]);
	  float radius = r.width / 2;
	  
	  if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
	    setLabel(img_dst, string("CIR").append(convertInt(i)), contours[i]);
	}

      addLandmark(landmarks,actual,img_landmarks);
      
      actual.clear();
    }
  
  cv::Mat img_connections = img_src.clone();
  
  assignIndices(landmarks);
  displayIndices(img_connections,landmarks);
  printLandmarks(landmarks);

  int **adjMat = initAdjMat(landmarks.size());
  fillAdjMat(adjMat,landmarks,img_arena);
  drawConnections(img_connections,landmarks,adjMat);
  
  cv::imshow("landmarks", img_landmarks);
  cv::imshow("src", img_src);
  //cv::imshow("bw", img_bw);
  //cv::imshow("dst", img_dst);
  cv::imshow("connections",img_connections);

  cv::waitKey(0);

  return 0;
}
