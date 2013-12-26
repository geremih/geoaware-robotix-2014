#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <time.h>

#define PRIMARY_THRESHOLD 180
#define BLACK_THRESHOLD 65
#define EPSILON 10

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
  cout << "Drawing label " << label << " at point " << pt.x << "," << pt.y << endl;
}

bool isPrimaryColor(int blue, int green, int red)
{
  return ((blue-green>PRIMARY_THRESHOLD && blue-red>PRIMARY_THRESHOLD) || (green-blue>PRIMARY_THRESHOLD && green-red>PRIMARY_THRESHOLD) || (red-blue>PRIMARY_THRESHOLD && red-green>PRIMARY_THRESHOLD));
}

string convertInt(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

void removeDuplicateContours(std::vector<std::vector<cv::Point> >& contours_dupl, std::vector<std::vector<cv::Point> >& contours)
{
  int i,j,k,t;
  int X,Y,x,y;
  cv::Rect R,r;
  for(i=0;i<contours_dupl.size();++i)
    {
      R = cv::boundingRect(contours_dupl[i]);
      X = R.x;
      Y = R.y;
      //cout << "Contour " << i << " at " << X << "," << Y <<endl;
      for(j=0;j<contours.size();++j)
	{
	  r = cv::boundingRect(contours[j]);
	  x = r.x;
	  y = r.y;
	  if(abs(X-x)<EPSILON && abs(Y-y)<EPSILON)
	    {
	      //cout << "Found duplicate entry X,Y = " << X << "," << Y << " and x,y = " << x << "," << y << endl;
	      break;
	    }
	}
      if(j==contours.size())
	{
	  //cout << "Adding contour " << i << " to contours vector" << endl;
	  contours.push_back(contours_dupl[i]);
	}
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
  //cv::imshow("landmarks", landmarks);
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
  //cv::imshow("arena",arena);
  return arena;
}

int main()
{
  srand (time(NULL));
  
  const string path = "../assets/map3.png";
  cv::Mat src = cv::imread(path);
  if (src.empty())
    return -1;
  
  // Isolate the landmarks
  cv::Mat landmarks = obtainLandmarks(src);
  
  // Get the traversible part of the arena
  cv::Mat arena = obtainArena(src);
  
  // Convert to grayscale
  cv::Mat gray;
  cv::cvtColor(landmarks, gray, CV_BGR2GRAY);
  
  // Use Canny instead of threshold to catch squares with gradient shading
  cv::Mat bw;
  cv::Canny(gray, bw, 0, 50, 5);
  
  // Find contours
  std::vector<std::vector<cv::Point> > contours_dupl;
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(bw.clone(), contours_dupl, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  removeDuplicateContours(contours_dupl,contours);
  
  std::vector<cv::Point> approx;
  std::vector<cv::Point> actual;
  cv::Mat dst = landmarks.clone();
  
  cout << "Number of contours found : " << contours.size() << endl;
  
  for (int i = 0; i < contours.size(); i++)
    {
      // Approximate contour with accuracy proportional to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

      // Skip small or non-convex objects
      if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
	continue;
     
      cout << "Processing contour " << i << ":" << endl;
      
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
	      //cout << a << " -> " << b << endl;
	      double res = cv::norm(a-b);
	      if(res>=EPSILON)
		  actual.push_back(approx[j]);
	      ++j;
	    }
	  cout << "Approx Size = " << approx.size() << " -> Actual Size = " << actual.size() << endl;
	  vtc = actual.size();
	  if(vtc==3)
	    {
	      setLabel(dst, string("TRI").append(convertInt(i)), contours[i]);
	      //line(dst,actual[0],actual[1],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[1],actual[2],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[2],actual[0],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	    }
	  if (vtc == 4)
	    {
	      setLabel(dst, string("RECT").append(convertInt(i)), contours[i]);
	      //line(dst,actual[0],actual[1],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[1],actual[2],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[2],actual[3],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[3],actual[0],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	    }
	  if (vtc == 5)
	    {
	      setLabel(dst, string("PENTA").append(convertInt(i)), contours[i]);
	      //line(dst,actual[0],actual[1],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[1],actual[2],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[2],actual[3],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[3],actual[4],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[4],actual[0],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	    }
	  if (vtc == 6)
	    {
	      setLabel(dst, string("HEXA").append(convertInt(i)), contours[i]);
	      //line(dst,actual[0],actual[1],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[1],actual[2],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[2],actual[3],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[3],actual[4],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[4],actual[5],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	      //line(dst,actual[5],actual[0],CV_RGB(rand()%255,rand()%255,rand()%255),3,8);
	    }
	}
      else
	{
	  // Detect and label circles
	  double area = cv::contourArea(contours[i]);
	  cv::Rect r = cv::boundingRect(contours[i]);
	  float radius = r.width / 2;
	  
	  if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
	    setLabel(dst, string("CIR").append(convertInt(i)), contours[i]);
	}
      actual.clear();
      cout << endl;
    }
  
  
  cv::imshow("src", src);
  cv::imshow("bw", bw);
  cv::imshow("dst", dst);
  cv::waitKey(0);
  return 0;
}
