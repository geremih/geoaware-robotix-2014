#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

#define EPSILON 10
#define PRIMARY_THRESHOLD 69

using namespace cv;
using namespace std;

string vtxToShape(int vtc)
{
  if (vtc >= 8)
    return "CIRCLE";
  if(vtc == 3)
    return "TRIANGLE";
  if(vtc == 4)
    return "SQUARE";
  if(vtc == 5)
    return "PENTAGON";
  if(vtc == 6)
    return "HEXAGON";
  return "None";
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

void cleanEdges(std::vector<cv::Point>& approx, std::vector<cv::Point>& actual)
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

void detectSymbol(cv::Mat src, cv::Mat edges)
{
  std::vector<std::vector<cv::Point> > contours_dupl;
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(edges.clone(), contours_dupl, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  removeDuplicateContours(contours_dupl,contours);
  
  std::vector<cv::Point> approx;
  std::vector<cv::Point> actual;
  
  std::vector<Point2f>center( contours.size() );
  std::vector<float>radius( contours.size() );

  int vtx;
  string color;
  
  for(int i=0;i<contours.size();++i)
    {
      actual.clear();
      cv::Scalar clr = CV_RGB(rand()%255,rand()%255,rand()%255);
      cv::drawContours(src,contours,i,clr,2);
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
      cleanEdges(approx,actual);
      
      if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
      	continue;
      if(actual.size()<3)
	continue;
      
      cv::minEnclosingCircle( (Mat)actual, center[i], radius[i] );
      cv::circle( src, center[i], radius[i], clr, 2, 8, 0 );
      vtx = actual.size();
      
      Vec3b intensity = src.at<Vec3b>((int)center[i].y,(int)center[i].x);
      int blue = (int)intensity.val[0];
      int green = (int)intensity.val[1];
      int red = (int)intensity.val[2];
      color = getColor(blue,green,red);
      
      break;
      //cout << "contour " << i << " : " << endl;
      //cout << "vertices = " << actual.size() << endl;
      //cout << "center = " << center[i] << ", r = " << radius[i] << endl;
    }
  cout << "detected : shape = " << vtxToShape(vtx) << ", color = " << color << endl;
}

int main()
{
  srand(time(NULL));
  
  VideoCapture cap(0); // open the default camera
  
  if(!cap.isOpened()) // check if we succeeded
    {
      cout << "No video capture device detected!" << endl;
      return -1;
    }

  cv::Mat src, src_gray, src_gray_smooth, src_binary;
  cv::Mat edges_normal, edges_smooth;
  cv::Mat eroded, dilated;
  cv::Mat kernel = Mat::ones(Size(4, 4), CV_8U);
  
  while(true)
    {
      cap >> src;
      cv::cvtColor( src, src_gray, COLOR_RGB2GRAY );
      cv::blur( src_gray, src_gray_smooth, Size( 5, 5 ), Point(-1,-1) );

      src_binary = src_gray.clone();
      cv::threshold(src_gray, src_binary, 128.0, 255.0, THRESH_BINARY);
      
      eroded = src_binary.clone();
      cv::erode(src_binary,eroded,kernel);
      dilated = eroded.clone();
      cv::dilate(eroded,dilated,kernel);
      
      edges_normal = dilated.clone();
      cv::Canny(dilated, edges_normal, 50, 200, 3 );

      detectSymbol(src,edges_normal);
      
      cv::imshow("src",src);
      cv::imshow("edges_normal",edges_normal);
      //cv::imshow("binary", src_binary);
      //cv::imshow("eroded",eroded);
      //cv::imshow("dilated",dilated);

      if(cv::waitKey(33) == 'q')
	break;
    }
  return 0;
}
