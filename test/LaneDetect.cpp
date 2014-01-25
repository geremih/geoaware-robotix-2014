/*------------------------------------------------------------------------------------------*\
  Lane Detection

  General idea and some code modified from:
  chapter 7 of Computer Vision Programming using the OpenCV Library. 
  by Robert Laganiere, Packt Publishing, 2011.

  This program is free software; permission is hereby granted to use, copy, modify, 
  and distribute this source code, or portions thereof, for any purpose, without fee, 
  subject to the restriction that the copyright notice may not be removed 
  or altered from any source or altered source distribution. 
  The software is released on an as-is basis and without any warranties of any kind. 
  In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
  The author disclaims all warranties with regard to this software, any use, 
  and any consequent failure, is purely the responsibility of the user.
 
  Copyright (C) 2013 Jason Dorweiler, www.transistor.io


  Notes: 

	Add up number on lines that are found within a threshold of a given rho,theta and 
	use that to determine a score.  Only lines with a good enough score are kept. 

	Calculation for the distance of the car from the center.  This should also determine
	if the road in turning.  We might not want to be in the center of the road for a turn. 
	
	Several other parameters can be played with: min vote on houghp, line distance and gap.  Some
	type of feed back loop might be good to self tune these parameters. 

	We are still finding the Road, i.e. both left and right lanes.  we Need to set it up to find the
	yellow divider line in the middle. 

	Added filter on theta angle to reduce horizontal and vertical lines. 

	Added image ROI to reduce false lines from things like trees/powerlines
  \*------------------------------------------------------------------------------------------*/

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include "linefinder.h"

#define PI 3.1415926

using namespace cv;
using namespace std;


float abs( float x , float y){
  if((x-y) < 0)
    return y-x;
  else
    return x-y;

}

vector<Vec4i> getLineSegments( Mat& edgeIm , vector<Vec2f> clines){

  vector<Vec2f> lines;
  bool exists = false;

  //Removing lines close to each other
  for(int i = 0; i < clines.size() ; i++){
    exists = false;
    float crho = clines[i][0], ctheta = clines[i][1];
    for( int j = 0 ; j < lines.size() ; j++){
      float rho = lines[j][0], theta = lines[j][1];
      if( abs(rho - crho) <= 10 && abs(theta - ctheta) <= .05){
        crho = (rho + crho) /2;
        ctheta = (theta + ctheta)/2;
        exists = true;
      }
    }
    if(!exists){
      lines.push_back(Vec2f( crho , ctheta));
    }
  }
  cout<<"There are total " <<  lines.size() << " lines"<<endl;
  imshow("Getting line segments on " , edgeIm);
  //remove contours that are close by
  int kernel = 2;
  vector<Vec4i> segments;
  Point p1 , p2;
  Mat lineTest;
  edgeIm.copyTo(lineTest);
  //equation is p = xcostheta + ysintheta
  for(int i = 0 ; i< lines.size() ; i++){

    float rho = lines[i][0], theta = lines[i][1];
    double a = cos(theta), b = sin(theta);
    bool setP1 , setP2;
    setP1 = setP2 = false;

    double y = 0;
    double  x = (rho /a) - y*( b/a);

    if(x >=0 && x< edgeIm.cols){
      if(!setP1){
        p1 = Point( x , y);
        setP1 = true;
      }
      else if(!setP2){
        p2 = Point(x , y);
        setP2 = true;
      }
      else
        cout<<"Error"<<endl;
    }
    y = edgeIm.rows -1;
    x = (rho /a) - y*( b/a);

    if(x >=0 && x< edgeIm.cols){
      if(!setP1){
        p1 = Point( x , y);
        setP1 = true;
      }
      else if(!setP2){
        p2 = Point(x , y);
        setP2 = true;

      }
      else cout<<"Error"<<endl;
    }

    x = edgeIm.cols -1;
    y = (rho /b) - x*( a/b);
    

    if(y >=0 && y< edgeIm.rows){
      if(!setP1){
        p1 = Point( x , y);
        setP1 = true;
      }
      else if(!setP2){
        p2 = Point(x , y);
        setP2 = true;
      }
      else cout<<"Error"<<endl;
    }
    x = 0;
    y = (rho /b) - x*( a/b);

    if(y >=0 && y< edgeIm.rows){
      if(!setP1){
        p1 = Point( x , y);
        setP1 = true;
      }
      else if(!setP2){
        p2 = Point(x , y);
        setP2 = true;
      }
      else cout<<"Error"<<endl;
    }

    //Find the points using the funda that the line will touch two edges
 
    LineIterator lit( edgeIm , p1 , p2 );
    Point start_point , end_point;
    bool isOnLine = false;
    Point curr_pos;
    for(int j = 0; j < lit.count; j++, ++lit)
      {
        curr_pos = lit.pos();
        bool isBlack = false;
        for (int xi = curr_pos.x  - kernel ; xi <= curr_pos.x + kernel ; xi++)
          {
            for (int yi = curr_pos.y  - kernel ; yi <= curr_pos.y + kernel ; yi++)
              {
                if( xi < 0 || xi >= edgeIm.cols || yi < 0 || yi >= edgeIm.rows )
                  continue;
                Scalar intensity = edgeIm.at<uchar>(yi , xi);
                      
                if( intensity.val[0] == 0){
                  isBlack = true;
                }
              }
          }
        if(isBlack){
          if(isOnLine == false){
            start_point = curr_pos;
            isOnLine =true;
          }
        }
        else{
          if(isOnLine){
            end_point = curr_pos;
            isOnLine = false;
            segments.push_back(Vec4i(start_point.x , start_point.y , end_point.x , end_point.y ));
          }
        }
        
      }
    //end checking. just in case
    if(isOnLine){
      end_point = curr_pos;
      isOnLine = false;
      segments.push_back(Vec4i(start_point.x , start_point.y , end_point.x , end_point.y ));
    }
  }

  vector<Vec4i> finalsegments;
  for(int i =0 ; i < segments.size(); i++)
    {
      if(cv::norm(cv::Mat(Point(segments[i][0] , segments[i][1])),cv::Mat(Point(segments[2][0] , segments[i][3]))) > 10)
        finalsegments.push_back(segments[i]);

    }
  cout<<"There are total " <<finalsegments.size() << " segments"<<endl;
  return finalsegments;
}


void drawLineSegments(Mat& ime , vector<Vec4i> segments ,  cv::Scalar color=cv::Scalar(255)){

  // Draw the lines
  std::vector<cv::Vec4i>::const_iterator it2= segments.begin();
  while (it2!=segments.end()) {
    cv::Point pt1((*it2)[0],(*it2)[1]);        
    cv::Point pt2((*it2)[2],(*it2)[3]);
    cv::line( ime, pt1, pt2, color, 10 );
    ++it2;	
  }
}


int houghVote = 60;
int cannyLower = 50;
int cannyHigher = 250;
Mat image;
string argp;
VideoCapture capture;


void alignToLane(){

  


}

void centerBot( Mat ){


}


void LaneDetect(){

  if (image.empty())
    return;
  Mat gray;
  //remove colors

  cvtColor(image,gray,CV_RGB2GRAY);
  Rect roi(0,image.rows/3,image.cols-1,image.rows - image.rows/3);// set the ROI for the image
  //set ROI dynamically
  Mat imgROI = image(roi);
  //Mat imgROI = image;
  // Display the image
  imshow("Original Image",imgROI );
  moveWindow("Original Image" , 400 , 0);
  // Canny algorithm
  Mat contours;
  Canny(imgROI,contours,cannyLower,cannyHigher);
  Mat contoursInv;
  threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
  imshow("Canny",contoursInv);
      
  std::vector<Vec2f> lines;
  HoughLines(contours,lines,1,PI/180, houghVote);
  Mat result(imgROI.size(),CV_8U,Scalar(255));
  imgROI.copyTo(result);
  // Draw the limes
  std::vector<Vec2f>::const_iterator it= lines.begin();
  Mat hough(imgROI.size(),CV_8U,Scalar(0));
  while (it!=lines.end()) {
          
    float rho= (*it)[0];   // first element is distance rho
    float theta= (*it)[1]; // second element is angle theta

    Point pt1(rho/cos(theta),0);        
    // point of intersection of the line with last row
    Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
    // draw a white line
    line( result, pt1, pt2, Scalar(255), 1); 
    line( hough, pt1, pt2, Scalar(255), 1);
    ++it;
  }
  // Display the detected line image
  imshow("Detected Lines with Hough",result);
  drawLineSegments(contoursInv ,getLineSegments( contoursInv, lines) , Scalar(0));
  imshow("Personal algo", contoursInv);
  lines.clear();
  waitKey(1000);
}

void on_trackbar( int, void* )
{
  LaneDetect();
}
int main(int argc, char* argv[])
{

  argp = argv[1];
  capture = VideoCapture(argp);

  if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
    {capture.open(atoi(argp.c_str()));}
  double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
  double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
  Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
  image = imread(argv[1]);	
  namedWindow("Trackbars" , 1);
  createTrackbar( "Hough Vote" , "Trackbars", &houghVote, 400, on_trackbar );
  createTrackbar( "cannyLower" , "Trackbars", &cannyLower, 400, on_trackbar );
  createTrackbar( "cannyHigher" , "Trackbars", &cannyHigher, 400, on_trackbar );


  while (1)
    {
      capture >> image;
      LaneDetect();
      char key = (char) waitKey(10000);
    }
}




