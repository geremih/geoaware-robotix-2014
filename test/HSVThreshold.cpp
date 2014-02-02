#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;


Mat src;
int hl = 10;
int hh = 100;
int sl = 10;
int sh = 100;
int vl = 10;
int vh = 100;

void Threshold(int, void*)
{

  Mat hsv;
  cvtColor(src ,hsv ,CV_BGR2HSV);
  Mat mask;
  inRange(hsv, Scalar(hl , sl ,vl), Scalar(hh , sh , vh) , mask);
  //inRange(hsv, Scalar(100 , 100 ,100), Scalar(160 , 256 , 256) , mask);
  imshow(  "Video", src);
  imshow( "Red Mask" , mask);
  /// Reduce noise with a kernel 3x3
  imshow( "Mask", mask );
 }

int main()
{
  srand(time(NULL));
  
  VideoCapture cap(1); // open the default camera
  
  if(!cap.isOpened()) // check if we succeeded
    {
      cout << "No video capture device detected!" << endl;
      return -1;
    }

  cv::Mat edges_normal, edges_blur;
  cv::Mat eroded, dilated, dilated_blur;
  cv::Mat kernel = Mat::ones(Size(7, 7), CV_8U);

          namedWindow( "Trackbars", CV_WINDOW_AUTOSIZE );
        
        createTrackbar( "HL:", "Trackbars", &hl , 180, Threshold );
        createTrackbar( "SL:", "Trackbars", &sl , 256, Threshold );
        createTrackbar( "VL:", "Trackbars", &vl , 256, Threshold );
        createTrackbar( "HH:", "Trackbars", &hh , 180, Threshold );
        createTrackbar( "SH:", "Trackbars", &sh , 256, Threshold );
        createTrackbar( "VH:", "Trackbars", &vh , 256, Threshold );
        

  while(true)
    {
      cap >> src;
      
      //src = cv::imread("../assets/samples/symbols/shape_3.jpg");
      
      //detectSymbol(src, src_thresh, edges_normal);
        
        Threshold( 0 ,0);
      if(cv::waitKey(33) == 'q')
        break;
    }
  return 0;
}
