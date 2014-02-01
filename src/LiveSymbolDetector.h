#ifndef LIVE_SYMBOL_DETECTOR_H
#define LIVE_SYMBOL_DETECTOR_H

#include "GeoAware.h"
#include <iostream>
#include <stdio.h>

#define WHITE_THRESHOLD 175
#define MAXVAL 255
#define THRESH 100

using namespace cv;
using namespace std;

class LiveSymbolDetector
{
 public:
  LiveSymbolDetector();
  Point getSymbol(cv::Mat frame, string& shape, string& color);
 private:
  static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
  string vtxToShape(int vtc);
  bool isPrimaryColor(int blue, int green, int red);
  string getColor(int blue, int green, int red);
  void removeDuplicateContours(std::vector<std::vector<cv::Point> >& contours_dupl, std::vector<std::vector<cv::Point> >& contours);
  void cleanEdges(std::vector<cv::Point>& approx, std::vector<cv::Point>& actual);
  void addShape(cv::Mat src_thresh, std::vector<cv::Point2f>& centers, std::vector<float>& radii, std::vector<double>& areas, std::vector<string>& shapes, std::vector<string>& colors, std::vector<cv::Point>& actual);
  Point detectSymbol(cv::Mat src, cv::Mat src_thresh, cv::Mat edges, string& shape, string& color);
  void customThreshold(cv::Mat& src_color, cv::Mat& dst, int thresh, int maxval);
};

#endif
