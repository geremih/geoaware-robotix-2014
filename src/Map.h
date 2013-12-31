#ifndef MAP_H
#define MAP_H

#include "GeoAware.h"
#include "Landmark.h"

using namespace std;
using namespace cv;

class Map
{
 public:
  cv::Mat img_src;
  cv::Mat img_landmarks;
  cv::Mat img_arena;
  std::vector<Landmark> landmarks;
  
  Map();
  Map(const string path);
  Map(const Map&);
  void printLandmarks();
  void displayMap();
  const Map& operator = (const Map&);

 private:
  cv::Mat obtainLandmarksImg();
  cv::Mat obtainArenaImg();
  std::vector<Landmark> getLandmarks();
  void addBorders(int blue, int green, int red);
  void addLandmark(std::vector<Landmark>& landmarks, std::vector<cv::Point>& symbol);
  void removeDuplicateContours(std::vector<std::vector<cv::Point> >& contours_dupl, std::vector<std::vector<cv::Point> >& contours);
  void cleanEdges(std::vector<cv::Point>& approx, std::vector<cv::Point>& actual);
  static string getColor(int blue, int green, int red);
  static bool isPrimaryColor(int blue, int green, int red);
  static void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);
  static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
  static string convertInt(int number);
  static string vtxToShape(int vtc);
};

#endif
