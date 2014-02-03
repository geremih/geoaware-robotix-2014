#ifndef MAP_H
#define MAP_H

#include "GeoAware.h"
#include "Landmark.h"
#include "TJunction.h"

#define MIN_AREA 5

using namespace std;
using namespace cv;

class Map
{
 public:
  cv::Mat img_src;
  cv::Mat img_landmarks;
  cv::Mat img_arena;
  cv::Mat img_waypoints;
  cv::Mat img_tjs;
  std::vector<Landmark> landmarks;
  // made both as landmark for now
  std::vector<Landmark> TJs;
  
  Map();
  Map(const string path);
  Map(const Map&);
  void printLandmarks();
  void displayMap();
  const Map& operator = (const Map&);

 private:
  std::vector<cv::Point> corners;
  
  cv::Mat obtainLandmarksImg();
  cv::Mat obtainArenaImg();
  
  std::vector<cv::Point> getCorners(cv::Mat);
  std::vector<Landmark> getLandmarks();
  std::vector<Landmark> getTJs();
  
  void addBorders(int blue, int green, int red);
  void addLandmark(std::vector<Landmark>& landmarks, std::vector<cv::Point>& symbol);
  void removeDuplicateContours(std::vector<std::vector<cv::Point> >& contours_dupl, std::vector<std::vector<cv::Point> >& contours);
  void removeDuplicateTJpts(std::vector<cv::Point>& TJs_dupl, std::vector<cv::Point>& TJs);
  void cleanEdges(std::vector<cv::Point>& approx, std::vector<cv::Point>& actual);
  void drawTJs();
  
  bool addCorner(std::vector<cv::Point>& corners, cv::Point pt);
  int lineColor(cv::Mat img, cv::Point2f p1, cv::Point2f p2);
  int getIndex(std::vector<int> vec, int val);
    
  static string getColor(int blue, int green, int red);
  static bool isPrimaryColor(int blue, int green, int red);
  static void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);
  static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
  static string convertInt(int number);
  static string vtxToShape(int vtc);
};

#endif
