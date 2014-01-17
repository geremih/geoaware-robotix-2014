#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

#define EPSILON 10
#define EPSILON_TJ 25

using namespace cv;
using namespace std;

int thresh = 85;
int max_thresh = 255;

cv::Mat img_src,img_arena,img_edges,erosion_dst;
char corners_window[] = "Corners";

std::vector<cv::Point> corners;

cv::Mat obtainArenaImg(cv::Mat img_src)
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

//adds 2 px wide border around the image to take care of boundary contours
void addBorders(cv::Mat& img_src,int blue, int green, int red)
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

bool addCorner(cv::Point pt)
{
  for(int i=0;i<corners.size();++i)
    {
      if(abs(corners[i].x - pt.x) < EPSILON && abs(corners[i].y - pt.y) < EPSILON)
	return false;
    }
  corners.push_back(pt);
  return true;
}

// returns 1 if all white
//         0 if all black
//         -1 otherwise
int isUniform(cv::Mat img, cv::Point2f p1, cv::Point2f p2)
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

void cornerHarris_demo( int, void* )
{
  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( img_arena.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  
  cv::Mat img_arena_gray;
  cvtColor( img_arena, img_arena_gray, CV_BGR2GRAY );
  /// Detecting corners
  cornerHarris( img_arena_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );
  
  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
    { for( int i = 0; i < dst_norm.cols; i++ )
	{
	  if( (int) dst_norm.at<float>(j,i) > thresh )
	    {
	      if(addCorner(cv::Point(i,j)))
		  circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
	    }
	}
    }
  /// Showing the result
  namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
  imshow( corners_window, dst_norm_scaled );
}

int getIndex(std::vector<int> vec, int val)
{
  int i;
  for(i=0;i<vec.size();++i)
    {
      if(vec[i] == val)
	return i;
    }
  return -1;
}

void removeDuplicateTJ(std::vector<cv::Point>& TJ_dupl, std::vector<cv::Point>& TJ)
{
  int i,j;
  int X,Y,x,y;
  cv::Rect R,r;
  for(i=0;i<TJ_dupl.size();++i)
    {
      X = TJ_dupl[i].x;
      Y = TJ_dupl[i].y;
      for(j=0;j<TJ.size();++j)
	{
	  x = TJ[j].x;
	  y = TJ[j].y;
	  if(abs(X-x)<EPSILON_TJ && abs(Y-y)<EPSILON_TJ)
	    break;
	}
      if(j==TJ.size())
	  TJ.push_back(TJ_dupl[i]);
    }
}

std::vector<cv::Point> findTJunctions()
{
  int i,j,k,l;
  int width;
  cv::Mat img_conn = img_arena.clone();
  cv::Point p,p1,p2,p3,p4,p5,p6;
  
  std::vector<std::vector<int> > TMat(corners.size(), std::vector<int>(corners.size(),0));
  std::vector<std::vector<int> > adjMat(corners.size(), std::vector<int>(corners.size(),0));
  
  for(i=0;i<corners.size();++i)
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

	      if(isUniform(img_arena,p3,p5) + isUniform(img_arena,p4,p6) == 2)
		TMat[i][j] = TMat[j][i] = 1;
	      if(isUniform(img_arena,p3,p5) + isUniform(img_arena,p4,p6) == 1)
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

	      if(isUniform(img_arena,p3,p5) + isUniform(img_arena,p4,p6) == 2)
		TMat[i][j] = TMat[j][i] = 1;
	      if(isUniform(img_arena,p3,p5) + isUniform(img_arena,p4,p6) == 1)
		adjMat[i][j] = adjMat[j][i] = 1;
	    }
	  else
	    continue;
	}
    }
  
  std::vector<int> seq;
  std::vector<int> used(corners.size(),0);
  int curr = 0;
  int next = 0;
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

  int idx_i, idx_j;
  int n = corners.size();
  char dir_i1,dir_i2,dir_j1,dir_j2;
  char dirT;
  std::vector<cv::Point> TJunctions;
  
  for(i=0;i<n;++i)
    {
      for(j=0;j<i;++j)
	{
	  if(TMat[i][j]==1)
	    {
	      idx_i = getIndex(seq,i);
	      idx_j = getIndex(seq,j);
	      
	      p = corners[i];
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

	      cv::Point TJ;
	      Scalar clr;
	      switch(dirT)
		{
		case 'U' : width = abs(corners[i].x - corners[j].x);
		  TJ = cv::Point(0.5*(corners[i].x + corners[j].x),corners[i].y - width/2);
		  break;
		case 'L' : width = abs(corners[i].y - corners[j].y);
		  TJ = cv::Point(corners[i].x - width/2,0.5*(corners[i].y + corners[j].y));
		  break;
		case 'R' : width = abs(corners[i].y - corners[j].y);
		  TJ = cv::Point(corners[i].x + width/2,0.5*(corners[i].y + corners[j].y));
		  break;
		case 'D' : width = abs(corners[i].x - corners[j].x);
		  TJ = cv::Point(0.5*(corners[i].x + corners[j].x),corners[i].y + width/2);
		  break;
		}
	      TJunctions.push_back(TJ);

	    }
	
	}
    }
  std::vector<cv::Point> TJunctions_unique;
  removeDuplicateTJ(TJunctions,TJunctions_unique);
  return TJunctions_unique;
}

void drawTJ(std::vector<cv::Point>& TJ)
{
  cv::Mat img_TJ = img_arena.clone();
  for(int i=0;i<TJ.size();++i)
    circle(img_TJ,TJ[i],5,Scalar(0),2,8,0);
  cv::imshow("TJ",img_TJ);
}

int main(int argc, char *argv[])
{
  string path(argv[1]);
  img_src =  cv::imread(path);
  if(img_src.empty())
    {
      cout << "Invalid input map! Exiting. " << endl;
      exit(EXIT_FAILURE);
    }

  addBorders(img_src,0,0,0);
  img_arena = obtainArenaImg(img_src);
  
  cv::imshow("Arena",img_arena);

  createTrackbar( "Threshold: ", "Arena", &thresh, max_thresh, cornerHarris_demo );
  cornerHarris_demo( 0, 0 ); 
  
  std::vector<cv::Point> TJunctions = findTJunctions();
  drawTJ(TJunctions);
  
  cv::waitKey(0);
  return 0;
}
