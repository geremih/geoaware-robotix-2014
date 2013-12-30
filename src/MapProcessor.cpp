#include "MapProcessor.h"

using namespace std;
using namespace cv;

MapProcessor::MapProcessor()
{
  cout << "Calling empty MapProcessor constructor" << endl;
}

MapProcessor::MapProcessor(Map inp_map)
  :  m(inp_map)
{
  landmarks = inp_map.landmarks;
  size = inp_map.landmarks.size();
  img_connections = inp_map.img_src.clone();
  adjMat = initAdjMat();

  assignIndices();
  drawIndices();
  fillAdjMat();
  drawConnections();
}

MapProcessor::~MapProcessor()
{
  for(int i=0;i<size;++i)
    delete adjMat[i];
  delete adjMat;
}

void MapProcessor::findRoute()
{
  return;
}

void MapProcessor::displayConnections()
{
  cv::imshow("connections",img_connections);
}

void MapProcessor::drawConnections()
{
  if(!adjMat)
    return;

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

void MapProcessor::assignIndices()
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

void MapProcessor::drawIndices()
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

bool MapProcessor::isConnected(Landmark l1, Landmark l2)
{
  LineIterator it(m.img_arena, l1.centroid, l2.centroid, 8);
 
  for(int i = 0; i < it.count; i++, ++it)
  {
    Vec3b intensity = m.img_arena.at<Vec3b>(it.pos());
    int blue = (int)intensity.val[0];
    int green = (int)intensity.val[1];
    int red = (int)intensity.val[2];
    if(blue==0 && green==0 && red==0)
      return false;
  }
  return true;
}

int ** MapProcessor::initAdjMat()
{
  int **adjMat = new int*[size];
  for(int i=0;i<size;++i)
    adjMat[i] = new int[size];
  return adjMat;
}

void MapProcessor::fillAdjMat()
{
  if(!adjMat)
    return;
  
  int i,j;
  for(i=0;i<landmarks.size();++i)
    {
      for(j=0;j<i;++j)
	{
	  if(isConnected(landmarks[i], landmarks[j]))
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

void MapProcessor::printLandmarks()
{
  for(int i=0;i<landmarks.size();++i)
    cout << landmarks[i] << endl;
}

string MapProcessor::convertInt(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}
