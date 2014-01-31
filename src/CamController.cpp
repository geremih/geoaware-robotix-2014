#include "CamController.h"



#define TUNNEL_GAP 50
#define LANE_ANGLE_THRESH 30
#define LINE_RHO_THRESH 50
#define LINE_ANGLE_THRESH 10


using namespace std;
using namespace cv;


float abs( float x , float y){
  if((x-y) < 0)
    return y-x;
  else
    return x-y;
}

vector<Vec6f> getLineSegments( Mat& edgeIm , vector<Vec2f> clines){


  
  vector<Vec2f> lines;
  bool exists = false;

  vector<Vec2f> hvlines;
  cout<<"Crho ctheta are"<<endl;
  //Removing lines close to each other
  for(int i = 0; i < clines.size() ; i++){

    exists = false;
    float crho = clines[i][0], ctheta = clines[i][1];

    cout<< crho << " " << ctheta * 180 / PI<<endl;

    //remove nearly horizontal or vertical lines
    if( ctheta * 180/PI < 10 || (ctheta * 180/PI < 100 && ctheta * 180/PI > 80) || (ctheta * 180/PI > 170) )
      {
        hvlines.push_back( Vec2f( crho , ctheta));
        continue;

      }
    for( int j = 0 ; j < lines.size() ; j++){
      float rho = lines[j][0], theta = lines[j][1];
      if( abs(rho - crho) <= LINE_RHO_THRESH && abs(theta - ctheta)*180/PI <= LINE_ANGLE_THRESH){
        crho = (rho + crho) /2;
        ctheta = (theta + ctheta)/2;
        exists = true;
      }
    }
    if(!exists){
      lines.push_back(Vec2f( crho , ctheta));
    }
  }

  for( int j = 0 ; j < lines.size() ; j++){
    float rho = lines[j][0], theta = lines[j][1];
    cout<<j<<  " Rho " <<  rho << "Theta " << theta << endl;
  }
 
  cout<<"There are total " <<  lines.size() << " lines"<<endl;
  imshow("Getting line segments on " , edgeIm);

  int whitekernel = 1;
  //remove hor and vert lines
  for( size_t i = 0; i < hvlines.size(); i++ )
    {
      float rho = hvlines[i][0], theta = hvlines[i][1];
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b)); //??
      pt1.y = cvRound(y0 + 1000*(a)); //??
      pt2.x = cvRound(x0 - 1000*(-b)); //??
      pt2.y = cvRound(y0 - 1000*(a)); //??
      LineIterator wit( edgeIm , pt1 , pt2 );
      Point curr_pos;
      for(int j = 0; j < wit.count; j++, ++wit)
        {
          curr_pos = wit.pos();
          for (int xi = curr_pos.x  - whitekernel ; xi <= curr_pos.x + whitekernel ; xi++)
            {
              for (int yi = curr_pos.y  - whitekernel ; yi <= curr_pos.y + whitekernel ; yi++)
                {
                  if( xi < 0 || xi >= edgeIm.cols || yi < 0 || yi >= edgeIm.rows )
                    continue;
                  edgeIm.at<uchar>(yi , xi) = 255;
                }
            }
        }
    }
  //remove contours that are close by
  int kernel = 10;
  vector<Vec6f> segments;
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
            segments.push_back(Vec6f(start_point.x , start_point.y , end_point.x , end_point.y  , rho , theta));
          }
        }
        
      }
    //end checking. just in case
    if(isOnLine){
      end_point = curr_pos;
      isOnLine = false;
      segments.push_back(Vec6f(start_point.x , start_point.y , end_point.x , end_point.y ,rho , theta));
    }
  }

  vector<Vec6f> finalsegments;
  for(int i =0 ; i < segments.size(); i++)
    {
      double length =  cv::norm(cv::Mat(Point(segments[i][0] , segments[i][1])),cv::Mat(Point(segments[i][2] , segments[i][3])));
      if(length > kernel*5)
        {
          finalsegments.push_back(segments[i]);
          cout<<length<<endl;
        }

    }

  
  cout<<"There are total " <<finalsegments.size() << " segments"<<endl;


  return finalsegments;
}



void drawLineSegments(Mat& ime , vector<Vec6f> segments ,  cv::Scalar color=cv::Scalar(255)){

  // Draw the lines
  std::vector<cv::Vec6f>::const_iterator it2= segments.begin();
  while (it2!=segments.end()) {
    cv::Point pt1((*it2)[0],(*it2)[1]);        
    cv::Point pt2((*it2)[2],(*it2)[3]);
    cv::line( ime, pt1, pt2, color, 10 );
    ++it2;	
  }
}



void removeSymbols(Mat& img){
  
  Mat hsv;
  cvtColor(img ,hsv ,CV_RGB2HSV);

  Mat edges;
  std::vector<std::vector<cv::Point> > contours;
  cv::Canny(img, edges, 50, 200, 3 );

  cv::findContours(edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

  for( int i=0;i<contours.size();++i)
    {

      int colored = 0;
      int black =1;
      int white = 1;
      Scalar background;
      Rect bRect =  boundingRect( contours[i]);
      for( int j = bRect.x ; j< bRect.x + bRect.width ; j++)
        for(int k = bRect.y ; k < bRect.y + bRect.height; k++){
          Vec3b intensity = hsv.at<Vec3b>(k , j);
          int hue = (int)intensity.val[0];
          int sat = (int)intensity.val[1];
          int val = (int)intensity.val[2];
          if(sat > 50 && val > 50)
            colored++;
          else if( val < 10)
            black++;
          else{
            white++;
          }
        }

      if(colored > black + white){
        if(black > white)
          rectangle(img,  bRect, Scalar(0 ,0 ,0) ,-1 );
        else
          rectangle(img, bRect , Scalar(161 , 161 , 161) , -1 );
      }
      
    }
  
  imshow("Removed symbols from frames", img);
}

void CamController::detectTunnel(vector<Vec6f> segments , bool& pLeft , bool& pRight){

  pLeft = pRight = false;
  int tunnelGap= TUNNEL_GAP;
  for ( int i =0 ; i< segments.size(); i++)
    for( int j = i+1 ; j <segments.size(); j++){
      float distance = 99999999;
      if( abs(segments[i][4]  - segments[j][4]) < .02 &&   abs(segments[i][5]  - segments[j][5]) < .02)
        {

          float ndist =  norm(Mat(Point(segments[i][0] , segments[i][1])),Mat(Point(segments[j][0] , segments[j][1])));
          distance = ndist < distance ? ndist : distance;
          ndist =  norm(Mat(Point(segments[i][0] , segments[i][1])),Mat(Point(segments[j][2] , segments[j][3])));
          distance = ndist < distance ? ndist : distance;
          ndist =  norm(Mat(Point(segments[i][2] , segments[i][3])),Mat(Point(segments[j][0] , segments[j][1])));
          distance = ndist < distance ? ndist : distance;
          ndist =  norm(Mat(Point(segments[i][2] , segments[i][3])),Mat(Point(segments[j][2] , segments[j][3])));
          distance = ndist < distance ? ndist : distance;
          if(distance > tunnelGap){
            cout<<"Tunnel detected";
            if( segments[j][5] < (PI/2)){
              pRight= true;
              cout<<" on the right" <<endl;
            }
            else
              pLeft = true;
            cout<<" on the left" <<endl;
          }
        }
    }
}

float slope( Point p1 , Point p2){
  return   atan((p2.y - p1.y)/(p2.x - p1.x)) * 180 / PI;
}

void CamController::processVideo(Mat image , string type , bool& pLeft , bool& pRight){

  int houghVote = 60;
  int cannyLower = 50;
  int cannyHigher = 250;
  Mat gray;
  removeSymbols(image);
  cvtColor(image,gray,CV_RGB2GRAY);
  Rect roi(0,image.rows/3,image.cols-1,image.rows - image.rows/3);// set the ROI for the image
  Mat imgROI = gray(roi);

  imshow("Original Image",imgROI );
  
  // Canny algorithm
  Mat contours;
  Canny(imgROI,contours,cannyLower,cannyHigher);
  Mat contoursInv;
  threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
  
  imshow("Canny",contoursInv);
  
  int seg1, seg2;
  bool foundLane  = false;
  std::vector<Vec2f> lines;
  Mat result(imgROI.size(),CV_8U,Scalar(255));
  imgROI.copyTo(result);
  Mat hough(imgROI.size(),CV_8U,Scalar(0));
  vector<Vec6f> segments;
  bool noLane  = false;
  lines.clear();
  segments.clear();
  for(houghVote = 60 ; houghVote>=30; houghVote-=5){
    
    HoughLines(contours,lines,1,PI/180, houghVote);

    // Draw the limes
    std::vector<Vec2f>::const_iterator it= lines.begin();
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
      
    segments  =getLineSegments( contoursInv, lines) ;
    drawLineSegments(contoursInv ,segments, Scalar(0));
      
    if(type == "TUNNEL"){
      detectTunnel(segments , pLeft , pRight);
      return;
    }
    
    for ( int i =0 ; i< segments.size(); i++)
      for( int j = i+1 ; j <segments.size(); j++){
        cout<< segments[i][5] << " " << segments[j][5] << " sum is " << (segments[i][5] + segments[j][5]) * 180 /PI<<endl;
        if( abs ((segments[i][5] + segments[j][5])* 180 /PI - 180 ) < LANE_ANGLE_THRESH)
          {
            seg1 = i;
            seg2 = j;
            foundLane = true;
          }
      }

    if(foundLane || noLane)
      break;
    else{
      if(houghVote == 30){
        noLane = true;
        houghVote = 65;
      }
    }
  }
  cout<<"Using hough vote " << houghVote<<endl;
  imshow("Personal algo", contoursInv);

  //seg1 is to the left of seg2
  if(segments.size() > 0 && foundLane){
    if( segments[seg1][5] > segments[seg2][5])
      {
        swap( seg1 , seg2);
      }

    if( (180- segments[seg1][5] * 180/PI) - segments[seg2][5] * 180/PI < -20 ){
      pLeft = true;
      pRight = false;
      cout<< "LEFT"<<endl;
    }
    else if( (180- segments[seg1][5] * 180/PI) - segments[seg2][5] * 180/PI >  20 ){
      pRight = true;
      pLeft = false;
      cout<<"RIGHT"<<endl;
    }
    else{
      pLeft = pRight = false;
      cout<<"STRAIGHT"<<endl;
    }
  }
  else if (segments.size() >0 ){
    //looking at single lines figure out
    double max_length = 0;
    double max_l_angle;
    Point midpoint;
    for ( int i =0 ; i< segments.size(); i++)
      {
        double length =  cv::norm(cv::Mat(Point(segments[i][0] , segments[i][1])),cv::Mat(Point(segments[i][2] , segments[i][3])));
        if(length > max_length){
          max_length = length;
          max_l_angle = segments[i][5];
          midpoint = Point( (segments[i][0] + segments[i][2]) /2 ,  (segments[i][1] + segments[i][3]) /2 );
        }
      }
    if(midpoint.x > contours.cols/2){
      pLeft = true;
      pRight = false;
      cout<< "LEFT"<<endl;
    }
    else{
      pRight = true;
      pLeft = false;
      cout<<"RIGHT"<<endl;
    }
  }
}

void CamController::isPassage(Mat frame , bool& pLeft,bool& pRight){

  pLeft = pRight = false;
  processVideo(frame,"TUNNEL" , pLeft , pRight );
  //TODO : Better dependency of nDir and pDir


}

string CamController::laneFollowDir(Mat frame){
  bool pLeft , pRight;
  pLeft = pRight = false;
  processVideo(frame,"LANE" , pLeft , pRight );
  if(pLeft)
    return "LEFT";
  else if(pRight)
    return "RIGHT";
  else
    return "STRAIGHT";
}

