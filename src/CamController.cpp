#include "CamController.h"



#define TUNNEL_GAP 50
#define LANE_ANGLE_THRESH 30
#define LINE_RHO_THRESH 50
#define LINE_ANGLE_THRESH 10


using namespace std;
using namespace cv;

#define IMSHOW 1

float abs( float x , float y){
  if((x-y) < 0)
    return y-x;
  else
    return x-y;
}

vector<Vec6f> getLineSegments( Mat& edgeIm , vector<Vec2f> clines , vector<Vec2f>& hvlines){
  
  vector<Vec2f> lines;
  bool exists = false;

#ifdef DEBUG
  cout<<"Crho ctheta are of the detected segments are"<<endl;
#endif
  for(int i = 0; i < clines.size() ; i++){

    exists = false;
    float crho = clines[i][0], ctheta = clines[i][1];

#ifdef DEBUG
    cout <<crho << " " << ctheta * 180 / PI<<endl;
#endif

    //remove nearly horizontal or vertical lines
    if( ctheta * 180/PI < 10 || (ctheta * 180/PI < 110 && ctheta * 180/PI > 70) || (ctheta * 180/PI > 170) )
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

#ifdef DEBUG
  cout<<"Rho and theta of the detected lines are:"<<endl;
  #endif
  for( int j = 0 ; j < lines.size() ; j++){
    float rho = lines[j][0], theta = lines[j][1];
#ifdef DEBUG
    cout<<j<<  " Rho: " <<  rho << " , Theta: " << theta * 180 / PI << endl;
    #endif
  }
 
#ifdef DEBUG
  cout<<"There are total " <<  lines.size() << " lines"<<endl;
  #endif

  #ifdef IMSHOW
  imshow("Getting line segments on " , edgeIm);
  #endif

  int whitekernel = 1;
  //remove hor and vert lines
  for( size_t i = 0; i < hvlines.size(); i++ )
    {
      float rho = hvlines[i][0], theta = hvlines[i][1];
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b)); 
      pt1.y = cvRound(y0 + 1000*(a)); 
      pt2.x = cvRound(x0 - 1000*(-b)); 
      pt2.y = cvRound(y0 - 1000*(a)); 
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
	    else{
#ifdef DEBUG
	      cout<<"Error"<<endl;
#endif
	    }
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
#ifdef DEBUG
	    else cout<<"Error"<<endl;
#endif
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
#ifdef DEBUG
	    else cout<<"Error"<<endl;
#endif
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
#ifdef DEBUG
	    else cout<<"Error"<<endl;
#endif
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

#ifdef DEBUG
    cout<<"The segments are"<< endl;
#endif
    vector<Vec6f> finalsegments;
    for(int i =0 ; i < segments.size(); i++)
      {
	double length =  cv::norm(cv::Mat(Point(segments[i][0] , segments[i][1])),cv::Mat(Point(segments[i][2] , segments[i][3])));
	if(length > kernel*2)
	  {
	    finalsegments.push_back(segments[i]);
#ifdef DEBUG
	    cout<<" Rho: " << segments[i][4] << " Theta: " << segments[i][5] * 180 / PI;
#endif
	  }

      }

  
#ifdef DEBUG
    cout<<"There are total " <<finalsegments.size() << " segments"<<endl;
#endif


	return finalsegments;
      }



    void drawLineSegments(Mat& ime , vector<Vec6f> segments ,  cv::Scalar color=cv::Scalar(255)){

      // Draw the lines
      int i =0;
      std::vector<cv::Vec6f>::const_iterator it2= segments.begin();
      while (it2!=segments.end()) {



	cv::Point pt1((*it2)[0],(*it2)[1]);        
	cv::Point pt2((*it2)[2],(*it2)[3]);
	cv::line( ime, pt1, pt2, Scalar(i*50 % 255 ), 10 );
	++it2;
	++i;
      }
    }


    string CamController::getColorHSV( int h , int s , int v){

      string color = "default";
      if( s > 30 &&  v >50 && v< 220){
	if( h < 30){
	  color = "RED";
	}
	else if( h > 192 && h < 240){
	  color = "BLUE";
	}
	else if ( h>100 && h < 180){
	  color = "GREEN";
	}
      }
      else if ( s<  50 && v > 150){
	color = "WHITE";
      }
      else if ( v < 50){
	color = "BLACK";
      }
      //cout << "h = " << h << ", s = " << s << ", v = " << v << endl;
      return color;   

    }


    void removeSymbols(Mat& img , Mat edges , Mat& contoursInv) {
  
      Mat hsv;
      cvtColor(img ,hsv ,CV_BGR2HSV);

      std::vector<std::vector<cv::Point> > contours;
      //cv::Canny(img, edges, 50, 200, 3 );

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
	      string color = CamController::getColorHSV( hue , sat , val);
	      if(color == "BLUE" || color == "GREEN" || color == "RED")
		colored++;
	      else if( color == "BLACK")
		black++;
	      else if( color == "WHITE")
		white++;

	    }

	  if(colored > black + white){
	    //rectangle(contoursInv,  bRect, Scalar(255 , 255 ,255) ,-1 );
	    circle(contoursInv, Point(bRect.x + bRect.width/2, bRect.y + bRect.height/2), sqrt(bRect.height*bRect.height + bRect.width*bRect.width)/2 + 5, CV_RGB(255,255,255), -1);

	  }
      
	}
  
#ifdef IMSHOW
      imshow("Removed symbols from frames", img);
#endif
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
		  pLeft= true;
		  cout<<" on the left" <<endl;
		}
		else
		  pRight = true;
		cout<<" on the right" <<endl;
	      }
	    }
	}
    }

    float slope( Point p1 , Point p2){
      return   atan((p2.y - p1.y)/(p2.x - p1.x)) * 180 / PI;
    }

#define HOUGH_VOTE 80
    void CamController::processVideo(Mat image , string type , bool& pLeft , bool& pRight , bool & lane){

      pLeft = pRight = false;
      int houghVote = HOUGH_VOTE;
      int cannyLower = 100;
      int cannyHigher = 200;
      Mat gray;

      cv::Mat kernel = Mat::ones(Size(7, 7), CV_8U);  
      cv::erode(image,image,kernel);

      cv::dilate(image , image,kernel);
      
      cvtColor(image,gray,CV_RGB2GRAY);
      bool phorizontalLine = false;
      Mat edges_test;
      Mat contours;
      cv::blur( gray, gray, Size( 5, 5 ), Point(-1,-1) );
      Canny(image,contours,cannyLower,cannyHigher,3);
      Rect roi(0,image.rows/3,image.cols-1,image.rows - image.rows/3);// set the ROI for the image
      Mat imgROI =gray(roi);
      contours = contours(roi);
      cv::imshow("roi", imgROI);
      // Canny algorithm
      Mat contoursInv;
      threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
      removeSymbols(image , contours , contoursInv);
      
#ifdef IMSHOW
      imshow("Canny",contoursInv);
#endif
  
      int seg1, seg2;
      bool foundLane  = false;
      std::vector<Vec2f> lines;
      Mat result(imgROI.size(),CV_8U,Scalar(255));
      imgROI.copyTo(result);
      Mat hough(imgROI.size(),CV_8U,Scalar(0));
      vector<Vec6f> segments;
      bool noLane  = false;

      vector<Vec2f> hvlines;

      /*
    
	for(houghVote = HOUGH_VOTE ; houghVote>= HOUGH_VOTE - 30; houghVote-=10){

      */
      lines.clear();
      segments.clear();
      hvlines.clear();
      HoughLines(contours,lines,1,PI/180, houghVote);

      // Draw the limes
      std::vector<Vec2f>::const_iterator it= lines.begin();
#ifdef DEBUG
      cout<< "All lines detected by Hough"<<endl;
#endif
      while (it!=lines.end()) {


	float rho= (*it)[0];   // first element is distance rho
	float theta= (*it)[1]; // second element is angle theta
#ifdef DEBUG
	cout<<"Rho: " << rho << " Theta: "<<theta <<endl;
#endif
	Point pt1(rho/cos(theta),0);        
	// point of intersection of the line with last row
	Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
	// draw a white line
	line( result, pt1, pt2, Scalar(255), 1); 
	line( hough, pt1, pt2, Scalar(255), 1);
	++it;
      }
      // Display the detected line image
#ifdef IMSHOW
      imshow("Detected Lines with Hough",result);
#endif
      segments  =getLineSegments( contoursInv, lines , hvlines) ;
      drawLineSegments(contoursInv ,segments, Scalar(0));

      /*
	if(type == "TUNNEL"){
	detectTunnel(segments , pLeft , pRight);
	return;
	}
      */

      for ( int i =0 ; i< segments.size(); i++)
	for( int j = i+1 ; j <segments.size(); j++){
#ifdef DEBUG
	  cout<< segments[i][5] << " " << segments[j][5] << " sum is " << (segments[i][5] + segments[j][5]) * 180 /PI<<endl;
#endif
	  if( abs ((segments[i][5] + segments[j][5])* 180 /PI - 180 ) < LANE_ANGLE_THRESH)
	    {
	      seg1 = i;
	      seg2 = j;
	      foundLane = true;
	    }
	}

      /*
	if(foundLane || (noLane && houghVote ==HOUGH_VOTE))
	break;
	else{
	if(houghVote == HOUGH_VOTE - 30){
	noLane = true;
	houghVote = HOUGH_VOTE+ 10;
	}
	}
	}

      */

#ifdef IMSHOW
      imshow("Personal algo", contoursInv);
#endif

      	    float max_rho;
	    //Find line with maximum black
	    float max_theta;

      //seg1 is to the left of seg2
      if(segments.size() > 0 && foundLane){
	if( segments[seg1][5] > segments[seg2][5])
	  {
	    swap( seg1 , seg2);
	  }
    
	if( segments[seg1][5] * 180/PI +segments[seg2][5] * 180/PI > 195 ){
	  pLeft = true;
	  pRight = false;

	  cout<< "\t\t\t\tLEFT\t\t\t\tFOUNDLANE"<<endl;

	}
	else if( segments[seg1][5] * 180/PI + segments[seg2][5] * 180/PI  < 165 ){
	  pRight = true;
	  pLeft = false;
	  cout<<"\t\t\t\tRIGHT\t\t\t\tFOUNDLANE"<<endl;
	}
	else{
	  pLeft = pRight = false;
	  cout<<"\t\t\t\tSTRAIGHT\t\t\t\tFOUNDLANE"<<endl;
	}
      }
      else if (segments.size() >0 ){
	//looking at single lines figure out
	double max_length = 0;
	double max_l_angle;
	Vec6f longest_segment;
	for ( int i =0 ; i< segments.size(); i++)
	  {
	    double length =  cv::norm(cv::Mat(Point(segments[i][0] , segments[i][1])),cv::Mat(Point(segments[i][2] , segments[i][3])));
	    if(length > max_length){
	      max_length = length;
	      max_l_angle = segments[i][5];
	      longest_segment = segments[i];
	    }
	  }
	int y;
	y =max( longest_segment[1] ,  longest_segment[3]);
	cout<<"###The y max is "<< y <<endl;
	cout <<"---The imgROI.rows is " << imgROI.rows << endl;
	if ( y <imgROI.rows - 75)
	  {

	    pLeft = pRight = false;
	    cout<<"\t\t\t\tSTRAIGHT\t\t\t\tSINGLE LINE"<<endl;
	  }
	else {
	  if(max_l_angle >PI/2){

      
	    pLeft = true;
	    pRight = false;
	    cout<< "\t\t\t\tLEFT\t\t\t\tSINGLE LINE"<<endl;
	  }
	  else{

	    pRight = true;
	    pLeft = false;
	    cout<<"\t\t\t\tRIGHT\t\t\t\tSINGLE LINE"<<endl;
	  }
	}
      }
      else if (hvlines.size()!=0)
	{
	  bool toomanylines = false;
	  if(hvlines.size()> 100)
	    toomanylines = true;
	    
	  
	  


	  int max_black = 0;
	  bool foundHor = false;
	  for( int k =0 ; k< hvlines.size(); k++ ){
	    if(toomanylines)
	      break;
	    
	    float rho= (hvlines[k])[0];   // first element is distance rho
	    float theta= (hvlines[k])[1]; // second element is angle theta

	    //cout<<"Rho : " << rho <<" Theta: "<< theta<<endl;
	    if(theta* 180 / PI < 70 || theta* 180 / PI > 100)
	      continue;
	    foundHor = true;

	    Point pt1(rho/cos(theta),0);        
	    // point of intersection of the line with last row
	    Point pt2((rho-contoursInv.rows*sin(theta))/cos(theta),contoursInv.rows);

	    int kernel = 5;
	    LineIterator lit( contoursInv , pt1 , pt2 );
	    Point curr_pos;


	    for(int j = 0; j < lit.count; j++, ++lit)
	      {
		curr_pos = lit.pos();
		bool isBlack = false;
		int curr_black  = 0;
		for (int xi = curr_pos.x  - kernel ; xi <= curr_pos.x + kernel ; xi++)
		  {
		    for (int yi = curr_pos.y  - kernel ; yi <= curr_pos.y + kernel ; yi++)
		      {
			if( xi < 0 || xi >= contoursInv.cols || yi < 0 || yi >= contoursInv.rows )
			  continue;
			Scalar intensity = contoursInv.at<uchar>(yi , xi);
			if( intensity.val[0] == 0){
			  curr_black++;
			}
		      }
		  }
		if( curr_black> max_black){
		  max_theta = hvlines[k][1];
		  max_rho = hvlines[k][0];
		   max_black = curr_black;
		}
	      }
	  }

	  cout<<"Max black for the current line is "<<max_black <<endl;
	  if(max_black < 10)
	    {
	      pLeft = false;
	      pRight = false;
	      cout<<"\t\t\t\tUNKOWN HORIZONTAL "<<endl;
	      foundHor = false;
	    }

	  if(foundHor){
	    cout<<"\t\tUSING HORIZONTAL"<<endl;
	    cout<<"Horizontal line is " << max_theta* 180 / PI << " degrees" <<endl;
    
	    if(max_theta* 180 / PI > 95 && max_theta* 180 / PI < 110)
	      {
		pRight = true;
		pLeft = false;
		cout<<"\t\t\t\tRIGHT\t\t\t\tHORIZONTAL "<<endl;

	      }
	    else if (max_theta* 180 / PI < 85 &&  max_theta* 180 / PI > 70 ){
	      pLeft = true;
	      pRight = false;
	      cout<< "\t\t\t\tLEFT\t\t\t\tHORIZONTAL LINE"<<endl;

	    }
	    else
	      {
		pLeft = false;
		pRight = false;
		cout<< "\t\t\t\tSTRAIGHT\t\t\t\tHORIZONTAL LINE"<<endl;
	      }

	    phorizontalLine  =true;
	  }
	}
      else{
	pLeft = false;
	pRight = false;
	cout<<"\t\t\t\tUNKOWN"<<endl;
      }
  
      lane = foundLane || phorizontalLine;
      if(phorizontalLine)
	{
	  float rho = max_rho;
	  float theta = max_theta;
	  	Point pt1(rho/cos(theta),0);        
	// point of intersection of the line with last row
	Point pt2((rho-gray.rows*sin(theta))/cos(theta),gray.rows);
	// draw a white line
	line(contoursInv, pt1, pt2, Scalar(100), 3); 

	imshow("horizontal line" , contoursInv);
	}
    }

    void CamController::isPassage(Mat frame , bool& pLeft,bool& pRight){

      pLeft = pRight = false;
      bool blank;
      processVideo(frame,"TUNNEL" , pLeft , pRight , blank );
      //TODO : Better dependency of nDir and pDir


    }

    string CamController::laneFollowDir(Mat frame){
      bool pLeft , pRight;
      pLeft = pRight = false;
      bool blank;
      processVideo(frame,"LANE" , pLeft , pRight , blank );
      if(pLeft)
	return "LEFT";
      else if(pRight)
	return "RIGHT";
      else
	return "STRAIGHT";
    }

