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
        img_route = inp_map.img_src.clone();
        adjMat = initAdjMat();

        assignIndices();
        drawIndices();
        fillAdjMat();
}

MapProcessor::~MapProcessor()
{
        for(int i=0;i<size;++i)
                delete adjMat[i];
        delete adjMat;

        for(int i=0;i<route.size();++i)
                {
                        if(route[i]!=NULL)
                                delete route[i];
                }
}

void MapProcessor::findRoute()
{
        Waypoint *wp = new Landmark(landmarks[0]);
        Landmark *l;
        route.push_back(wp);
  
        int *used = new int[size];
        int i,j,idx;
        int nTJ = 0; //no. of T-Junctions found

        for(i=0;i<size;++i)
                used[i] = 0;
        used[0] = 1;
  
        // guaranteed that "route.back()" will be a landmark
        // since a T-Junction will be followed by a landmark
        while(1)
                {
                        wp = route.back();
                        l = dynamic_cast<Landmark*>(wp);
      
                        idx = l->idx;
      
                        if(l->color == "BROWN")
                                break;

                        for(i=0;i<size;++i)
                                {
                                        //this is the  next landmark in the route
                                        if(adjMat[idx][i]==1 && used[i]==0)
                                                {
                                                        wp = new Landmark(landmarks[i]);
                                                        route.push_back(wp);
                                                        used[i] = 1;
                                                        break;
                                                }
                                }
      
                        //next in sequence already found above, so go to next iteration
                        if(i<size)
                                continue;
      
                        //next in sequence is a T-Junction
                        //check all unused landmarks to see which can form a T-Junction with idx
                        for(i=0;i<size;++i)
                                {
                                        if(used[i] == 1)
                                                continue;
	  
                                        cv::Point2f p_idx(landmarks[idx].centroid);
                                        cv::Point2f p_i(landmarks[i].centroid);
	  
                                        cv::Point2f p1(p_idx.x,p_i.y);
                                        cv::Point2f p2(p_i.x,p_idx.y);
	  
                                        if(isConnected(p_idx,p1) && isConnected(p1,p_i))
                                                {
                                                        wp = new TJunction(p1,size + nTJ);
                                                        ++nTJ;
                                                        route.push_back(wp);
	      
                                                        wp = new Landmark(landmarks[i]);
                                                        used[i] = 1;
                                                        route.push_back(wp);

                                                        break;
                                                }
                                        if(isConnected(p_idx,p2) && isConnected(p2,p_i))
                                                {
                                                        wp = new TJunction(p2,size + nTJ);
                                                        ++nTJ;
                                                        route.push_back(wp);
	      
                                                        wp = new Landmark(landmarks[i]);
                                                        used[i] = 1;
                                                        route.push_back(wp);

                                                        break;
                                                }
                                }
                }
        delete used;
}

void MapProcessor::displayRoute()
{
        findRoute();
        drawRoute();
        cv::imshow("route",img_route);
}

void MapProcessor::drawRoute()
{
        int i,j;
        for(i=0;i<route.size()-1;++i)
                cv::line(img_route,route[i]->centroid,route[i+1]->centroid,CV_RGB(150,50,150),2);
}

string MapProcessor::getOrient(Waypoint *w1, Waypoint *w2)
{
        if(!w1 || !w2)
                return "None";
        string orientation;
        cv::Point2f p1(w1->centroid);
        cv::Point2f p2(w2->centroid);
  
        if(p2.y - p1.y > EPSILON)
                orientation = "SOUTH";
        else if(p1.y - p2.y > EPSILON)
                orientation = "NORTH";
        else if(p2.x - p1.x > EPSILON)
                orientation = "EAST";
        else if(p1.x - p2.x > EPSILON)
                orientation = "WEST";
        else
                orientation = "?";
        return orientation;
}

string MapProcessor::getDir(string orientation_curr, string orientation_next)
{
        string direction;
        if(orientation_curr == "NORTH")
                {
                        if(orientation_next == "NORTH")
                                direction = "STRAIGHT";
                        else if(orientation_next == "SOUTH")
                                direction = "U-TURN";
                        else if(orientation_next == "EAST")
                                direction = "RIGHT";
                        else if(orientation_next == "WEST")
                                direction = "LEFT";
                        else
                                direction =  "?";
                }
        else if(orientation_curr == "SOUTH")
                {
                        if(orientation_next == "NORTH")
                                direction = "U-TURN";
                        else if(orientation_next == "SOUTH")
                                direction = "STRAIGHT";
                        else if(orientation_next == "EAST")
                                direction = "LEFT";
                        else if(orientation_next == "WEST")
                                direction = "RIGHT";
                        else
                                direction = "?";
                }
        else if(orientation_curr == "EAST")
                {
                        if(orientation_next == "NORTH")
                                direction = "LEFT";
                        else if(orientation_next == "SOUTH")
                                direction = "RIGHT";
                        else if(orientation_next == "EAST")
                                direction = "STRAIGHT";
                        else if(orientation_next == "WEST")
                                direction = "U-TURN";
                        else
                                direction = "?";
                }
        else if(orientation_curr == "WEST")
                {
                        if(orientation_next == "NORTH")
                                direction = "RIGHT";
                        else if(orientation_next == "SOUTH")
                                direction = "LEFT";
                        else if(orientation_next == "EAST")
                                direction = "U-TURN";
                        else if(orientation_next == "WEST")
                                direction = "STRAIGHT";
                        else
                                direction = "?";
                }
        else
                direction = "?";
  
        return direction;
}

void MapProcessor::printRoute()
{
        Waypoint *wp,*wp1;
        Landmark *l;
        TJunction *tj;
        int i;
  
        string orientation_curr;
        string orientation_next;
        string direction;
  
        cout << "ROUTE :" << endl << endl;
        cout << "1. STRAIGHT from [START]" << endl << endl;
        for(i=1;i<route.size()-1;++i)
                {
                        wp = route[i];
                        l = dynamic_cast<Landmark*>(wp);
                        if(l!=NULL)
                                {
                                        orientation_curr = getOrient(route[i-1],route[i]);
                                        orientation_next = getOrient(route[i],route[i+1]);
                                        direction = getDir(orientation_curr,orientation_next);
	  
                                        cout << i+1 << ". " << direction << " at Landmark :" << endl;
                                        cout << *l << endl;
                                }
                        else
                                {
                                        orientation_curr = getOrient(route[i-1],route[i]);
                                        orientation_next = getOrient(route[i],route[i+1]);
                                        direction = getDir(orientation_curr,orientation_next);

                                        tj = dynamic_cast<TJunction*>(wp);
                                        cout << i+1 << ". " << direction << " at T-Junction :" << endl;
                                        cout << *tj << endl;
                                }
                }
        cout << i+1 << ". Reached [END]" << endl;
}

void MapProcessor::displayConnections()
{
        drawConnections();
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

bool MapProcessor::isConnected(cv::Point2f p1, cv::Point2f p2)
{
        LineIterator it(m.img_arena, p1, p2, 8);
        for(int i = 0; i < it.count; ++i, ++it)
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
                                        if(isConnected(landmarks[i].centroid, landmarks[j].centroid))
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
        for(i=0;i<landmarks.size();++i)
                adjMat[i][i] = 0;
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



void MapProcessor::fillAllAdjMat (){
        Landmark l;
        for(int i =0; i< landmarks.size() ; i++){

                //Copy initial adjmatric
                for(int j =0 ; j<landmarks.size() ; j++)
                        {
                                cadjMat[i][j] = adjMat[i][j];
                        }

                //add tjunctions to adjmatrix
                for(int j =0; j< m.TJs.size() ; j++)
                        {

                                if(isConnected(m.TJs[j].centroid , landmarks[i].centroid)){
                                        cadjMat[i][landmarks.size() + j] = 1;
                                        cadjMat[landmarks.size() + 1][i] = 1;
                                }
                        }
                //add landmarks to clandmarks
                clandmarks.push_back( landmarks[i]);
        }


        for(int i =0 ; i< m.TJs.size(); i++){
                //addJunctions to clandmarks
                l = m.TJs[i];
                l.idx = landmarks.size() + i;
                clandmarks.push_back( l);
        }
        
}
                



void MapProcessor::shortestPath(){

        //stores all hexagons, start and points
        std::deque<int> hexagon_index;
        //stores all the start landmarks
        std::vector<int> start_index;
        //index of the last landmark
        int  end_index = -1;

        //storage for floyd warshall
        std::vector<std::vector<int> > distances(clandmarks.size(), vector<int>(clandmarks.size(),999999999));
        std::vector<std::vector<int> > next(clandmarks.size(), vector<int>(clandmarks.size(),999999999));

        //vectors of landmark paths corresponding to each start_index element
        std::vector< std::vector<Landmark> > paths;

        //populating the hexagon_index vector
        for (int i =0; i < clandmarks.size() ; i++)
                {
                        if( clandmarks[i].shape == "HEXAGON" )
                                hexagon_index.push_back( i);
                        else if( clandmarks[i].start )
                                start_index.push_back(i);
                        else if (clandmarks[i].end){
                                if(end_index != -1)
                                        cout<< "Error: Multiple end landmarks" <<endl;
                                end_index = i;

                        }
                }
        hexagon_index.push_back(end_index);
        

        //run floyd warshal
        allPairsShortestDistance( clandmarks , distances , next); 

        //ranks to permute through hexagon indexes
        std::vector<int> ranks;
        for(int i=0; i < hexagon_index.size(); i++){
                ranks.push_back(i);
        }

        //helpers to find out the perfect path
        std::vector<int> min_distance_ranks;
        double min_distance , curr_distance;


        //Find the correct path for each different start_index
        for( int i = 0 ; i< start_index.size() ; i++)
                {
                        //add start_element
                        hexagon_index.push_front(start_index[i]);
                        //TODO: nmake this double_max ideally
                        min_distance = 999999999;
                        do {
                                curr_distance = 0;
                                
                                for(int j = 0 ; j < ranks.size()-1 ; j++)
                                        {
                                                curr_distance += distances[hexagon_index[ranks[j]]][hexagon_index[ranks[j+1]]];
                                        }
                                if( curr_distance < min_distance){
                                        min_distance = curr_distance;
                                        min_distance_ranks = ranks;
                                }
                        } while ( std::next_permutation(ranks.begin() +1 ,ranks.end() -1)); //the first one is start and last is end. so not permuting them

                        std::vector<Landmark> temp_path;
                        paths[i].push_back( clandmarks[hexagon_index[0]]);
                        for(int j=0; j< hexagon_index.size() -1 ; j++)
                                {
                                        
                                        //getPath only returns the intermediate path
                                        temp_path =  getPath(hexagon_index[min_distance_ranks[j]] , hexagon_index[ min_distance_ranks[j+1]] ,distances , next );
                                        paths[i].insert( paths[i].end() , temp_path.begin() , temp_path.end());
                                        paths[i].push_back( clandmarks[hexagon_index[ min_distance_ranks[j+1]]] );
                                }

                        //remove the start element
                        hexagon_index.pop_front();
                }
}


void MapProcessor::allPairsShortestDistance( std::vector<Landmark> vertices , std::vector<std::vector<int> > & distances , std::vector<std::vector<int> > & next ){

        // initialize diagonal
        for(int i=0; i < vertices.size(); i++)
                distances[i][i]=0;
			

        for( int i =0 ; i < vertices.size() ; i++ ){
                for( int j =0 ; j < vertices.size() ; j++ ){
                        distances[i][j]  = distance( vertices[i].centroid , vertices[j].centroid);
                }
        }


        // Floyd-Warshall
        // Add nodes between (first 1 then 2, 3 till n) and look if
        // distance is shorter
        for(int k = 0; k < vertices.size(); k++)
                for(int i = 0; i < vertices.size(); i++)
                        for(int j = 0; j < vertices.size(); j++)
                                if(distances[i][j]>distances[i][k]+distances[k][j]){
                                        distances[i][j]=distances[i][k]+distances[k][j];
                                        next[i][j]= k;
                                }
                                 
	
        // Print out final distance matrix
        for(int i = 0; i < vertices.size(); i++){
                for(int j = 0; j < vertices.size(); j++)
                        cout << distances[i][j] << " ";
                cout << endl;
        }

}

std::vector<Landmark> MapProcessor::getPath(int i, int  j , std::vector<std::vector<int> >& distances,  std::vector<std::vector<int> >& next ){

        //the path returned only contains the intermediate points to be visited
        std::vector<Landmark> pathi , pathj , path;

        // TODO: this should be made something that says its not possible to reach
        if (distances[i][j] == 9999999)
                return path;
        int intermediate = next[i][j];
        if (intermediate ==  -1)
                return path;
        else {
                pathi = getPath(i, intermediate , distances , next);
                pathj = getPath(intermediate, j , distances , next);

                //concatenate the paths
                path.reserve(pathi.size() + pathj.size() + 1);
                path.insert( path.end(), pathi.begin(), pathi.end() );
                path.push_back(clandmarks[intermediate]);
                path.insert( path.end(), pathj.begin(), pathj.end() );
                return path;
        }
        
}


double MapProcessor::distance( cv::Point2f a , cv::Point2f b){
        return cv::norm(cv::Mat(a),cv::Mat(b));
}



