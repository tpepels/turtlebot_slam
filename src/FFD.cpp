#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <algorithm>
#include "FFD.hpp"
#include "nav_msgs/OccupancyGrid.h"


using namespace std;

//#define Frontier vector<MyPoint>
vector<Frontier> frontiersDB;





void get_neighbours2(int n_array[], int position, int map_width)
{
    n_array[0] = position - map_width - 1;
    n_array[1] = position - map_width;
    n_array[2] = position - map_width + 1;
    n_array[3] = position - 1;
    n_array[4] = position + 1;
    n_array[5] = position + map_width - 1;
    n_array[6] = position + map_width;
    n_array[7] = position + map_width + 1;
}

bool is_frontier_point2(const nav_msgs::OccupancyGrid& map, int point, int map_size, int map_width)
{
    // The point under consideration must be known
    if(map.data[point] != -1)
    {
        return false;
    }
    //
    int locations[8];
    get_neighbours2(locations, point, map_width);
    for(int i = 0; i < 8; i++)
    {
        if(locations[i] < map_size && locations[i] >= 0)
        {
            //At least one of the neighbours is open and known space, hence frontier point //AANPASSING
            //if(map.data[locations[i]] < OCC_THRESHOLD && map.data[locations[i]] >= 0) {
            if(map.data[locations[i]] == -1)
            {
                return true;
            }
        }
    }
    return false;
}


float CrossProduct(float x0,float y0,float x1,float y1,float x2,float y2)
{
    return (x1 - x0)*(y2 - y0) - (x2 - x0)*(y1 - y0);
}

vector<MyPoint> Sort_Polar( vector<MyPoint> lr, MyPoint pose )
{
//simple bubblesort
    bool swapped = 0;
    do
    {
        swapped = 0;
        for(unsigned int i=1; i<lr.size(); i++)
        {
            if( CrossProduct(pose.x,pose.y,lr[i-1].x,lr[i-2].y,lr[i].x,lr[i].y) >= 0 )  //sorting clockwise
            {
                MyPoint swap = lr[i-1];
                lr[i-1] = lr[i];
                lr[i] = swap;
                swapped = 1;
            }
        }
    }
    while(swapped);

    return lr;
}


//Bresenham's line algorithm
Line Get_Line( MyPoint prev, MyPoint curr )
{
    Line output;

    int x0=prev.x, y0=prev.y, x1=curr.x, y1=curr.y;

    int dx=x1-x0;
    int dy=y1-y0;

    float D = 2*dy - dx;

    MyPoint newPoint;
    newPoint.x = x0;
    newPoint.y = y0;
    output.points.push_back(newPoint);

    int y=y0;

    for( int x=x0+1; x<=x1; x++)
    {
        if (D > 0)
        {
            y = y+1;
            MyPoint newPoint;
            newPoint.x = x;
            newPoint.y = y;
            output.points.push_back(newPoint);
            D = D + (2*dy-2*dx);
        }
        else
        {
            MyPoint newPoint;
            newPoint.x = x;
            newPoint.y = y;
            output.points.push_back(newPoint);
            D = D + (2*dy);
        }
    }


    return output;
}

vector<vector<int> > FFD( MyPoint pose,vector<MyPoint> lr, const nav_msgs::OccupancyGrid& map, int map_height, int map_width)
{
    int map_size = map_height * map_width;

// polar sort readings according to robot position
    vector<MyPoint> sorted = Sort_Polar(lr,pose);
// get the contour from laser readings
    MyPoint prev = sorted.back();
    sorted.pop_back();
    vector<MyPoint> contour;

    for(unsigned int i=0; i<sorted.size(); i++)
    {
        Line line = Get_Line(prev, sorted[i]);
        for(unsigned int j=0; j<line.points.size(); j++)
        {
            contour.push_back(line.points[j]);
        }
    }

// extract new frontiers from contour
    vector<Frontier> NewFrontiers;
    prev = contour.back(); //Point prev
    contour.pop_back();


    if ( is_frontier_point2(map, (prev.x + prev.y * map_width)  ,map_size,map_width) )
    {
        Frontier newFrontier;
        NewFrontiers.push_back(newFrontier);
    }
    for(unsigned int i=0; i<contour.size(); i++)
    {
        MyPoint curr = contour[i];
        if( !is_frontier_point2(map, (curr.x + curr.y * map_width)  ,map_size,map_width) )
        {
            prev = curr;
        }
        else if ( !(map.data[(curr.x + curr.y * map_width)] != -1) )    //curr is already visited
        {
            prev = curr;
        }
        else if ( is_frontier_point2(map, (curr.x + curr.y * map_width)  ,map_size,map_width)
                  &&  is_frontier_point2(map, (prev.x + prev.y * map_width)  ,map_size,map_width) )
        {
            NewFrontiers[NewFrontiers.size()-1].push_back(curr);
            prev = curr;
        }
        else
        {
            Frontier newFrontier;
            newFrontier.push_back(curr);
            NewFrontiers.push_back(newFrontier);
            prev = curr;
        }
    }


// maintainance of previously detected frontiers
//Get active area
    int x_min=-90000,x_max=90000,y_min=-90000,y_max=90000;
    for(unsigned int i=0; i<lr.size(); i++)
    {
        x_max = max(x_max,lr[i].x);
        y_max = max(y_max,lr[i].y);
        x_min = min(x_min,lr[i].x);
        y_min = min(y_min,lr[i].y);
    }

    for(int x=x_min; x<=x_max; x++)
    {
        for(int y=y_min; y<=y_max; y++)
        {
            MyPoint p;
            p.x = x;
            p.y = y;
            if( is_frontier_point2(map, (p.x + p.y * map_width)  ,map_size,map_width) )
            {
                // split the current frontier into two partial frontiers
                int Enables_f = -1;
                int Enables_p = -1;
                for(unsigned int i=0; i<frontiersDB.size(); i++)
                {
                    for(unsigned int j=0; j<frontiersDB[i].size(); j++)
                    {
                        if(  frontiersDB[i][j].x == p.x && frontiersDB[i][j].y == p.y )
                        {
                            Enables_f = i;
                            Enables_p = j;
                        }
                    }//for j
                }//for i

                if(Enables_f == -1 || Enables_p == -1)
                    continue;

                Frontier f1;
                Frontier f2;
                for(int i=0; i<=Enables_p; i++)
                {
                    f1.push_back( frontiersDB[Enables_f][i] );
                }
                for(unsigned int i=Enables_p+1; i<frontiersDB[Enables_f].size(); i++)
                {
                    f2.push_back( frontiersDB[Enables_f][i] );
                }
                frontiersDB.erase(frontiersDB.begin() + Enables_f);
            }//if p is a frontier

        } //for y
    }//for x

//Storing new detected frontiers
    int ActiveArea[ map.info.width][map.info.height];
    for(unsigned int i=0; i<frontiersDB.size(); i++)
    {
        for(unsigned int j=0; j<frontiersDB[i].size(); j++)
        {
            ActiveArea[frontiersDB[i][j].x][frontiersDB[i][j].y] = i;
        }
    }

    for(unsigned int i=0; i<NewFrontiers.size(); i++)
    {
        Frontier f = NewFrontiers[i];
        bool overlap = 0;
        for(unsigned int j=0; j<f.size(); j++)
        {
            if( ActiveArea[f[j].x][f[j].y] != 0 ) //overlap
            {
                int exists = ActiveArea[f[j].x][f[j].y];
                //merge f and exists
                for(unsigned int merged=0; merged<f.size(); merged++)
                {
                    frontiersDB[exists].push_back(f[merged]);
                }
                NewFrontiers[i].clear();
                overlap = 1;
                break;
            }
        }//for j
        if(overlap == 0)
        {
            frontiersDB.push_back(f);
        }
    }//for i

//remove empty frontier
    for(unsigned int i=0; i<frontiersDB.size(); i++)
    {
        if( frontiersDB[i].size() == 0 )
        {
            frontiersDB.erase(frontiersDB.begin() + i);
        }
    }


vector<vector<int> > Result;
//convert frontierDB to frontiers
for(unsigned int i=0; i<frontiersDB.size(); i++){
  vector<int> NewFrontiers;
  vector<MyPoint> ThisFrontier = frontiersDB[i];
  for(unsigned int j=0; j<ThisFrontier.size(); j++){
    NewFrontiers.push_back(   ThisFrontier[j].x + (ThisFrontier[j].y * map.info.width) );
  }
  Result.push_back(NewFrontiers);
}


vector<int> NewFrontiers2;
for(int x=0; x<5; x++){
  for(int y=0; y<5; y++){
  NewFrontiers2.push_back( x + (y * map.info.width) );
 }
}

Result.push_back(NewFrontiers2);

return Result;

}//end FFD

