#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include <queue>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"

#define Frontier vector<Point>

using namespace std;

struct Point{
int x;
int y;
};

struct Line{
vector<Point> points;
};


// vector<Frontier> frontiersDB, 
void FFD( Point pose,vector<Point> lr, const nav_msgs::OccupancyGrid& map, int map_height, int map_width);
vector<Point> Sort_Polar( vector<Point> lr, Point pose);
Line Get_Line( Point prev, Point curr );
