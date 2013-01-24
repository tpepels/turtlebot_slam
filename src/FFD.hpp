#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include <queue>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"

#define Frontier std::vector<MyPoint>

//using namespace std;

struct MyPoint{
int x;
int y;
};

struct Line{
std::vector<MyPoint> points;
};


// vector<Frontier> frontiersDB, 
std::vector<std::vector<int> > FFD( MyPoint pose, std::vector<MyPoint> lr, const nav_msgs::OccupancyGrid& map, int map_height, int map_width);
std::vector<MyPoint> Sort_Polar( std::vector<MyPoint> lr, MyPoint pose);
Line Get_Line( MyPoint prev, MyPoint curr );
