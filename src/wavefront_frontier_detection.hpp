#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include <queue>
#include "nav_msgs/OccupancyGrid.h"

void get_neighbours(int n_array[], int position, int map_width);
nav_msgs::OccupancyGrid downSizeMap(const nav_msgs::OccupancyGrid& map, int width, int height);
bool is_frontier_point(const nav_msgs::OccupancyGrid& map, int point, int map_size, int map_width);
int get_row_from_offset(int offset, int width);
int get_column_from_offset(int offset, int width);
std::vector<std::vector<int> > wfd(const nav_msgs::OccupancyGrid& map, int map_height, int map_width, int pose);
