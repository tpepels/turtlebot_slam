#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include <queue>

using namespace std;

void get_neighbours(int n_array[], int position, int map_width);
bool is_frontier_point(int occ_grid[], int point, int map_size, int map_width);
int get_row_from_offset(int offset, int width);
int get_column_from_offset(int offset, int width);
vector<vector<int> > wfd(int occ_grid[], int map_height, int map_width, int pose);

const int MAP_OPEN_LIST = 1, MAP_CLOSE_LIST = 2, FRONTIER_OPEN_LIST = 3, FRONTIER_CLOSE_LIST = 4;
const int OCC_THRESHOLD = 10;

vector<vector<int> > wfd(int occ_grid[], int map_height, int map_width, int pose) {	
	vector<vector<int> > frontiers;
	// Cell state list for map/frontier open/closed
	int map_size = map_height * map_width;
	int cell_states[map_size];
	memset(cell_states, 0, map_size);	
	//
	queue<int> q_m;	
	q_m.push(pose);
	cell_states[pose] = MAP_OPEN_LIST;
	int adj_vector[8];
	int v_neighbours[8];
	//
	while(!q_m.empty()) {
		int cur_pos = q_m.front();
		q_m.pop();
		// Skip if map_close_list
		if(cell_states[cur_pos] == MAP_CLOSE_LIST)
			continue;
		if(is_frontier_point(occ_grid, cur_pos, map_size, map_width)) {
			queue<int> q_f;
			vector<int> new_frontier;
			q_f.push(cur_pos);
			cell_states[cur_pos] = FRONTIER_OPEN_LIST;
			// Second BFS
			while(!q_f.empty()) {
				int n_cell = q_f.front();
				q_f.pop();
				//
				if(cell_states[n_cell] == MAP_CLOSE_LIST || cell_states[n_cell] == FRONTIER_CLOSE_LIST)
					continue;
				//
				if(is_frontier_point(occ_grid, n_cell, map_size, map_width)) {
					new_frontier.push_back(n_cell);
					get_neighbours(adj_vector, cur_pos, map_width);			
					//
					for(int i = 0; i < 8; i++) {
						if(adj_vector[i] < map_size && adj_vector[i] >= 0) {
							if(adj_vector[i] != FRONTIER_OPEN_LIST && adj_vector[i] != FRONTIER_CLOSE_LIST && adj_vector[i] != MAP_CLOSE_LIST) {
								q_f.push(adj_vector[i]);
								cell_states[adj_vector[i]] = FRONTIER_OPEN_LIST;
							}
						}
					}
				}
				cell_states[n_cell] = FRONTIER_CLOSE_LIST;
			}
			frontiers.push_back(new_frontier);
			//
			for(vector<int>::iterator it = new_frontier.begin(); it != new_frontier.end(); ++it) {
				cell_states[*it] = MAP_CLOSE_LIST;
			}
		}
		//
		get_neighbours(adj_vector, cur_pos, map_width);

		for (int i = 0; i < 8; ++i) {
			if(adj_vector[i] < map_size && adj_vector[i] >= 0) {
				if(cell_states[adj_vector[i]] != MAP_OPEN_LIST &&  cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
					get_neighbours(v_neighbours, adj_vector[i], map_width);
					bool map_open_neighbor = false;
					for(int j = 0; j < 8; j++) {
						if(v_neighbours[j] < map_size && v_neighbours[j] >= 0) {
							if(occ_grid[v_neighbours[j]] < OCC_THRESHOLD) {
								map_open_neighbor = true;
								break;
							}
						}
					}
					if(map_open_neighbor) {
						q_m.push(adj_vector[i]);
						cell_states[adj_vector[i]] = MAP_OPEN_LIST;
					}
				}
			}
		}
		cell_states[cur_pos] = MAP_CLOSE_LIST;
	}
	return frontiers;
}

void get_neighbours(int n_array[], int position, int map_width) {
	n_array[0] = position - map_width - 1;
	n_array[1] = position - map_width; 
	n_array[2] = position - map_width + 1; 
	n_array[3] = position - 1;
	n_array[4] = position + 1;
	n_array[5] = position + map_width - 1;
	n_array[6] = position + map_width;
	n_array[7] = position + map_width + 1;
}

bool is_frontier_point(int occ_grid[], int point, int map_size, int map_width) {
	// The point under consideration must be known.
	if(occ_grid[point] == -1) {
		return false;
	}
	//
	int locations[8]; 
	get_neighbours(locations, point, map_width);
	for(int i = 0; i < 8; i++) {
		if(locations[i] < map_size && locations[i] >= 0) {
			// The state in the map is unknown, hence frontier point.
			if(occ_grid[locations[i]]== -1) {
				return true;
			}
		}
	}
	return false;
}

int get_row_from_offset(int offset, int width) {
	return offset / width;
}

int get_column_from_offset(int offset, int width) {
	return offset % width;	
}