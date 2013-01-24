#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf/transform_datatypes.h"
#include "wavefront_frontier_detection.hpp"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "FFD.hpp"
#include "actionlib/client/simple_action_client.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class TurtlebotExploration {

public:
	// Construst a new RandomWalk object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	TurtlebotExploration(ros::NodeHandle& nh) {
		// Initialize random time generator
		srand(time(NULL));
		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		//  the queue to be sent, only the last command will be sent)
		frontier_publisher = nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);
		// Subscribe to the simulated robot's laser scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		//laserSub = nh.subscribe("base_scan", 1, &TurtlebotExploration::commandCallback,this);
		mapSub = nh.subscribe("map", 1, &TurtlebotExploration::mapCallback, this);
		//
		frontier_cloud.header.frame_id = "map";
	}
	;

	void mapCallback( const nav_msgs::OccupancyGrid& map )
	{
		//
		float resolution = map.info.resolution;
		float map_x = map.info.origin.position.x / resolution;
		float map_y = map.info.origin.position.y / resolution;
		float x = 0. - map_x;
		float y = 0. - map_y;

		vector<vector<int> > frontiers = wfd(map, map.info.height, map.info.width, x + (y * map.info.width));
		int num_points = 0;
		for(int i = 0; i < frontiers.size(); i++) {
			for(int j = 0; j < frontiers[i].size(); j++) {
				num_points++;
			}
		}
		//
		frontier_cloud.points.resize(num_points);
		int pointI = 0;
		for(int i = 0; i < frontiers.size(); i++) {
			for(int j = 0; j < frontiers[i].size(); j++) {
				frontier_cloud.points[pointI].x = ((frontiers[i][j] % map.info.width) + map_x) * resolution ;
				frontier_cloud.points[pointI].y = ((frontiers[i][j] / map.info.width) + map_y) * resolution;
				frontier_cloud.points[pointI].z = 0;
				pointI++;
			}
		}
		//
		frontier_publisher.publish(frontier_cloud);
		//ROS_INFO("published cloud! Size: %d.", num_points);
	}
	;

	// Main FSM loop for ensuring that ROS messages are
	// processed in a timely manner, and also for sending
	// velocity controls to the simulated robot based on the FSM state
	void spin() {
		ros::Rate rate(10); // Specify the FSM loop rate in Hz
		while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
			ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
			frontier_publisher.publish(frontier_cloud);
			rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
		}
	}
	;

protected:
	sensor_msgs::PointCloud frontier_cloud;
	ros::Publisher frontier_publisher;
	ros::Subscriber mapSub;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "TurtlebotExploration"); // Initiate new ROS node named "random_walk"
	ros::NodeHandle n;
	TurtlebotExploration walker(n);
	ROS_INFO("INFO! FRINTIERS");
	walker.spin(); // Execute FSM loop
	return 0;
}
;
