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
#include "actionlib/client/simple_action_client.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class TurtlebotExploration {

public:
	// Construst a new RandomWalk object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	TurtlebotExploration(ros::NodeHandle& nh) :
			fsm(FSM_MOVE_FORWARD), rotateStartTime(ros::Time::now()), rotateDuration(0.f) {
		// Initialize random time generator
		srand(time(NULL));
		//tfListener = &list;
		//ac = &mbc;
		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		//  the queue to be sent, only the last command will be sent)
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);		
		frontier_publisher = nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);
		// Subscribe to the simulated robot's laser scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		//laserSub = nh.subscribe("base_scan", 1, &TurtlebotExploration::commandCallback,this);
		mapSub = nh.subscribe("map", 10, &TurtlebotExploration::mapCallback, this);
		//
		frontier_cloud.header.frame_id = "map";
	}
	;

	// Send a velocity command
	void move(double linearVelMPS, double angularVelRadPS) {
		geometry_msgs::Twist msg; // The default constructor will set all commands to 0
		msg.linear.x = linearVelMPS;
		msg.angular.z = angularVelRadPS;
		commandPub.publish(msg);
		ros::spinOnce();
	}
	;	

	void mapCallback( const nav_msgs::OccupancyGrid& map )
	{
		//tfListener->waitForTransform("/map", "/odom", ros::Time(0), ros::Duration(3.0));
		//tfListener->lookupTransform("/map", "/odom", ros::Time(0), transform);
		//tfListener->lookupTransform("/odom", "/base_link", ros::Time(0),transform);
		//
		float resolution = map.info.resolution;
		//ROS_INFO("Resolution: %f, map width: %d, map_height: %d", resolution, map.info.width, map.info.height);
		//float x = transform.getOrigin().x() / resolution;
		//float y = transform.getOrigin().y() / resolution;
		//
		float map_x = map.info.origin.position.x / resolution;
		float map_y = map.info.origin.position.y / resolution;
		//
		float x = 0. - map_x;
		float y = 0. - map_y;
		//ROS_INFO("Index: %f", x + (y * map.info.width));

		vector<vector<int> > frontiers = wfd(map, map.info.height, map.info.width, x + (y * map.info.width));
		int num_points = 0, largest_frontier_i = 0, largest_frontier_size = 0;
		for(int i = 0; i < frontiers.size(); i++) {
			// Find the largest frontier.
			if(frontiers[i].size() > largest_frontier_size) {
				largest_frontier_size = frontiers[i].size();
				largest_frontier_i = i;
			}

			for(int j = 0; j < frontiers[i].size(); j++) {
				num_points++;
			}
		}

		//ROS_INFO("Found %d frontiers.", frontiers.size());
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
		frontier_publisher.publish(frontier_cloud);
		ROS_INFO("published cloud!");
		//
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		//
		bool at_target = false;
		int attempts = 0;
		while(!at_target && attempts < 5) {
			if(attempts > 0){
				largest_frontier_i = (rand() % frontiers.size());
				largest_frontier_size = frontiers[largest_frontier_i].size();
				at_target = false;
			}
			attempts++;
			// Get a random point on the largest frontier.
			ROS_INFO("Largest frontier index: %d, size: %d, number of frontiers: %d, largest_frontier_size: %d", 
				largest_frontier_i, largest_frontier_size, frontiers.size(), largest_frontier_size);
			//
			int frontier = frontiers[largest_frontier_i][rand() % largest_frontier_size];
			ROS_INFO("Frontier index: %d, frontier_point: %d", largest_frontier_i, frontier);			
			goal.target_pose.pose.position.x = ((frontier % map.info.width) + map_x) * resolution;
			goal.target_pose.pose.position.y = ((frontier / map.info.width) + map_y) * resolution;
			goal.target_pose.pose.orientation.w = 1.0;
			ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			//
			MoveBaseClient ac("move_base", true);
			//wait for the action server to come up
			while(!ac.waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the move_base action server to come up");
			}
			ac.sendGoal(goal);
			ac.waitForResult();

			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				at_target = true;
			  	ROS_INFO("Hooray, the base moved to %f,%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			} else {
			  	ROS_INFO("The base failed to move");
			}
		}
	}
	;

	// Main FSM loop for ensuring that ROS messages are
	// processed in a timely manner, and also for sending
	// velocity controls to the simulated robot based on the FSM state
	void spin() {
		ros::Rate rate(10); // Specify the FSM loop rate in Hz
		while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
			frontier_publisher.publish(frontier_cloud);
			ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
			rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
		}
	}
	;

	enum FSM {
		FSM_MOVE_FORWARD, FSM_ROTATE
	};
	// Tunable parameters

	// TODO: tune parameters as you see fit
	const static double MIN_SCAN_ANGLE_RAD = -10.0 / 180 * M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +10.0 / 180 * M_PI;
	const static float PROXIMITY_RANGE_M = 1; // Should be smaller than sensor_msgs::LaserScan::range_max
	const static double FORWARD_SPEED_MPS = 1;
	const static double ROTATE_SPEED_RADPS = M_PI / 2;

protected:
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Publisher posePublisher;
	sensor_msgs::PointCloud frontier_cloud;
	ros::Publisher frontier_publisher;
	ros::Subscriber mapSub;
	// ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
	enum FSM fsm; // Finite state machine for the random walk algorithm
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Duration rotateDuration; // Duration of the rotation
	tf::TransformListener *tfListener;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "TurtlebotExploration"); // Initiate new ROS node named "random_walk"
	ros::NodeHandle n;
	TurtlebotExploration walker(n); // Create new random walk object
	ROS_INFO("INFO!");
	walker.spin(); // Execute FSM loop
	return 0;
}
;
