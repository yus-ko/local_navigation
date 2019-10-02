#include <ros/ros.h>
#include <local_navigation/obstacleAvoidance.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"local_navigation_avoid");
	
	// ROS_INFO("obstacleAvoidance define");
    obstacleAvoidance oa; //
    ros::spin();
	
	return 0;
}