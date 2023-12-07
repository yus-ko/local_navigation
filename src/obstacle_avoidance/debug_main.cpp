#include <ros/ros.h>
#include <local_navigation/obstacleAvoidance.h>
#include<std_msgs/Empty.h>
#include <ros/callback_queue.h>

bool PROCESS_START;
//switch関数
void callback_function(const std_msgs::Empty::ConstPtr& msg)
{
	ROS_INFO("Turn On");
	PROCESS_START =true;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"local_navigation_avoid");
	
	// ROS_INFO("obstacleAvoidance define");
    obstacleAvoidance oa; //

	//idle process
	std_msgs::Empty empty_msg;
	ros::NodeHandle nh_s;
	ros::Publisher pub_s;
	pub_s = nh_s.advertise<std_msgs::Empty>("/robot1/mobile_base/commands/reset_odometry", 1);

	ros::Subscriber sub_s;
	ros::CallbackQueue queue;

	nh_s.setCallbackQueue(&queue);
	sub_s=nh_s.subscribe("start_swich",1,callback_function);

	PROCESS_START=false;
	while(ros::ok()&&!PROCESS_START)
	{
		ROS_INFO("Waiting switch msg");
		pub_s.publish(empty_msg);
		queue.callOne(ros::WallDuration(0.1));
	}
	ROS_INFO("start");
    ros::spin();
	
	return 0;
}