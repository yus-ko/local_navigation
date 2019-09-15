#include<local_navigation/obstacleAvoidance.h>

//subscribe
void obstacleAvoidance::cluster_callback(const local_navigation::ClassificationVelocityData::ConstPtr& msg)
{
	ROS_INFO("cluster_callback");
    //データをコピー
	clstr = *msg;
	//move manage method
	manage();
}
void obstacleAvoidance::manage(){
	ROS_INFO("into manage");
	//
	ROS_INFO("publishData");
	publishData();
	ROS_INFO("debug");
	debug();
}

void obstacleAvoidance::publishData(){//データ送信
    // pub.publish(pubData);
}