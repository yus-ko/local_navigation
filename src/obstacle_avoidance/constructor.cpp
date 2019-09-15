#include<local_navigation/obstacleAvoidance.h>

obstacleAvoidance::obstacleAvoidance()
{
	//ROS_INFO("subscriber define");
	//subscriber
	// nhSub1.setCallbackQueue(&queue1);
	sub=nhSub1.subscribe("classificationDataEstimateVelocity",1,&obstacleAvoidance::cluster_callback,this);
	//publisher
	//ROS_INFO("publisher define");
    // pub= nhPub.advertise<local_navigation::ClassificationVelocityData>("classificationDataEstimateVelocity", 1);

	//デバッグ用
	// pubDebPcl= nhDeb.advertise<sensor_msgs::PointCloud2>("debugEstimatedVelocity", 1);
	// pubDebMarker= nhDeb.advertise<visualization_msgs::MarkerArray>("estimatedVelocityMarker", 1);

	//launchファイルからパラメータの読み込み
	//ROS_INFO("setLaunchParam");
	setLaunchParam();
	//
	//rqt_reconfigure
	f = boost::bind(&obstacleAvoidance::configCallback, this, _1, _2);
	server.setCallback(f);
	ROS_INFO("ready");
}
obstacleAvoidance::~obstacleAvoidance(){
}