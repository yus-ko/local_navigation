﻿#include<local_navigation/obstacleAvoidance.h>

obstacleAvoidance::obstacleAvoidance()
	:d(0.3145),angle_min(45),angle_max(135), angle_div(1.0),goalX(0),goalY(6.0),
	RECEIVED_CLUSTER(false),RECEIVED_GOAL_ODOM(false),RECEIVED_ROBOT_ODOM(false),
	SEARCH_ONLY_ANGLE(false),MAX_COST(900000)
{
	//ROS_INFO("subscriber define");
	//subscriber
	sub1=nhSub1.subscribe("classificationDataEstimateVelocity",1,&obstacleAvoidance::cluster_callback,this);
	sub2=nhSub1.subscribe("zed_node/odom",1,&obstacleAvoidance::robotOdom_callback,this);
	sub3=nhSub1.subscribe("goalOdometry",1,&obstacleAvoidance::goalOdom_callback,this);
	sub4=nhSub1.subscribe("/encoder",1,&obstacleAvoidance::robotEncoder_callback,this);
	//publisher
	//ROS_INFO("publisher define");
    	pub= nhPub.advertise<geometry_msgs::Twist>("/beego/cmd_vel", 1);

	//デバッグ用
	// pubDebPcl= nhDeb.advertise<sensor_msgs::PointCloud2>("debugEstimatedVelocity", 1);
	pubDebMarkerArray= nhDeb.advertise<visualization_msgs::MarkerArray>("crossPointMarkerArray", 1);
	pubDebCross= nhDeb.advertise<visualization_msgs::MarkerArray>("crossPointCheckerResult", 1);
	pubDebHst = nhDeb.advertise<visualization_msgs::MarkerArray>("histgramChecker", 1);
	pubDebOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputChecker", 1);
	pubDebCPVFHOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputCPVFHChecker", 1);
	pubDebBagOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputBagOutputChecker", 1);
	pubDebRotOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("RotChecker", 1);
	pubDebOdom = nhDeb.advertise<nav_msgs::Odometry>("DeltaOdomChecker", 1);
	pubRotVel =  nhDeb.advertise<visualization_msgs::MarkerArray>("rotVelMarker", 1);
	//launchファイルからパラメータの読み込み
	setLaunchParam();
	//クロスポイントチェッカーデフォルト値入力
	setDefaultCrossPointChecker();

	//vfhクラスのパラメータ設定
	vfh_c.set_histgram_param(angle_min,angle_max, angle_div);
	vfh_c.set_dis_threshold(dis_th);
	vfh_c.set_eta(eta_g, eta_curAngle, eta_prevAngle);
	if(rqt_reconfigure){
		//rqt_reconfigure
		f = boost::bind(&obstacleAvoidance::configCallback, this, _1, _2);
		server.setCallback(f);

	}
	ROS_INFO("ready");
}
obstacleAvoidance::~obstacleAvoidance(){
}
