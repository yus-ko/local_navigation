#include<local_navigation/obstacleAvoidance.h>

obstacleAvoidance::obstacleAvoidance()
	:d(0.3145),angle_min(45),angle_max(135), angle_div(1.0)
{
	//ROS_INFO("subscriber define");
	//subscriber
	sub1=nhSub1.subscribe("classificationDataEstimateVelocity",1,&obstacleAvoidance::cluster_callback,this);
	sub2=nhSub1.subscribe("robotOdometry",1,&obstacleAvoidance::robotOdom_callback,this);
	sub3=nhSub1.subscribe("goalOdometry",1,&obstacleAvoidance::goalOdom_callback,this);
	//publisher
	//ROS_INFO("publisher define");
    pub= nhPub.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	//デバッグ用
	// pubDebPcl= nhDeb.advertise<sensor_msgs::PointCloud2>("debugEstimatedVelocity", 1);
	pubDebMarkerArray= nhDeb.advertise<visualization_msgs::MarkerArray>("crossPointMarkerArray", 1);
	pubDebCross= nhDeb.advertise<visualization_msgs::MarkerArray>("crossPointCheckerResult", 1);
	pubDebHst = nhDeb.advertise<visualization_msgs::MarkerArray>("histgramChecker", 1);
	pubDebOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputChecker", 1);
	pubDebCPVFHOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputCPVFHChecker", 1);
	//launchファイルからパラメータの読み込み
	//ROS_INFO("setLaunchParam");
	setLaunchParam();
	//クロスポイントチェッカーデフォルト値入力
	setDefaultCrossPointChecker();
	//
	//vfhクラスのパラメータ設定
	vfh_c.set_histgram_param(angle_min,angle_max, angle_div);
	vfh_c.set_dis_threshold(dis_th);
	vfh_c.set_eta(eta_g, eta_curAngle, eta_prevAngle);
	//rqt_reconfigure
	f = boost::bind(&obstacleAvoidance::configCallback, this, _1, _2);
	server.setCallback(f);
	ROS_INFO("ready");
}
obstacleAvoidance::~obstacleAvoidance(){
}