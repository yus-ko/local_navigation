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
void obstacleAvoidance::robotOdom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("robotOdom_callback");
    //データをコピー
	robotOdom = *msg;
	//move manage method
	// manage();
}
void obstacleAvoidance::goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("goalOdom_callback");
    //データをコピー
	goalOdom = *msg;
	//move manage method
	// manage();
}

void obstacleAvoidance::manage(){
	ROS_INFO("into manage");
	//
	ROS_INFO("publishData");
	publishData();
	ROS_INFO("debug");
	debug();
}
// 障害物タイプをラベリング, 
void obstacleAvoidance::labelObstacles(){
	std::vector<int> obstacleType;
	//process

}
//障害物とx,y座標の交差位置を算出
// 相対速度を使用する
void obstacleAvoidance::crossPointDetect(){
	//
	// struct crossPoint{
	// 	float x;//交差位置x
	// 	float y;//交差位置y
	// 	float dis;//交差位置とロボットの距離
	// 	float t;//交差時の時間
	// 	int index;//障害物番号
	// };
	std::vector<crossPoint> crsPts;
	//process
}
// 評価関数で目標角度を設定
void obstacleAvoidance::evaluation(float& angle){
	std::vector<int> obstacleType;
	//process

}
// 最適角度探索
void obstacleAvoidance::searchProcess(){
	//探索回数
	int count = 0;
	const int countThreshold =10;
	//探査対象
	float vel;
	float angle;
	//pre process
	labelObstacle();
	//探索処理
	while(count++ > countThreshold){
		//探索プロセス
		//探査値の設定
		setCmdVel();
		setCmdAngle();
		//交差位置算出
		crossPointDetect();
		//評価
		evaluation(angle);
	}
}
// セット命令速度(最適探査用)
void obstacleAvoidance::setCmdVel(){
	// processs
}
// セット目標角度(最適探査用)
void obstacleAvoidance::setCmdAngle(){
	// processs
}
// データ送信
void obstacleAvoidance::publishData(){
    // pub.publish(pubData);
}