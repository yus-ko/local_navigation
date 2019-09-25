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
void obstacleAvoidance::robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg)
{
	ROS_INFO("robotEncoder_callback");
    //データをコピー
	robotEncoder = *msg;
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
//障害物１つに対するx,y座標の交差位置を算出(交差位置を返す)
// 相対速度を使用する
crossPoint obstacleAvoidance::getCrossPoint(int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cmd_vel, float& cmd_angle){
	// 
	// struct crossPoint{
	// 	float x;//交差位置x
	// 	float y;//交差位置y
	// 	float dis;//交差位置とロボットの距離
	// 	float t;//交差時の時間
	// 	int index;//障害物番号
	// };
	// std::vector<crossPoint> crsPts;
	//ロボット速度 
	// 現在の走行速度
	float v = (robotEncoder.vel.r + robotEncoder.vel.l)/2;
	float w = (robotEncoder.vel.r - robotEncoder.vel.l)/(2 * d);
	float Vrx = v * cos(w*1 + M_PI_2);
	float Vry = v * sin(w*1 + M_PI_2);
	// 目標速度(探査対象)
	//cmd_angle は水平右をx軸, 正面をy軸とする
	float Vrx_c = cmd_vel * cos(cmd_angle);
	float Vry_c = cmd_vel * sin(cmd_angle);
	//障害物
	// 位置
	float Xox = gpRef.x;
	float Xoy = gpRef.y;
	// 速度
	float Vox = twistRef.linear.x;
	float Voy = twistRef.linear.y;
	// 番号
	int index = indexRef;
	//交差位置
	crossPoint crsPt;
	bool safeObstacle = false;
	float Vcx = Vox - (Vrx_c - Vrx);
	float Vcy = Voy - (Vry_c - Vry);
	crsPt.safe = false;
	// 場合分け
	bool straight_y = false;	
	// 傾きが無限大に近い
	float angle = atan2(Vcy,Vcx);
	float angleThreshold = M_PI/18;//10 deg :後でrqt_reconfigureで設定できるようにする
	if(std::abs(angle) < angleThreshold){
		straight_y = true;
	}
	//正面方向で直線移動の障害物
	if(straight_y){
		crsPt.x = Xox;
		crsPt.y = 0;
		crsPt.dis = Xox;
		crsPt.t = Xoy/Vcy;
		crsPt.index = index;
	}
	//それ以外
	else{
		//直線の式
		float a = Vcy/Vcx;//X軸の右が正
		float b = Xoy - a*Xox;
		//交差位置 仮
		// x = 0
		crossPoint crsPt_x0;
		crsPt_x0.x = 0;
		crsPt_x0.y = b;
		crsPt_x0.dis = crsPt_x0.y;
		crsPt_x0.t = Xox/Vcx;//
		// y = 0
		crossPoint crsPt_y0;
		crsPt_y0.x = - a / b;
		crsPt_y0.y = 0;
		crsPt_y0.dis = crsPt_y0.x;
		crsPt_y0.t = Xoy/Vcy;//
		// 時間t が短い方を採用 and t > 0
		if(crsPt_x0.t < 0 && crsPt_y0.t < 0){
			//時間がどちらもマイナス -> 遠ざかっている障害物
			if(crsPt_x0.t > crsPt_y0.t){
				crsPt = crsPt_x0;
			}
			else{
				crsPt = crsPt_y0;
			}
			crsPt.safe = true;
		}
		else if(crsPt_x0.t > 0 && crsPt_y0.t > 0){
			//時間がどちらもプラス
			if(crsPt_x0.t > crsPt_y0.t){
				crsPt = crsPt_y0;
			}
			else{
				crsPt = crsPt_x0;
			}
		}
		else{
			//時間がどっちかがプラス -> プラスの値を格納
			if(crsPt_x0.t > crsPt_y0.t){
				crsPt = crsPt_x0;
			}
			else{
				crsPt = crsPt_y0;
			}
		}
	}
	return crsPt;
}
//障害物データ群に対する各x,y座標の交差位置を算出(交差位置の配列)
// 相対速度を使用する
void obstacleAvoidance::crossPointsDetect(std::vector<crossPoint>& crsPts, float& cmd_vel, float& cmd_angle){
	//
	// struct crossPoint{
	// 	float x;//交差位置x
	// 	float y;//交差位置y
	// 	float dis;//交差位置とロボットの距離
	// 	float t;//交差時の時間
	// 	int index;//障害物番号
	// };
	// std::vector<crossPoint> crsPts;
	//ロボット速度 
	// 目標速度(探査対象)
	// float cmd_vel;
	// float cmd_angle;
	// 番号
	// int index;
	//交差位置を算出
	// crsPts.resize(clstr.data.size());
	for(int k=0; k<clstr.data.size(); k++){
		crsPts[k] = getCrossPoint(k, clstr.data[k].gc, clstr.twist[k],cmd_vel,cmd_angle);
	}
}
//交差位置に対するコストを算出する
float obstacleAvoidance::culcCrossPointCost(crossPoint& crsPt){
	// eta_cp : cost算出用パラメータ
	//コスト関数: あとで変更する予定
	float cost_cp = eta_cp / (eta_cp + crsPt.dis);
	return cost_cp;
}
// 交差位置に対するコストを算出, 取得する
float obstacleAvoidance::getCrossPointCost(float& cmd_vel, float& cmd_angle){
	// 交差位置ベクトル
	std::vector<crossPoint> crsPts;
	crsPts.resize(clstr.data.size());
	// 交差位置と障害物状態の取得
	crossPointsDetect(crsPts,cmd_vel,cmd_angle);
	// 衝突検出フラグ
	// cost算出
	float sumCost_cp;//交差位置に対するコスト値
	for(int k = 0; k < crsPts.size(); k++){
		if(crsPts[k].safe){
			continue;
		}
		sumCost_cp += culcCrossPointCost(crsPts[k]);
	}
	return sumCost_cp;
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
	labelObstacles();
	//探索処理
	while(count++ > countThreshold){
		//探索プロセス
		//探査値の設定
		setCmdVel();
		setCmdAngle();
		//交差位置算出
		// crossPointsDetect();
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