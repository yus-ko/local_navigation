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
	// publishData();
	ROS_INFO("debug");
	debug();
}
// 障害物が接近障害物か判断
bool obstacleAvoidance::checkSafetyObstacle(float& t, float& angle, float& x, float& y){
	//Safe Time range
	float safeTime = 100;
	//process
	//時間がマイナス
	if( t > safeTime){
		return true;
	}
	else if( t < 0){
		return true;
	}
	//
	ROS_INFO("angle:%f",angle);
	if(x > 0){
		if(y > 0){//第1象限
			if(angle <= -M_PI_2 && angle >= -M_PI){
				//WARNIGN
				ROS_INFO("Num 1 field WARNIGN");
			}
			else{
				//SAFE
				return true;
			}
		}
		else{//第4象限
			if(angle <=0 && angle > -M_PI_2){
				//WARNIGN					
				ROS_INFO("Num 4 field WARNIGN");
			}
			else{
				//SAFE
				return true;
			}
		}
	}
	else{
		if(y > 0){//第2象限
			if(angle >= M_PI_2 && angle <= M_PI){
				//WARNIGN		
				ROS_INFO("Num 2 field WARNIGN");			
			}
			else{
				//SAFE
				return true;
			}
		}
		else{//第3象限
			if(angle >=0 && angle < M_PI_2){
				//WARNIGN	
				ROS_INFO("Num 3 field WARNIGN");				
			}
			else{
				//SAFE
				return true;
			}
		}
	}
	return false;
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
	ROS_INFO("Vr(x,y):(%f,%f)",Vrx_c,Vry_c);
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
	ROS_INFO("Vo(x,y):(%f,%f)",Vox,Voy);
	float Vcx = Vox - (Vrx_c - Vrx);
	float Vcy = Voy - (Vry_c - Vry);
	crsPt.vx = Vcx;
	crsPt.vy = Vcy;
	ROS_INFO("Vc(x,y):(%f,%f)",Vcx,Vcy);
	crsPt.safe = false;
	// 場合分け
	//相対速度ゼロ
	if(Vcx == 0&& Vcy ==0){
		crsPt.safe = true;
		return crsPt;
	}

	// 直線(y軸方向で接近)
	bool straight_y = false;	
	// 傾きが無限大に近い
	float angle = atan2(Vcy,Vcx);
	float angleThreshold = M_PI/18;//10 deg :後でrqt_reconfigureで設定できるようにする
	float frontAngle = M_PI_2;
	ROS_INFO("angle:%f",angle/M_PI * 180);
	if(std::abs(angle + frontAngle) < angleThreshold){
		straight_y = true;
	}
	//正面方向で直線移動の障害物
	if(straight_y){
		ROS_INFO("Xox:%f",Xox);
		crsPt.x = Xox;
		crsPt.y = 0;
		crsPt.dis = Xox;
		crsPt.t = (0-Xoy)/Vcy;
		crsPt.index = index;
	}
	//それ以外
	else{
		//直線の式
		float a = Vcy/Vcx;//X軸の右が正
		float b = Xoy - a*Xox;
		//交差位置 仮
		// x = 0
		ROS_INFO("%f x + %f ",a,b);
		crossPoint crsPt_x0;
		crsPt_x0.x = 0;
		crsPt_x0.y = b;
		crsPt_x0.dis = crsPt_x0.y;
		crsPt_x0.t = (0-Xox)/Vcx;//
		crsPt_x0.safe = false;
		// y = 0
		crossPoint crsPt_y0;
		crsPt_y0.x = - b /a;	
		crsPt_y0.y = 0;
		crsPt_y0.dis = crsPt_y0.x;
		crsPt_y0.t = (0-Xoy)/Vcy;//
		crsPt_y0.safe = false;
		ROS_INFO("x0,y0;(%f,%f),(%f,%f)",crsPt_x0.x,crsPt_x0.y,crsPt_y0.x,crsPt_y0.y);
		// 時間t が短い方を採用 and t > 0
		if(crsPt_x0.t < 0 && crsPt_y0.t < 0){
			//時間がどちらもマイナス -> 遠ざかっている障害物
			if(crsPt_x0.t > crsPt_y0.t){
				crsPt = crsPt_x0;
			}
			else{
				crsPt = crsPt_y0;
			}
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
	ROS_INFO("%f,crsPt:%f,%f",crsPt.t,crsPt.x,crsPt.y);
	// if(crsPt.t > safeTime){
	// 	crsPt.safe = true;
	// }
	if(checkSafetyObstacle(crsPt.t, angle,Xox,Xoy)){
		crsPt.safe = true;
	}
	else{
		crsPt.safe = false;
	}
	return crsPt;
}
//障害物データ群に対する各x,y座標の交差位置を算出(交差位置の配列)
// 相対速度を使用する
void obstacleAvoidance::crossPointsDetect(float& cmd_vel, float& cmd_angle){
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
//--コスト関数
//汎用コスト関数
float obstacleAvoidance::generalCostFunction(float& eta, float& value){
	// eta:  valueに対する重み
	//コストを返す( 0 〜 1 )
	return (eta / (eta + value));
}
//交差位置に対するコストを算出する
float obstacleAvoidance::costCrossPoint(crossPoint& crsPt){
	// eta_cp : cost算出用パラメータ
	//コスト関数: あとで変更する予定
	return (generalCostFunction(eta_cp, crsPt.dis));
}
float obstacleAvoidance::costVFHGoalAngle(float goalAngle){//vfh+第1項
	// eta_g : cost算出用パラメータ
	//コスト関数: 目的地角度差に対するコスト関数
	return (generalCostFunction(eta_g, goalAngle));
}
float obstacleAvoidance::costVFHDeltaAngle(float delAngle){//vfh+第2項
	// eta_theta : cost算出用パラメータ
	//コスト関数: 角度変化に対するコスト関数
	return (generalCostFunction(eta_theta, delAngle));
}
float obstacleAvoidance::costVFHDeltaOmega(float delOmega){//vfh+第3項
	// eta_omega : cost算出用パラメータ
	//コスト関数: 角速度度変化に対するコスト関数
	return (generalCostFunction(eta_omega, delOmega));
}

// 交差位置に対するコストを算出, 取得する
float obstacleAvoidance::getCrossPointCost(){
	// 衝突検出フラグ
	// cost算出
	float sumCost_cp;//交差位置に対するコスト値
	for(int k = 0; k < crsPts.size(); k++){
		if(crsPts[k].safe){
			continue;
		}
		sumCost_cp += costCrossPoint(crsPts[k]);
	}
	return sumCost_cp;
}
// 評価関数で全体コストを算出
double obstacleAvoidance::evaluation(float& vel, float& angle){
	//process

	//get each cost
	float costCross = getCrossPointCost();//交差位置コスト
	float costGoalAng = costVFHGoalAngle(abs(goal_angle - angle));//目的地角度
	float costDeltaAng = costVFHDeltaAngle(abs(angle - pre_angle));//角度差
	// float costDeltaOmg = costVFHDeltaOmega(abs(omega - pre_omega));//角速度差
	float costStaticObst;//未作成
	double eval;
	eval = k_cp * costCross
	 + k_g * costGoalAng
	 + k_o * costStaticObst
	 + k_theta * costDeltaAng;
	//  + k_omega * costDeltaOmega; 
	//
	return eval;
}
// 最適角度探索
void obstacleAvoidance::searchProcess(){
	//探索回数
	int count = 0;
	const int countThreshold =10;
	//探査対象
	float vel;
	float angle;
	//最適化対象: 評価値
	double evalMax;//最大値
	double evalVal = evalMax;
	//探索処理
	while(count++ > countThreshold){
		//探索プロセス
		//探査値の設定
		setCmdVel();
		setCmdAngle();
		//交差位置算出
		crsPts.resize(clstr.data.size());// 交差位置ベクトル
		// 交差位置と障害物状態の取得
		crossPointsDetect(vel,angle);
		//
		create_histgram();
		create_binary_histgram();
		// crossPointsDetect();
		//評価
		float evalTemp = evaluation(vel, angle);
		if(evalTemp < evalVal){
			evalVal = evalTemp;
		}
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
//ヒストグラム配列の作成
void obstacleAvoidance::create_histgram(){
	//データ定義
	//選択角度範囲
	// float angle_min = M_PI_4;//最小センサ角度
	// float angle_max = M_PI_2 + M_PI_4;//最大センサ角度
	// float angle_dev = 1.0;//解像度
	//
	// std::vector<double> hst;//ヒストグラム配列
	// hst.resize( (int)((angle_max - angle_min)/angle_dev) );
	//process
	for(int k =0; k < clstr.data.size(); k++){//クラスタ数
		//障害部判断結果を使用
		if(crsPts[k].safe){
			//safe
			continue;
		}
		//各クラスタに含まれる点群を取得しヒストグラムを作成
		for(int m = 0; m < clstr.data[k].pt.size(); m++){
			float angleTemp = atan2(clstr.data[k].pt[m].y,clstr.data[k].pt[m].x)*180/M_PI;
			float disTemp = sqrt(clstr.data[k].pt[m].y*clstr.data[k].pt[m].y + clstr.data[k].pt[m].x*clstr.data[k].pt[m].x);
			vfh_c.add_histgram_dis(angleTemp, disTemp);
		}
	}
}
void obstacleAvoidance::create_binary_histgram(){
	vfh_c.create_binary_histgram();
}
