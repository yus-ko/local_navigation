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
	cur_vel = (robotEncoder.vel.r + robotEncoder.vel.l)/2;
	cur_angVel = (robotEncoder.vel.r - robotEncoder.vel.l)/(2 * d);
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
void obstacleAvoidance::update_goal_position(){
	//未完
}
void obstacleAvoidance::manage(){
	ROS_INFO("into manage");
	//現在地とゴール位置の位置関係更新
	update_goal_position();
	//ヒストグラム作成
	create_histgram();
	//
	// std::vector<double> histgram_dis;
    // vfh_c.get_histgram_dis(histgram_dis);
	// ROS_INFO_STREAM("histgram_dis size: "<< histgram_dis.size());
	//
	create_binary_histgram(robotRadius, marginRadius);
	//探索処理
	float tagVel, tagAng;
	searchProcess(tagVel, tagAng);
	ROS_INFO_STREAM("target vel =" <<tagVel<<"\n"<< "target angle =" <<tagAng);
	//命令速度生成
	geometry_msgs::Twist cmd = controler(tagVel, tagAng);
	ROS_INFO_STREAM("publishData = \n" <<cmd);
	publishData(cmd);
	ROS_INFO("debug");
	debug();
}
// 障害物が接近障害物か判断
bool obstacleAvoidance::checkSafetyObstacle(float& t, float& angle, float& x, float& y){
	//Safe Time range
	float safeTime = 100;
	//process
	//時間がマイナスor当分ぶつからない
	if( t > safeTime){
		return true;
	}
	else if( t < 0){
		return true;
	}
	//
	//ROS_INFO("angle:%f",angle);
	if(x > 0){
		if(y > 0){//第1象限
			if(angle <= M_PI_2 && angle >= 0){
				//SAFE
				return true;
			}
			else{
				//WARNIGN
				//ROS_INFO("Num 1 field WARNIGN");
			}
		}
		else{//第4象限
			if(angle <=-M_PI_2 && angle > -M_PI){
				//SAFE
				return true;
			}
			else{
				//WARNIGN					
				//ROS_INFO("Num 4 field WARNIGN");
			}
		}
	}
	else{
		if(y > 0){//第2象限
			if(angle >= M_PI_2 && angle <= M_PI){
				//SAFE
				return true;		
			}
			else{
				//WARNIGN		
				//ROS_INFO("Num 2 field WARNIGN");	
			}
		}
		else{//第3象限
			if(angle >=-M_PI && angle < -M_PI_2){
				//SAFE
				return true;			
			}
			else{
				//WARNIGN	
				//ROS_INFO("Num 3 field WARNIGN");	
			}
		}
	}
	return false;
}
//障害物１つに対するx,y座標の交差位置を算出(交差位置を返す)
// 相対速度を使用する
crossPoint obstacleAvoidance::getCrossPoint(int& cp_num, std::vector<crossPoint>& crsPts, int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cur_vel, float& cmd_dV, float& cmd_dAng){
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
	float v = cur_vel;
	// float w = cur_angVel;
	// float Vrx = v * cos(w*1 + M_PI_2);
	// float Vry = v * sin(w*1 + M_PI_2);
	// 目標速度(探査対象)
	//cmd_dAng は水平右をx軸, 正面をy軸とする
	float dVrx_c = (cmd_dV+v) * cos(cmd_dAng);
	float dVry_c = (cmd_dV+v) * sin(cmd_dAng);
	// ROS_INFO("Vr(x,y):(%f,%f)",dVrx_c,dVry_c);
	//障害物
	// 位置
	float Xox = gpRef.x;
	float Xoy = gpRef.y;
	// 速度
	float Vox = twistRef.linear.x;
	float Voy = twistRef.linear.y - v;
	// 番号
	int index = indexRef;
	//交差位置
	crossPoint crsPt;
	// ROS_INFO("Vo(x,y):(%f,%f)",Vox,Voy);
	float Vcx = Vox - dVrx_c;
	float Vcy = Voy - dVry_c;
	crsPt.vx = Vcx;
	crsPt.vy = Vcy;
	// ROS_INFO("Vc(x,y):(%f,%f)",Vcx,Vcy);
	crsPt.safe = false;
	// 場合分け
	//相対速度ゼロ
	if(Vcx == 0&& Vcy ==0){
		crsPt.safe = true;
		return crsPt;
	}
	//障害物が離れていっているとき
	// process
	// 
	// 直線(y軸方向で接近)
	bool straight_y = false;	
	// 傾きが無限大に近い
	float angle = atan2(Vcy,Vcx);
	float angleThreshold = M_PI/18;//10 deg :後でrqt_reconfigureで設定できるようにする
	float frontAngle = M_PI_2;
	// //ROS_INFO("angle:%f",angle/M_PI * 180);
	// if(std::abs(angle + frontAngle) < angleThreshold){
	// 	straight_y = true;
	// }
	// //正面方向で直線移動の障害物
	// if(straight_y){
	// 	//ROS_INFO("Xox:%f",Xox);
	// 	crsPt.x = Xox;
	// 	crsPt.y = 0;
	// 	crsPt.dis = Xox;
	// 	crsPt.t = (0-Xoy)/Vcy;
	// 	crsPt.index = index;
	// }
	//それ以外
	// else{
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
		//--おおよそ直線方向接近物ー＞直進接近物に近似をやめ、2つの交差位置をどちらも採用
		if(checkSafetyObstacle(crsPt_x0.t, angle,Xox,Xoy)){
			crsPt_x0.safe = true;
		}
		else{
			crsPt_x0.safe = false;
		}
		if(checkSafetyObstacle(crsPt_y0.t, angle,Xox,Xoy)){
			crsPt_y0.safe = true;
		}
		else{
			crsPt_y0.safe = false;
		}
		crsPts[cp_num++] = crsPt_x0;
		crsPts[cp_num++] = crsPt_y0;
		//--
		// //ROS_INFO("x0,y0;(%f,%f),(%f,%f)",crsPt_x0.x,crsPt_x0.y,crsPt_y0.x,crsPt_y0.y);
		// // 時間t が短い方を採用 and t > 0
		// if(crsPt_x0.t < 0 && crsPt_y0.t < 0){
		// 	//時間がどちらもマイナス -> 遠ざかっている障害物
		// 	if(crsPt_x0.t > crsPt_y0.t){
		// 		crsPt = crsPt_x0;
		// 	}
		// 	else{
		// 		crsPt = crsPt_y0;
		// 	}
		// }
		// else if(crsPt_x0.t > 0 && crsPt_y0.t > 0){
		// 	//時間がどちらもプラス
		// 	if(crsPt_x0.t > crsPt_y0.t){
		// 		crsPt = crsPt_y0;
		// 	}
		// 	else{
		// 		crsPt = crsPt_x0;
		// 	}
		// }
		// else{
		// 	//時間がどっちかがプラス -> プラスの値を格納
		// 	if(crsPt_x0.t > crsPt_y0.t){
		// 		crsPt = crsPt_x0;
		// 	}
		// 	else{
		// 		crsPt = crsPt_y0;
		// 	}
		// }
	// }
	//ROS_INFO("%f,crsPt:%f,%f",crsPt.t,crsPt.x,crsPt.y);
	// if(crsPt.t > safeTime){
	// 	crsPt.safe = true;
	// }
	// if(checkSafetyObstacle(crsPt.t, angle,Xox,Xoy)){
	// 	crsPt.safe = true;
	// }
	// else{
	// 	crsPt.safe = false;
	// }
	// return crsPt;
}
void obstacleAvoidance::getCrossPoints(crossPoint& crsPt_x0, crossPoint& crsPt_y0, int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang){
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
	float Vrx = cur_vel * cos(cur_ang);
	float Vry = cur_vel * sin(cur_ang);
	// 目標速度(探査対象)
	//cmd_dAng は水平右をx軸, 正面をy軸とする
	float dVrx_c = (cmd_dV+cur_vel) * cos(cmd_ang);
	float dVry_c = (cmd_dV+cur_vel) * sin(cmd_ang);
	// ROS_INFO("%d,Vr(x,y):(%f,%f)",indexRef,dVrx_c,dVry_c);
	//障害物
	// 位置
	float Xox = gpRef.x;
	float Xoy = gpRef.y;
	// 速度
	float Vox = twistRef.linear.x;// + Vrx;
	float Voy = twistRef.linear.y;// + Vry;
	// ROS_INFO("Aft: Vo(x,y):(%f,%f)",Vox,Voy);
	// 番号
	int index = indexRef;
	//交差位置
	float Vcx = Vox - dVrx_c;
	float Vcy = Voy - dVry_c;
	crsPt_x0.vx = Vcx;
	crsPt_x0.vy = Vcy;
	// ROS_INFO("Vc(x,y):(%f,%f)",Vcx,Vcy);
	crsPt_x0.safe = false;
	// 場合分け
	//相対速度ゼロ
	if(Vcx == 0&& Vcy ==0){
		crsPt_x0.safe = true;
	}
	crsPt_y0 = crsPt_x0;
	//障害物が離れていっているとき
	// 傾き
	float angle = atan2(Vcy,Vcx);
	float frontAngle = M_PI_2;
	//直線の式
	float a = Vcy/Vcx;//X軸の右が正
	float b = Xoy - a*Xox;
	//交差位置 仮
	// x = 0
	// ROS_INFO("%f x + %f ",a,b);
	crsPt_x0.x = 0;
	crsPt_x0.y = b;
	crsPt_x0.dis = crsPt_x0.y;
	crsPt_x0.t = (0-Xox)/Vcx;//
	// y = 0
	crsPt_y0.x = - b /a;	
	crsPt_y0.y = 0;
	crsPt_y0.dis = crsPt_y0.x;
	crsPt_y0.t = (0-Xoy)/Vcy;//
	//--おおよそ直線方向接近物ー＞直進接近物に近似をやめ、2つの交差位置をどちらも採用
	if(checkSafetyObstacle(crsPt_x0.t, angle,Xox,Xoy)){
		crsPt_x0.safe = true;
	}
	else{
		crsPt_x0.safe = false;
	}
	if(checkSafetyObstacle(crsPt_y0.t, angle,Xox,Xoy)){
		crsPt_y0.safe = true;
	}
	else{
		crsPt_y0.safe = false;
	}
}
//障害物データ群に対する各x,y座標の交差位置を算出(交差位置の配列)
// 相対速度を使用する
void obstacleAvoidance::crossPointsDetect(float& cmd_dV, float& cmd_dAng){
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
	// float cmd_dV;
	// float cmd_dAng;
	// 番号
	// int index;
	//交差位置を算出
	// crsPts.resize(clstr.data.size());
	int cp_num = 0;
	crsPts.resize((int)clstr.data.size()*2);
	for(int k=0; k<clstr.data.size(); k++){
		getCrossPoint(cp_num, crsPts, k, clstr.data[k].gc, clstr.twist[k], cur_vel,cmd_dV,cmd_dAng);
	}
}
void obstacleAvoidance::crossPointsDetect(std::vector<crossPoint>& crsPts, float& cur_vel_temp, float& cur_angle_temp, float& cmd_dV, float& cmd_dAng){
	crsPts.resize((int)clstr.data.size()*2);
	for(int k=0; k<clstr.data.size(); k++){
		getCrossPoints(crsPts[k*2], crsPts[k*2+1], k, clstr.data[k].gc, clstr.twist[k],cur_vel_temp,  cur_angle_temp, cmd_dV,cmd_dAng);
		//getCrossPoint(cp_num, crsPts, k, clstr.data[k].gc, clstr.twist[k],cur_vel_temp, cmd_dV,cmd_dAng);
	}
}
//--コスト関数
//汎用コスト関数
float obstacleAvoidance::generalCostFunction(float& eta, float& value){
	// eta:  valueに対する重み
	//コストを返す( 0 〜 1 )
	return (eta / (eta + value));
}
double obstacleAvoidance::costCrossPoint(crossPoint& crsPt, float eta_cp){
	// eta_cp : cost算出用パラメータ
	//コスト関数: あとで変更する予定
	// return (-(pow(crsPt.dis/crsPt.t/eta_cp,2.0)));
	return ((pow(1/crsPt.dis/eta_cp,2.0)/crsPt.t));
}

double obstacleAvoidance::getCrossPointCost(std::vector<crossPoint>& crsPts, float eta_cp){
	// 衝突検出フラグ
	// cost算出
	float sumCost_cp=0;//交差位置に対するコスト値
	for(int k = 0; k < crsPts.size(); k++){
		if(crsPts[k].safe){
			continue;
		}
		sumCost_cp += costCrossPoint(crsPts[k], eta_cp);
	}
	return sumCost_cp;
}

geometry_msgs::Twist obstacleAvoidance::controler(float& tagVel, float& tagAng){
	//p制御
	double cur_ang = 90;//正面を向いているため
	float gainP = 0.01;
	float tagAngVel = (tagAng-cur_ang)*gainP;
	//
	geometry_msgs::Twist twist;
	twist.linear.x =tagVel; 
	twist.linear.y =0; 
	twist.linear.z =0;
	twist.angular.x =0;
	twist.angular.y =0;
	twist.angular.z =tagAngVel;

	return twist;
}
void obstacleAvoidance::searchProcess(float& tagVel, float& tagAng){
	//only angle
	float dV=debugCmd_vel;
	vfh_angleSearch(tagAng,cur_vel, dV);
	//float& cur_vel_temp, float& cmd_dV, float& cmd_dAng)
	//angle and vel 
	// search_vel_ang(tagVel, tagAng);
	//
	prev_tagAng = tagAng;
	tagVel = cur_vel + dV;
}
// 最適角度探索
float obstacleAvoidance::vfh_angleSearch(float& target_angle_temp, float& cur_vel_temp, float& cmd_dV){//return cost
	
	std::vector<bool> histgram_bi;
    vfh_c.get_histgram_bi(histgram_bi);
	//
	goal_x=debugGoalPosX;
	goal_y=debugGoalPosY;
	//
	float goalAng = atan2(goal_y, goal_x)*180/M_PI;
	//weight正規化
    double sum_weight = k_cp + k_curAngle + k_prevAngle + k_cp;
    k_g/=sum_weight;
    k_curAngle/=sum_weight;
    k_prevAngle/=sum_weight;
    k_cp/=sum_weight;
    //コスト算出
    double min_cost = 10000;
    int min_num = -1;
	std::vector<crossPoint> min_cost_crsPts;
	ROS_INFO("cur_vel_temp, prev_tagAng, cmd_dV:(%f,%f -- %f)",cur_vel_temp, prev_tagAng, cmd_dV);
	// ROS_INFO_STREAM("histgram_bi.size: "<<histgram_bi.size());
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        // double l = histgram_dis[i];
        double ang = vfh_c.transform_numToAngle(i);//deg
        float cmd_ang = ang*M_PI/180;
		double cur_ang = 90;//正面を向いているため

		//
		float cmd_dAng_rad = cmd_ang;//(cmd_ang-cur_ang);
		float prev_tagAng_rad = prev_tagAng*M_PI/180;
		//交差位置算出
        std::vector<crossPoint> crsPts;
		//現在の進行方向はprev_tagAngの方向であると仮定
		crossPointsDetect(crsPts, cur_vel_temp, prev_tagAng_rad, cmd_dV, cmd_dAng_rad);
		//コスト算出
        double goalCost = vfh_c.cost_goal_angle(ang, goalAng);
        double angCost = vfh_c.cost_current_angle(ang, cur_ang);
        double prevAngCost = vfh_c.cost_prev_select_angle(ang, prev_tagAng);
        double crossCost = getCrossPointCost(crsPts,eta_cp);
        double cost = k_g * goalCost + k_curAngle * angCost + k_prevAngle * prevAngCost + k_cp*crossCost;
        // std::cout<<i<<":"<<cost<<std::endl;
		if(min_cost > cost){
            min_cost = cost;
            min_num = i;
			//デバッグ用
			// min_cost_crsPts.clear();
			min_cost_crsPts = crsPts;
        }
    }
	//目標角度
	target_angle_temp = vfh_c.transform_numToAngle(min_num);
	if(min_cost == 1){
		ROS_INFO_STREAM("Not found space");
	}
	//デバッグ関数に交差位値情報を渡す
	ROS_INFO_STREAM("min crsPts size:" << min_cost_crsPts.size());
	ROS_INFO_STREAM("cur_vel, dV" << cur_vel_temp <<","<< cmd_dV);
	showOutPut(min_cost_crsPts, cur_vel_temp + cmd_dV, min_num);
	return min_cost;
}
void obstacleAvoidance::search_vel_ang(float& target_vel, float& target_angle){
	//探索回数
	int count = 0;
	const int countThreshold =10;
	//最適化対象: 評価値
	double evalMax;//最大値
	double evalVal = evalMax;
	//--全探査
	float dV = 0;//探査対象dv
	float dAng = 0;//探査対象dAng
	for(float search_dV = cur_vel-dV_range; dV <= cur_vel+dV_range; dV +=dV_div){
		if(search_dV + cur_vel > 0.6 || search_dV + cur_vel <= 0.0){//
			continue;
		}
		float searching_angle;//探査対象angle
		//評価
		float evalTemp = vfh_angleSearch(searching_angle, cur_vel, search_dV);
		if(evalTemp < evalVal){
			evalVal = evalTemp;
			dV = search_dV;
			dAng = searching_angle;
		}
	}
	target_vel = dV + cur_vel;
	target_angle = dAng;
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
void obstacleAvoidance::publishData(geometry_msgs::Twist& pubData){
    pub.publish(pubData);
}
//ヒストグラム配列の作成
void obstacleAvoidance::create_histgram(){
	//データ定義
	//選択角度範囲
	// float angle_min = M_PI_4;//最小センサ角度
	// float angle_max = M_PI_2 + M_PI_4;//最大センサ角度
	// float angle_div = 1.0;//解像度
	//
	// std::vector<double> hst;//ヒストグラム配列
	// hst.resize( (int)((angle_max - angle_min)/angle_div) );
	//process
	// ROS_INFO("create_histgram");
	for(int k =0; k < clstr.data.size(); k++){//クラスタ数
		//静止障害物のみADD
		if(clstr.twist[k].linear.x !=0 || clstr.twist[k].linear.y != 0){
			std::cout<<"("<<k<<" : moving),";
			continue;
		}
		std::cout<<"("<<k<<" : static),";
		//各クラスタに含まれる点群を取得しヒストグラムを作成
		for(int m = 0; m < clstr.data[k].pt.size(); m++){
			float angleTemp = atan2(clstr.data[k].pt[m].y,clstr.data[k].pt[m].x)*180/M_PI;
			float disTemp = sqrt(clstr.data[k].pt[m].y*clstr.data[k].pt[m].y + clstr.data[k].pt[m].x*clstr.data[k].pt[m].x);
			vfh_c.add_histgram_dis(angleTemp, disTemp);
		}
	}
}
void obstacleAvoidance::create_binary_histgram(float& robotRadius, float& marginRadius){
	// ROS_INFO("create_binary_histgram");
	vfh_c.create_binary_histgram(robotRadius,marginRadius);
}
