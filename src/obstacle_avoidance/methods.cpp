#include<local_navigation/obstacleAvoidance.h>

//subscribe
void obstacleAvoidance::cluster_callback(const local_navigation::ClassificationVelocityData::ConstPtr& msg)
{
	ROS_INFO("cluster_callback");
    //データをコピー
	clstr = *msg;
	//move manage method
	RECEIVED_CLUSTER = true;
	manage();
}
void obstacleAvoidance::robotOdom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("robotOdom_callback");
    //データをコピー
	robotOdom = *msg;
	RECEIVED_ROBOT_ODOM = true;
	//move manage method
	manage();
}
void obstacleAvoidance::robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg)
{
	ROS_INFO("robotEncoder_callback");
    //データをコピー
	robotEncoder = *msg;
	cur_vel = (robotEncoder.vel.r + robotEncoder.vel.l)/2;
	cur_angVel = (robotEncoder.vel.r - robotEncoder.vel.l)/(2 * d);
	RECEIVED_ROBOT_ENCODAR = true;
	//move manage method
	manage();
}
void obstacleAvoidance::goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("goalOdom_callback");
    //データをコピー
	goalOdom = *msg;
	RECEIVED_GOAL_ODOM = true;
	//move manage method
	manage();
}
void obstacleAvoidance::update_goal_position(){
	//自己位置姿勢とゴール位置からロボット座標軸上でのゴール座標を算出する
	//robotOdom, goalOdom -> relationOdom
	//ゴール位置のフレームIDをマップに設定してgoalOdomをbase_linkに座標変化すればいいのでは
	// tf::TransformListener listener_;
	// tf::Transform cam_to_target;
	// tf::poseMsgToTF(p->pose.pose, cam_to_target);
	// tf::StampedTransform req_to_cam;
	// listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
	//面倒なので位置の差と回転行列だけで良さそう

	//位置差
	std::cout<<"goal== "<<goalOdom.pose.pose.position.x<<","<<goalOdom.pose.pose.position.y<<","<<goalOdom.pose.pose.position.z<<std::endl;
	std::cout<<"robot== "<<robotOdom.pose.pose.position.y<<","<<-robotOdom.pose.pose.position.x<<","<<robotOdom.pose.pose.position.z<<std::endl;

	// relationOdom.pose.pose.position.x = goalOdom.pose.pose.position.x - robotOdom.pose.pose.position.x;
	// relationOdom.pose.pose.position.y = goalOdom.pose.pose.position.y - robotOdom.pose.pose.position.y;
	// relationOdom.pose.pose.position.x = goalOdom.pose.pose.position.x - robotOdom.pose.pose.position.y;
	// relationOdom.pose.pose.position.y = goalOdom.pose.pose.position.y - (-robotOdom.pose.pose.position.x);
	relationOdom.pose.pose.position.x = goalOdom.pose.pose.position.y - robotOdom.pose.pose.position.x;
	relationOdom.pose.pose.position.y = (-goalOdom.pose.pose.position.x) - robotOdom.pose.pose.position.y;
	relationOdom.pose.pose.position.z = goalOdom.pose.pose.position.z - robotOdom.pose.pose.position.z;
	
	tf::Quaternion quatGoal=tf::createQuaternionFromYaw(M_PI_2);//debugRobotYaw-M_PI_2);
	// quaternionTFToMsg(quatGoal, goalOdom.pose.pose.orientation);
	//角度差はロボット姿勢角度 
	tf::Quaternion quat;
	double r,p,y;
	quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(r, p, y);
	//ロボット座標系の軸を揃える
	// y += M_PI_2;//90deg回転
	
	double theta_goal = atan2(relationOdom.pose.pose.position.y,relationOdom.pose.pose.position.x);
	double theta_robot = y;
	// std::cout<<"theta_goal - theta_robot = "<<theta_goal - theta_robot<<std::endl;
	double theta_relation = theta_goal - theta_robot;
	double length = std::sqrt(std::pow(relationOdom.pose.pose.position.x,2.0) + std::pow(relationOdom.pose.pose.position.y,2.0));
	//グロ-バル座標軸で算出
	// std::cout<< length <<"*"<< "cos("<<theta_relation<<");"<<std::endl;
	// std::cout<< length <<"*"<< cos(theta_relation)<<";"<<std::endl;
	relationOdom.pose.pose.position.x = length * cos(theta_relation);
	relationOdom.pose.pose.position.y = length * sin(theta_relation);
	relationOdom.pose.pose.position.z = 0;
	// std::cout<< length <<"*"<< "sin("<<theta_relation<<");"<<std::endl;
	// std::cout<< length <<"*"<< sin(theta_relation)<<";"<<std::endl;
	// std::cout<<"relationOdom.pose.pose.position.x="<< length * cos(theta_relation)<<std::endl;
	// std::cout<<"relationOdom.pose.pose.position.y="<< length * sin(theta_relation)<<std::endl;
	// std::cout<<"relationOdom.pose.pose.position.x="<< relationOdom.pose.pose.position.x<<std::endl;
	// std::cout<<"relationOdom.pose.pose.position.y="<< relationOdom.pose.pose.position.y<<std::endl;
	quat=tf::createQuaternionFromYaw(theta_relation);
	quaternionTFToMsg(quat, relationOdom.pose.pose.orientation);
	//ゴールセット
	goal_x = -relationOdom.pose.pose.position.y;
	goal_y = relationOdom.pose.pose.position.x;
	std::cout<<"goal_xy: "<<goal_x <<","<<goal_y<<std::endl;
}
void obstacleAvoidance::manage(){
	ROS_INFO("into manage");
	if(data_check()){
		data_check_reset();
		get_time();
		if(!culc_delta_time()){
			return;
		}
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
}
bool obstacleAvoidance::data_check(){
	ROS_INFO_STREAM(
		(RECEIVED_CLUSTER ? "RECEIVED_CLUSTER" : "NOT CLUSTER") <<","<<
		(RECEIVED_GOAL_ODOM ? "RECEIVED_GOAL_ODOM" : "NOT GOAL_ODOM") <<","<<
		(RECEIVED_ROBOT_ODOM ? "RECEIVED_ROBOT_ODOM" : "NOT ROBOT_ODOM") <<","<<
		(RECEIVED_ROBOT_ENCODAR ? "RECEIVED_ROBOT_ENCODAR" : "NOT ROBOT_ENCODAR") 
	);
	return(
	    RECEIVED_CLUSTER&& RECEIVED_GOAL_ODOM&& RECEIVED_ROBOT_ODOM 
	    && RECEIVED_ROBOT_ENCODAR
	);
}
void obstacleAvoidance::data_check_reset(){
	RECEIVED_CLUSTER = false;
	RECEIVED_ROBOT_ODOM = false;
	RECEIVED_ROBOT_ENCODAR = false;
}
void obstacleAvoidance::get_time(){
	cur_time = ros::Time::now();
}
bool obstacleAvoidance::culc_delta_time(){
	if(!PROCESS_ONCE){
		delta_time_ros = cur_time - pre_time;
		delta_time = delta_time_ros.toSec();
		pre_time = cur_time;
		PROCESS_ONCE = false;
		return true;
	}
	//
	else{
		pre_time = cur_time;
		PROCESS_ONCE = false;
		return false;
	}
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
	float Vrx = cur_vel * cos(cur_ang);//*delta_time);
	float Vry = cur_vel * sin(cur_ang);//*delta_time);
	// 目標速度(探査対象)
	//cmd_dAng は水平右をx軸, 正面をy軸とする
	float dVrx_c = (cmd_dV+cur_vel) * cos(cmd_ang);//*delta_time);//(cmd_dV+cur_vel) * cos(cmd_ang);
	float dVry_c = (cmd_dV+cur_vel) * sin(cmd_ang);//*delta_time);//(cmd_dV+cur_vel) * sin(cmd_ang);
	//障害物
	// 位置
	float Xox = gpRef.x;
	float Xoy = gpRef.y;
	// 速度
	float Vox = twistRef.linear.x + Vrx;
	float Voy = twistRef.linear.y + Vry;
	// 番号
	int index = indexRef;
	//交差位置
	float Vcx = Vox - dVrx_c;
	float Vcy = Voy - dVry_c;
	crsPt_x0.vx = Vcx;
	crsPt_x0.vy = Vcy;
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
void obstacleAvoidance::getCrossPoints(crossPoint& crsPt_x0, crossPoint& crsPt_y0, int& indexRef, const local_navigation::ClassificationElement& clst_data, geometry_msgs::Twist& twistRef, float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang){
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
	float Vrx = cur_vel * cos(cur_ang);//*delta_time);
	float Vry = cur_vel * sin(cur_ang);//*delta_time);
	// 目標速度(探査対象)
	//cmd_dAng は水平右をx軸, 正面をy軸とする
	float dVrx_c = (cmd_dV+cur_vel) * cos(cmd_ang);//*delta_time);//(cmd_dV+cur_vel) * cos(cmd_ang);
	float dVry_c = (cmd_dV+cur_vel) * sin(cmd_ang);//*delta_time);//(cmd_dV+cur_vel) * sin(cmd_ang);
	//障害物
	// 位置
	float Xox = clst_data.gc.x;
	float Xoy = clst_data.gc.y;
	// 速度
	float Vox = twistRef.linear.x + Vrx;
	float Voy = twistRef.linear.y + Vry;
	// 番号
	int index = indexRef;
	//交差位置
	float Vcx = Vox - dVrx_c;
	float Vcy = Voy - dVry_c;
	crsPt_x0.vx = Vcx;
	crsPt_x0.vy = Vcy;
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
		getNearestDistance(crsPt_x0, clst_data);
	}
	if(checkSafetyObstacle(crsPt_y0.t, angle,Xox,Xoy)){
		crsPt_y0.safe = true;
	}
	else{
		crsPt_y0.safe = false;
		getNearestDistance(crsPt_y0, clst_data);
	}
}
double obstacleAvoidance::getNearestDistance(crossPoint& crsPt, const local_navigation::ClassificationElement& clst_data){
	//点群の中で最も近い距離を交差位置.距離にセット
	for(int i=0; i<clst_data.pt.size();i++){
		double x = crsPt.x - (clst_data.pt[i].x - clst_data.gc.x);
		double y = crsPt.y - (clst_data.pt[i].y - clst_data.gc.y);
		double dis = std::sqrt(std::pow(x,2.0)+std::pow(y,2.0));
		if(crsPt.dis > dis){
			crsPt.dis = dis;
		}
	}
}
//障害物データ群に対する各x,y座標の交差位置を算出(交差位置の配列)
// 相対速度を使用する
void obstacleAvoidance::crossPointsDetect(std::vector<crossPoint>& crsPts, float& cur_vel_temp, float& cur_angle_temp, float& cmd_dV, float& cmd_dAng){
	crsPts.resize((int)clstr.data.size()*2);
	for(int k=0; k<clstr.data.size(); k++){
		if(clstr.twist[k].linear.x ==0 || clstr.twist[k].linear.y == 0){
			continue;
		}
	getCrossPoints(crsPts[k*2], crsPts[k*2+1], k, clstr.data[k].gc, clstr.twist[k],cur_vel_temp,  cur_angle_temp, cmd_dV,cmd_dAng);
	//getCrossPoint(cp_num, crsPts, k, clstr.data[k].gc, clstr.twist[k],cur_vel_temp, cmd_dV,cmd_dAng);
	}
}
//--コスト関数
double obstacleAvoidance::costCrossPoint(crossPoint& crsPt, float eta_cp){
	// eta_cp : cost算出用パラメータ
	//コスト関数: あとで変更する予定
	// return (-(pow(crsPt.dis/crsPt.t/eta_cp,2.0)));
	// return ((pow(1/crsPt.dis/eta_cp,2.0)/crsPt.t));
	if(safe_range <= crsPt.dis){
		return (0);
	}
	return ((pow(1/crsPt.dis/eta_cp,2.0)/(crsPt.t+1)));
}
double obstacleAvoidance::getDeltaVelCost(float& cmd_dV_temp, float& eta_vel_temp,float& cur_vel_temp){
	// return ((pow((cmd_dV_temp+cur_vel_temp - default_speed)/eta_vel_temp,2.0)));
	// return (-cmd_dV_temp/eta_vel_temp);
	return (std::abs(cmd_dV_temp+cur_vel_temp - default_speed)/eta_vel_temp);
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
	float dV=0;
	if(SEARCH_ONLY_ANGLE){
		dV = debugCmd_vel;
		vfh_angleSearch(tagAng,cur_vel, dV);
		tagVel = default_speed;
	}
	else{
		search_vel_ang(tagAng,cur_vel, dV);
		tagVel = cur_vel + dV;
	}
	//
	prev_tagAng = tagAng;
}
// 最適角度探索
double obstacleAvoidance::vfh_angleSearch(float& target_angle_temp, float& cur_vel_temp, float& cmd_dV){//return cost
	
	std::vector<bool> histgram_bi;
    vfh_c.get_histgram_bi(histgram_bi);
	//
	// goal_x=debugGoalPosX;
	// goal_y=debugGoalPosY;
	//
	float goalAng = atan2(goal_y, goal_x)*180/M_PI;
	//weight正規化
    double sum_weight = k_cp + k_curAngle + k_prevAngle + k_cp;
    k_g/=sum_weight;
    k_curAngle/=sum_weight;
    k_prevAngle/=sum_weight;
    k_cp/=sum_weight;
    //コスト算出
    double min_cost = MAX_COST;
    int min_num = -1;
	std::vector<crossPoint> min_cost_crsPts;
	//
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double ang = vfh_c.transform_numToAngle(i);//deg
        float cmd_ang = ang*M_PI/180;
		double cur_ang = 90;//正面を向いているため

		//
		float cmd_dAng_rad = cmd_ang;//(cmd_ang-cur_ang);
		float prev_tagAng_rad = prev_tagAng*M_PI/180;
		//交差位置算出
        std::vector<crossPoint> crsPts;
		//現在の進行方向はprev_tagAngの方向であると仮定
		// ROS_INFO_STREAM(i<<":"<<cur_vel_temp<<", "<<prev_tagAng_rad<<", "<<cmd_dV<<", "<<cmd_dAng_rad);
		crossPointsDetect(crsPts, cur_vel_temp, prev_tagAng_rad, cmd_dV, cmd_dAng_rad);
		// std::cout<<i<<":"<<std::endl;
		// for(int k = 0; k < crsPts.size(); k++){
		// 	ROS_INFO_STREAM(k<<" -- "<<crsPts[k].x <<","<<crsPts[k].y<<","<<crsPts[k].t<<","<<crsPts[k].safe<<","<<crsPts[k].vx<<","<<crsPts[k].vy);
		// }
		// ROS_INFO_STREAM(i<<":"<<"for crsPts size:" << crsPts.size());		
		//コスト算出
        double goalCost = vfh_c.cost_goal_angle_deg(ang, goalAng);
        double angCost = vfh_c.cost_current_angle_deg(ang, cur_ang);
        double prevAngCost = vfh_c.cost_prev_select_angle_deg(ang, prev_tagAng);
        double crossCost = getCrossPointCost(crsPts,eta_cp);
        double cost = k_g * goalCost + k_curAngle * angCost + k_prevAngle * prevAngCost + k_cp*crossCost;
        // std::cout<<i<<":"<<cost<<std::endl;
        // std::cout<<i<<":"<<goalCost<<","<<angCost<<","<<prevAngCost<<","<<crossCost<<std::endl;
		if(min_cost > cost){
            min_cost = cost;
            min_num = i;
			//デバッグ用
			min_cost_crsPts = crsPts;
			// ROS_INFO_STREAM("for min crsPts size:" << min_cost_crsPts.size());
        }
    }
	//目標角度
	target_angle_temp = vfh_c.transform_numToAngle(min_num);
	if(min_cost == MAX_COST){
		ROS_INFO_STREAM("Not found space");
	}
	//デバッグ関数に交差位値情報を渡す
	if(display_output){
		ROS_INFO_STREAM("min crsPts size:" << min_cost_crsPts.size());
		ROS_INFO_STREAM("cur_vel, dV" << cur_vel_temp <<","<< cmd_dV);
		showOutPut(min_cost_crsPts, cur_vel_temp + cmd_dV, min_num);
	}
	return min_cost;
}
// 最適角度探索(デバッグ処理なし)
double obstacleAvoidance::vfh_angleSearch_nondeb(float& target_angle_temp, float& cur_vel_temp, float& cmd_dV ,std::vector<crossPoint>& min_cost_crsPts_temp ){//return cost
	
	std::vector<bool> histgram_bi;
    vfh_c.get_histgram_bi(histgram_bi);
	//
	// goal_x=debugGoalPosX;
	// goal_y=debugGoalPosY;
	//
	float goalAng = atan2(goal_y, goal_x)*180/M_PI;
	//weight正規化
    double sum_weight = k_cp + k_curAngle + k_prevAngle + k_cp + k_vel;
    k_g/=sum_weight;
    k_curAngle/=sum_weight;
    k_prevAngle/=sum_weight;
    k_cp/=sum_weight;
    k_vel/=sum_weight;
    //コスト算出
    double min_cost = MAX_COST;
    int min_num = -1;
	std::vector<crossPoint> min_cost_crsPts;
	//
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double ang = vfh_c.transform_numToAngle(i);//deg
        float cmd_ang = ang*M_PI/180;
		double cur_ang = 90;//正面を向いているため

		//
		float cmd_dAng_rad = cmd_ang;//(cmd_ang-cur_ang);
		float prev_tagAng_rad = prev_tagAng*M_PI/180;
		//交差位置算出
        std::vector<crossPoint> crsPts;
		//現在の進行方向はprev_tagAngの方向であると仮定
		// ROS_INFO_STREAM(i<<":"<<cur_vel_temp<<", "<<prev_tagAng_rad<<", "<<cmd_dV<<", "<<cmd_dAng_rad);
		crossPointsDetect(crsPts, cur_vel_temp, prev_tagAng_rad, cmd_dV, cmd_dAng_rad);
		// std::cout<<i<<":"<<std::endl;
		// for(int k = 0; k < crsPts.size(); k++){
		// 	ROS_INFO_STREAM(k<<" -- "<<crsPts[k].x <<","<<crsPts[k].y<<","<<crsPts[k].t<<","<<crsPts[k].safe<<","<<crsPts[k].vx<<","<<crsPts[k].vy);
		// }
		// ROS_INFO_STREAM(i<<":"<<"for crsPts size:" << crsPts.size());		
		//コスト算出
        double goalCost = vfh_c.cost_goal_angle_deg(ang, goalAng);
        double angCost = vfh_c.cost_current_angle_deg(ang, cur_ang);
        double prevAngCost = vfh_c.cost_prev_select_angle_deg(ang, prev_tagAng);
        double crossCost = getCrossPointCost(crsPts,eta_cp);
        double velCost = getDeltaVelCost(cmd_dV,eta_vel,cur_vel_temp);
        double cost = k_g * goalCost + k_curAngle * angCost + k_prevAngle * prevAngCost + k_cp*crossCost + k_vel*velCost;
        // std::cout<<i<<":"<<cost<<std::endl;
        // std::cout<<i<<":"<<goalCost<<","<<angCost<<","<<prevAngCost<<","<<crossCost<<std::endl;
		if(min_cost > cost){
            min_cost = cost;
            min_num = i;
			//デバッグ用
			min_cost_crsPts = crsPts;
			// ROS_INFO_STREAM("for min crsPts size:" << min_cost_crsPts.size());
        }
    }
	//目標角度
	target_angle_temp = vfh_c.transform_numToAngle(min_num);
	if(min_cost == 1){
		ROS_INFO_STREAM("Not found space");
	}
	min_cost_crsPts_temp.clear();
	min_cost_crsPts_temp = min_cost_crsPts;
	return min_cost;
}
void obstacleAvoidance::search_vel_ang(float& target_angle, float& cur_vel_temp, float& cmd_dV){
	//探索回数(未使用)
	int count = 0;
	const int countThreshold =10;
	//最適化対象: 評価値
	double evalVal = MAX_COST;
	//
	std::vector<crossPoint> min_cost_crsPts;
	//--全探査
	float dV = 0;//探査対象dv
	float dAng = 0;//探査対象dAng
	for(float search_dV = -dV_range; search_dV <= dV_range; search_dV +=dV_div){
		if(search_dV + cur_vel_temp > max_speed || search_dV + cur_vel_temp <= 0.0){//
			continue;
		}
		float searching_angle;//探査対象angle
		std::vector<crossPoint> min_cost_crsPts_temp;
		//評価
		double evalTemp = vfh_angleSearch_nondeb(searching_angle, cur_vel_temp, search_dV,min_cost_crsPts_temp);
		if(evalTemp < evalVal){
			evalVal = evalTemp;
			dV = search_dV;
			dAng = searching_angle;
			min_cost_crsPts.clear();
			min_cost_crsPts = min_cost_crsPts_temp;
		}
	}
	cmd_dV = dV ;
	target_angle = dAng;
	//デバッグ関数に交差位値情報を渡す
	if(display_output){
		ROS_INFO_STREAM("min crsPts size:" << min_cost_crsPts.size());
		ROS_INFO_STREAM("cur_vel, dV :" << cur_vel_temp <<","<< cmd_dV);
		ROS_INFO_STREAM("target_angle :" << target_angle);
		int min_num = vfh_c.transform_angle_RobotToNum(target_angle);
		showOutPut(min_cost_crsPts, cur_vel_temp + cmd_dV, min_num);
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
	vfh_c.create_binary_histgram(robotRadius,marginRadius);
}
