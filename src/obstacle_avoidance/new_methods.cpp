
#include<local_navigation/obstacleAvoidance.h>

//subscribe
void obstacleAvoidance::cluster_callback(const local_navigation::ClassificationVelocityData::ConstPtr& msg)
{
//	ROS_INFO("cluster_callback");
    //データをコピー
	clstr = *msg;
	//move manage method
	RECEIVED_CLUSTER = true;
	manage();
}
void obstacleAvoidance::robotOdom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
//	ROS_INFO("robotOdom_callback");
    //データをコピー
	robotOdom = *msg;
	RECEIVED_ROBOT_ODOM = true;
	//move manage method
	manage();
}
void obstacleAvoidance::robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg)
{
//	ROS_INFO("robotEncoder_callback");
    //データをコピー
	robotEncoder = *msg;
	cur_vel = ((-robotEncoder.vel.r) + robotEncoder.vel.l)/2;
	cur_angVel = ((-robotEncoder.vel.r) - robotEncoder.vel.l)/(2 * d);
	RECEIVED_ROBOT_ENCODAR = true;
	//ROS_INFO("cur_vel=%f",cur_vel);
	//move manage method
	manage();
}
void obstacleAvoidance::goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
//	ROS_INFO("goalOdom_callback");
    //データをコピー
	goalOdom = *msg;
	RECEIVED_GOAL_ODOM = true;
	//move manage method
	manage();
}

void obstacleAvoidance::manage(){
	//ROS_INFO("into manage");
	if(N != 1){
		if(data_check()){
			data_check_reset();
			get_time();
			if(!culc_delta_time()){
				pre_robotOdom = robotOdom;
				return;
			}
			culc_delta_robotOdom();
			//ROS_INFO("two");
			float tagVel, tagAng;
			potential(tagVel, tagAng);
			//命令速度生成
			//if(N=0){		
			geometry_msgs::Twist cmd = controler(tagVel, tagAng);
			geometry_msgs::Twist cmd_movmean = move_mean(cmd);
			publishData(cmd_movmean);
			//}
			//ROS_INFO("three");
			publish_deltaRobotOdom();		
		}

	}
	clstr.data.clear();
}

void obstacleAvoidance::culc_delta_robotOdom(){
	deltaRobotOdom.pose.pose.position.x = robotOdom.pose.pose.position.x - pre_robotOdom.pose.pose.position.x;
	deltaRobotOdom.pose.pose.position.y = robotOdom.pose.pose.position.y - pre_robotOdom.pose.pose.position.y;
	deltaRobotOdom.pose.pose.position.z = robotOdom.pose.pose.position.z - pre_robotOdom.pose.pose.position.z;
	deltaRobotOdom.twist.twist.linear.x = (robotOdom.pose.pose.position.x - pre_robotOdom.pose.pose.position.x ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	deltaRobotOdom.twist.twist.linear.y = (robotOdom.pose.pose.position.y - pre_robotOdom.pose.pose.position.y ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	deltaRobotOdom.twist.twist.linear.z = (robotOdom.pose.pose.position.z - pre_robotOdom.pose.pose.position.z ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	// deltaRobotOdom = robotOdom - pre_robotOdom;
	//回転
	tf::Quaternion quat;
	double r_cur,p_cur,y_cur;
	quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(r_cur,p_cur,y_cur);
	double r_pre,p_pre,y_pre;
	quaternionMsgToTF(pre_robotOdom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(r_pre,p_pre,y_pre);
	double delta_theta = y_cur - y_pre;
	double omega = delta_theta/( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	//
	tf::Quaternion quatTheta=tf::createQuaternionFromYaw(delta_theta);//debugRobotYaw-M_PI_2);
	quaternionTFToMsg(quatTheta, deltaRobotOdom.pose.pose.orientation);
	
	deltaRobotOdom.twist.twist.angular.x = 0;
	deltaRobotOdom.twist.twist.angular.y = 0;
	deltaRobotOdom.twist.twist.angular.z = omega;
	
	pre_robotOdom = robotOdom;
	pre_omega = y_cur;
	ROS_INFO("PRE omega = %f ",pre_omega/M_PI*180);

}

void obstacleAvoidance::potential(float& tagVel, float& tagAng){

//	ROS_ERROR("start!");

	K_kakudo = 1;

	//ゴール位置
	double GGX = 0,GGY = 6;
	double MAX_S = 0.2;
	double K = 1.1;
	double D = 1;

	//ロボット初期位置
	double rx = 0;
	double ry = 0;

	//切り替え範囲
	double R = 3;
	double R_den = 3;
	double delta = 0.2;
	double c = 1/5; 
	double de = 1;
	double Length_max = 2*GGX + 2*GGY;
	double fd = 0.05;

	//竹下さんの修論18ページ (3.17)式
	double F1 = POTENTIAL_F1;
	double F2 = POTENTIAL_F2;

	robotOdom.pose.pose.position.y = robotOdom.pose.pose.position.y + 0.035;
	
	double A = GGX + robotOdom.pose.pose.position.y;
	double B = GGY - robotOdom.pose.pose.position.x;
	double AA = A*A;
	double BB = B*B;
	double AB = sqrt(AA + BB);
//	ROS_INFO("AB :%f",AB);
	
	if(AB < 0.3){
		tagVel = 0;
		tagAng = 0;
		geometry_msgs::Twist cmd = controler(tagVel, tagAng);
		publishData(cmd);
		ROS_ERROR("X,Y :%f %f",-robotOdom.pose.pose.position.y,robotOdom.pose.pose.position.x);
		ROS_ERROR("AB :%f",AB);
		exit(0);
	}


	if(fabs(deltaRobotOdom.pose.pose.position.x) > 1 ||fabs(deltaRobotOdom.pose.pose.position.y)>1){
		tagVel = 0;
		tagAng = 0;
		geometry_msgs::Twist cmd = controler(tagVel, tagAng);
		publishData(cmd);
		ROS_WARN("X,Y :%f %f",-robotOdom.pose.pose.position.y,robotOdom.pose.pose.position.x);
		exit(0);
	}

	double U1 = 0,U2 = 0,U3 = 0,U4 = 0,U5 = 0,U6 = 0,UX = 0,UY = 0,U_all = 0;

	//ゴールへのポテンシャル
	//A
    	//U1 = (2*A)/(AB);
    	//U2 = (2*B)/(AB);
	//B
	U1 = 2*A;
	U2 = 2*B;
	//C
	//U1 = A/pow(AA+BB,3/2);
	//U2 = B/pow(AA+BB,3/2);

	if(AB < 1){
	    	U1 = (2*A)/(AB);
	    	U2 = (2*B)/(AB);
	}

	int culs = clstr.size.data;
	// ROS_ERROR("cls = %d",culs);

	struct tm stm;
	time_t tim;
	char s[100];

	tim = ros::Time::now().sec;
	strftime(s,sizeof(s),"%Y-%m-%d-%H-%M-%S.txt",localtime(&tim));

	//int CN = clstr.size.data;
	double x_c,y_c,XY_ookisa,XO,YO,inoutH;
	double omega = pre_omega;
	double gx1,gy1,nx1,ny1,gx2,gy2,nx2,ny2;
	double angle = 0,GGX1,GGY1,GGX2,GGY2,NNX1,NNY1,NNX2,NNY2;
	double c1,c2,c3,c4,d1,d2,d3,d4,e1,e2,e3,e4;

	for(int k =0; k < culs; k++){//クラスタ数

		int CN = clstr.data[k].size.data;
		double Mini = 100000;
		double most_co = 0,LO = 0;
		LO = 0;

		for(int kn =0; kn < CN; kn++){

			most_co = sqrt(pow(clstr.data[k].pt[kn].x,2)+pow(clstr.data[k].pt[kn].y,2));

			if(Mini > most_co){
				
				x_c = clstr.data[k].pt[kn].x;
				y_c = clstr.data[k].pt[kn].y;
				LO = sqrt(x_c*x_c + y_c*y_c);
									
			}
		}

		if(x_c == 0.000000){

			//ROS_WARN("x_c :%f",x_c);			
			x_c = clstr.data[k].gc.x;
			y_c = clstr.data[k].gc.y;
		} 
		

		if(LO < 0.0001){

			x_c = clstr.data[k].gc.x;
			y_c = clstr.data[k].gc.y;

		}

		if((x_c < 0.00001) && (x_c > -0.00001)){

			if(y_c < 0.0000001){

				//ROS_INFO("skip!");
				//ROS_WARN("x_c :%f",x_c);
				continue;
			}

		}

		// ROS_INFO("x_c :%f y_c :%f",x_c,y_c);
		XY_ookisa = sqrt(x_c*x_c + y_c*y_c);
		
		std::vector<std::vector<double>> vecO(5);
		vecO[0] = {cos(omega)*x_c - sin(omega)*y_c, sin(omega)*x_c + cos(omega)*y_c};
		vecO[1] = {vecO[0][0] - F1, vecO[0][1]};
		vecO[2] = {vecO[0][0] + F1, vecO[0][1]};
		vecO[3] = {vecO[0][0], vecO[0][1] - F2};
		vecO[4] = {vecO[0][0], vecO[0][1] + F2};

		for(int i = 0; i < vecO.size(); i++)
		{
			XO = vecO[i][0];
			YO = vecO[i][1];

			double vxr_o = clstr.twist[k].linear.y; //障害物の速度 LRF
			double vyr_o = -clstr.twist[k].linear.x; //障害物の速度 LRF
			double VX = cos(omega)*vxr_o - sin(omega)*vyr_o;//障害物の速度 World　くりお
			double VY = sin(omega)*vxr_o + cos(omega)*vyr_o;//障害物の速度 World 石川

			if(AB < 2){
				if(AB < XY_ookisa){
					ROS_WARN("skip!!!");
					continue;
				}
			}

			if(R_den > XY_ookisa){

				if((y_c < 0) && (XY_ookisa<0.8)){
					continue;
				}

				if(R > XY_ookisa){

					gx1 = cos(omega)*(GGX+delta) - sin(omega)*(GGY+delta);
					gy1 = sin(omega)*(GGX+delta) + cos(omega)*(GGY+delta);
					gx2 = cos(omega)*(GGX-delta) - sin(omega)*(GGY-delta);
					gy2 = sin(omega)*(GGX-delta) + cos(omega)*(GGY-delta);

					nx1 = cos(omega)*(-robotOdom.pose.pose.position.y+delta) - sin(omega)*(robotOdom.pose.pose.position.x+ delta);
					ny1 = sin(omega)*(-robotOdom.pose.pose.position.y+delta) + cos(omega)*(robotOdom.pose.pose.position.x+ delta);
					nx2 = cos(omega)*(-robotOdom.pose.pose.position.y-delta) - sin(omega)*(robotOdom.pose.pose.position.x - delta);
					ny2 = sin(omega)*(-robotOdom.pose.pose.position.y-delta) + cos(omega)*(robotOdom.pose.pose.position.x - delta);
		
					GGX1 = gx1 - XO;//start&goal X
					GGY1 = gy1 - YO;//start&goal Y
					GGX2 = gx2 - XO;//2nd X
					GGY2 = gy2 - YO;//2nd Y
					NNX1 = nx1 - XO;//4th X
					NNY1 = ny1 - YO;//4th Y
					NNX2 = nx2 - XO;//3rd X
					NNY2 = ny2 - YO;//3rd Y

					c1 = GGX1*GGX2 + GGY1*GGY2;
					d1 = GGX1*GGY2 - GGY1*GGX2;
					e1 = atan2(d1,c1);

					c2 = GGX2*NNX2 + GGY2*NNY2;
					d2 = GGX2*NNY2 - GGY2*NNX2;
					e2 = atan2(d2,c2);

					c3 = NNX2*NNX1 + NNY2*NNY1;
					d3 = NNX2*NNY1 - NNY2*NNX1;
					e3 = atan2(d3,c3);

					c4 = NNX1*GGX1 + NNY1*GGY1;
					d4 = NNX1*GGY1 - NNY1*GGX1;
					e4 = atan2(d4,c4);

					angle = e1 + e2 + e3 + e4;


					if( fabs(2*M_PI-fabs(angle)) < 0.005 ){
						
	//					if(x_c > 0.1){
							U5 = U5 - XO/(XO*XO + YO*YO);
							U6 = U6 + YO/(XO*XO + YO*YO);
	//					}else{
	//						U5 = U5 + YO/(XO*XO + YO*YO);
	//						U6 = U6 - XO/(XO*XO + YO*YO);
	//					}
						continue;

					}

				}

				U3 = U3 - XO/pow(XO*XO + YO*YO,3/2);
				U4 = U4 - YO/pow(XO*XO + YO*YO,3/2);
	//			ROS_INFO("============");
	//	//		ROS_WARN("x_c:%f",x_c);
	//	//		ROS_INFO("y_c:%f",y_c);
	//			ROS_WARN("XO:%f",XO);
	//			ROS_INFO("YO:%f",YO);

			}
		}

		// if(XO > robotOdom.pose.pose.position.x)
		// {
		// 	XO = XO - F1;
		// }
		// else
		// {
		// 	XO = XO + F2;
		// }

		// if(YO > robotOdom.pose.pose.position.y)
		// {
		// 	YO = YO - F1;
		// }
		// else
		// {
		// 	YO = YO + F2;
		// }
		
		//ROS_ERROR("x :%f y :%f",XO,YO);
//		ROS_INFO("============");
////		ROS_WARN("x_c:%f",x_c);
////		ROS_INFO("y_c:%f",y_c);
//		ROS_WARN("XO:%f",XO);
//		ROS_INFO("YO:%f",YO);
//		ROS_INFO("============");
		

	}

	//ROS_INFO("result!");

	UX = U1 + D*U3 + K*U5;
	UY = U2 + D*U4 + K*U6;

//	ROS_ERROR("U3:%f",U3);
//	ROS_ERROR("U4:%f",U4);
//	ROS_WARN("Ux:%f",UX);
//	ROS_INFO("Uy:%f",UY);
	//ROS_WARN("U3:%f & U4:%f",U3,U4);

	U_all = sqrt(UX*UX+UY*UY);


	
	if(U_all > Length_max){
		if(UY > 0){
			tagVel = MAX_S;
			tagAng = atan2(UY,UX)  - M_PI/2;
		}else{
			tagVel = 0.0001;
			if(UX < 0){
				tagAng = (atan2(UY,UX) + 2*M_PI  - M_PI/2);//MSCS2023:0.2
				//K_kakudo = 0.5;
			}else{
				tagAng = (atan2(UY,UX)  - M_PI/2);
				//K_kakudo = 0.5;
			}
		}
		

	}else{
		double at1 = U_all/Length_max;

		if(UY > 0){
			tagVel = at1*MAX_S;
			tagAng = atan2(UY,UX)  - M_PI/2;
			//tagAng = atan2(UY,UX)  - M_PI/2 + 0.002 *(1/at1);
			//tagAng = atan2(UY,UX)  - M_PI/2;
		}else{
			tagVel = 0.0001;
			if(UX < 0){
				tagAng = (atan2(UY,UX) + 2*M_PI  - M_PI/2);
				//K_kakudo = 0.5;
			}else{
				tagAng = (atan2(UY,UX)  - M_PI/2);
				//K_kakudo = 0.5;
			}
		}
		
	}

 
	std::ofstream ofs(s,std::ios::app);
	//ROS_WARN("tagAng : %f",tagAng); 

	if(!ofs){
		ROS_ERROR("deteniyo!");
	}else{
		ofs << s << std::endl;
		ofs <<"X = " << -robotOdom.pose.pose.position.y << std::endl;
		ofs <<"Y = " << robotOdom.pose.pose.position.x << std::endl;
		ofs <<"U = " << U_all << std::endl;
		ofs <<"U_x = " << UX << std::endl;
		ofs <<"U_y = " << UY << std::endl;
		ofs <<"U1 = " << U1 << std::endl;
		ofs <<"U2 = " << U2 << std::endl;
		ofs <<"V = " << tagVel << std::endl;
		ofs <<"rad = " << tagAng << std::endl;
	}
	//ofs.close();

	prev_tagAng = tagAng;
	clstr.data.clear();
//	tagVel = 0;
//	tagAng = 0;
	//ROS_ERROR("finish!");

	//ROS_INFO("x :%f",clstr.data[0].gc.x);

}

std::vector<double> shiftLeft(const std::vector<double>& vec, int shiftAmount) {
    std::vector<double> result(vec.size(), 0); // ゼロで初期化された同じサイズのベクターを作成

    for (size_t i = 0; i < vec.size(); ++i) {
        size_t newPos = (i + shiftAmount) % vec.size(); // 新しい位置を計算
        result[i] = vec[newPos]; // 元のベクターの要素を新しい位置にコピー
    }

    return result;
}

geometry_msgs::Twist obstacleAvoidance::move_mean(geometry_msgs::Twist cmd_input){
	int window_num = MOVE_MEAN_WINDOW_NUM;
	static size_t cnt = 0;
	static std::vector<double> linear_x(window_num);
	static std::vector<double> angular_z(window_num);
	linear_x = shiftLeft(linear_x,1);
	angular_z = shiftLeft(angular_z,1);
	linear_x.back() = cmd_input.linear.x;
	angular_z.back() = cmd_input.angular.z;
	geometry_msgs::Twist cmd_output;
	cmd_output.linear.x = accumulate( linear_x.begin(), linear_x.end(),0.0)/linear_x.size();
	cmd_output.angular.z = accumulate( angular_z.begin(), angular_z.end(),0.0)/angular_z.size();
	return cmd_output;
}

geometry_msgs::Twist obstacleAvoidance::controler(float& tagVel, float& tagAng){
	//
	if(tagVel ==0){
		geometry_msgs::Twist twist;
		twist.linear.x =0; 
		twist.linear.y =0; 
		twist.linear.z =0;
		twist.angular.x =0;
		twist.angular.y =0;
		twist.angular.z =0;
		return twist;
	}	
	//p制御
	double cur_ang = M_PI/2;//正面を向いているため
	float gainP = CONTROLLER_GAIN_P;
	//float tagAngVel = (M_PI*(tagAng-cur_ang)/180)*gainP;
	
	//A
	//float tagAngVel = (tagAng - cur_ang)*gainP;
	//B
	//float tagAngVel = (tagAng - robotOdom.twist.twist.angular.z)*gainP;

	tf::Quaternion quat;
	double r_cur,p_cur,y_cur;
	quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(r_cur,p_cur,y_cur);

	float tagAngVel = (tagAng - y_cur)*gainP;
	//ROS_ERROR("tagAngVel :%f",tagAngVel);

	//C
	//float tagAngVel = 2*sin(tagAng - robotOdom.twist.twist.angular.z);

	geometry_msgs::Twist twist;
	twist.linear.x =tagVel; 
	twist.linear.y =0; 
	twist.linear.z =0;
	twist.angular.x =0;
	twist.angular.y =0;
	twist.angular.z =tagAngVel;
	pre_rad = tagAng;

	return twist;
}

void obstacleAvoidance::publish_deltaRobotOdom(){
    //deltaRobotOdom
    pubDebOdom.publish(deltaRobotOdom);
}

void obstacleAvoidance::publishData(geometry_msgs::Twist& pubData){
    pub.publish(pubData);
}

void obstacleAvoidance::data_check_reset(){
	RECEIVED_CLUSTER = false;
	RECEIVED_ROBOT_ODOM = false;
	RECEIVED_ROBOT_ENCODAR = false;
}
bool obstacleAvoidance::data_check(){
//	ROS_INFO_STREAM(
//		(RECEIVED_CLUSTER ? "RECEIVED_CLUSTER" : "NOT CLUSTER") <<","<<
//		(RECEIVED_GOAL_ODOM ? "RECEIVED_GOAL_ODOM" : "NOT GOAL_ODOM") <<","<<
//		(RECEIVED_ROBOT_ODOM ? "RECEIVED_ROBOT_ODOM" : "NOT ROBOT_ODOM") <<","<<
//		(RECEIVED_ROBOT_ENCODAR ? "RECEIVED_ROBOT_ENCODAR" : "NOT ROBOT_ENCODAR") 
//	);
	return(
	    RECEIVED_CLUSTER&& RECEIVED_GOAL_ODOM&& RECEIVED_ROBOT_ODOM 
	    && RECEIVED_ROBOT_ENCODAR
	);
}
void obstacleAvoidance::get_time(){
	cur_time = ros::Time::now();
}
bool obstacleAvoidance::culc_delta_time(){
	if(!PROCESS_ONCE){
		delta_time_ros = cur_time - pre_time;
		delta_time = delta_time_ros.toSec();
		pre_time = cur_time;
		//
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

//	for(int k =0; k < culs; k++){クラスタ数

//		グローバル座標系での速度を算出
//		double vxr_r = cur_vel * sin( cur_angVel*delta_time*M_PI/180+M_PI_2);ロボットの速度
//		double vyr_r = -cur_vel * cos( cur_angVel*delta_time*M_PI/180+M_PI_2);
//		double xr_o = clstr.data[k].gc.x;
//		double yr_o = clstr.data[k].gc.y;

//		ROS_WARN("clstr.data[k].gc.x & gc.y :%f %f",clstr.data[k].gc.x,clstr.data[k].gc.y);
//		std::ofstream ofs(s,std::ios::app);
//		ofs << s << std::endl;
//		ofs <<"Xiti = " << clstr.data[k].gc.x << std::endl;
//		ofs <<"Yiti = " << clstr.data[k].gc.y << std::endl;
//		double vxr_o = clstr.twist[k].linear.y; 障害物の速度
//		double vyr_o = -clstr.twist[k].linear.x; 障害物の速度

//		double yaw = std::atan2(robotOdom.pose.pose.position.x,-robotOdom.pose.pose.position.y); 初期状態と現在の角度
//		v rotation
//		double vx_og = -cur_angVel*sin(yaw)*xr_o + cos(yaw)*vxr_o
//						- cur_angVel*cos(yaw)*yr_o - sin(yaw)*vyr_o;
//		
//		double vy_og = cur_angVel*cos(yaw)*xr_o + sin(yaw)*vxr_o 
//						- cur_angVel*sin(yaw)*yr_o + cos(yaw)*vyr_o;
//								
//		v linear
//		double vxr_g = vxr_o * cos(yaw) - vyr_o * sin(yaw);
//		double vyr_g = vxr_o * sin(yaw) + vyr_o * cos(yaw);

//		int count = clstr.data[k].size.data;		
//		int CP=0,CPMIN = 100000000,CPCO=0;


//		for (int i = 0; i < count; i++){

//			CP = sqrt(clstr.data[k].pt[i].x*clstr.data[k].pt[i].x + clstr.data[k].pt[i].y*clstr.data[k].pt[i].y);
//			
//			if(CP < CPMIN){
//				CPMIN = CP;
//				CPCO = i; 
//			}

//		}


//		double Xpt = clstr.data[k].gc.x;
//		double Ypt = clstr.data[k].gc.y;
//		double Xpt = clstr.data[k].gc.x*cos(-robotOdom.twist.twist.angular.z) + clstr.data[k].gc.y*sin(-robotOdom.twist.twist.angular.z);
//		double Ypt = clstr.data[k].gc.x*-sin(-robotOdom.twist.twist.angular.z) + clstr.data[k].gc.y*cos(-robotOdom.twist.twist.angular.z);

//		double XYpt = pow(Xpt*Xpt+Ypt*Ypt,3/2);
//		double L = 5;
//		double robot1[4] = {(robotOdom.pose.pose.position.x-delta)-robotOdom.pose.pose.position.x,robotOdom.pose.pose.position.y - robotOdom.pose.pose.position.y,
//						(robotOdom.pose.pose.position.x+delta)-robotOdom.pose.pose.position.x,robotOdom.pose.pose.position.y - robotOdom.pose.pose.position.y};
//		double goal1[4] = {(GGX - delta)-GGX,GGY - GGY,(GGX+delta)-GGX,GGY - GGY};
//	
//		double theta = M_PI - atan2(B,A);
//		double angle = 0;
//		if(XYpt<R){
//				double kaiten1[4] = {cos(theta),-1*sin(theta),sin(theta),cos(theta)};
//				double zahyou[5][2];

//				zahyou[0][0] = kaiten1[0]*robot1[0]+kaiten1[1]*robot1[1];
//				zahyou[0][1] = kaiten1[2]*robot1[0]+kaiten1[3]*robot1[1];
//				zahyou[1][0] = kaiten1[0]*robot1[2]+kaiten1[1]*robot1[3];
//				zahyou[1][1] = kaiten1[2]*robot1[2]+kaiten1[3]*robot1[3];
//				zahyou[2][0] = kaiten1[0]*goal1[0]+kaiten1[1]*goal1[1];
//				zahyou[2][1] = kaiten1[2]*goal1[0]+kaiten1[3]*goal1[1];
//				zahyou[3][0] = kaiten1[0]*goal1[2]+kaiten1[1]*goal1[3];
//				zahyou[3][1] = kaiten1[2]*goal1[2]+kaiten1[3]*goal1[3];
//				zahyou[4][0] = kaiten1[0]*robot1[0]+kaiten1[1]*robot1[1];
//				zahyou[4][1] = kaiten1[2]*robot1[0]+kaiten1[3]*robot1[1];

//				for(int i=0;i<5;i++){

//					double Ax,Ay,Bx,By;
//					double AxB,AvB;
//					Ax = zahyou[i][0] - Xpt;
//					Ay = zahyou[i][1] - Ypt;
//					Bx = zahyou[i+1][0] - Xpt;
//					By = zahyou[i+1][0] - Ypt;
//					AvB = Ax * Bx + Ay * By;
//					AxB = Ax * By - Ay * Bx;

//					angle = angle + atan2(AxB,AvB);

//				}

//				if( fabs(2*M_PI-fabs(angle)) < 0.001 ){

//					if(vyr_o < 0){
//						UX = UX - (Xpt)/(Xpt*Xpt+Ypt*Ypt);
//						UY = UY + (Ypt)/(Xpt*Xpt+Ypt*Ypt);
//					}
//					else{
//						UX = UX + (Xpt)/(Xpt*Xpt+Ypt*Ypt);
//						UY = UY - (Ypt)/(Xpt*Xpt+Ypt*Ypt);
//					}
//					continue;
//				}			
//			}

//		if(R_den > XYpt){

//			double UHX4 = -Xpt/XYpt;
//			double UHY4 = -Ypt/XYpt;
//			double UHX4 = -Xpt/(XYpt);
//			double UHY4 = -Ypt/(XYpt);
//			std::ofstream ofs(s,std::ios::app); 
//			ofs << s << std::endl;
//			ofs <<"X4 = " << UHX4 << std::endl;
//			ofs <<"Y4 = " << UHY4 << std::endl;

//			UX = UX + UHX4;
//			UY = UY + UHY4;
//			continue;
//		}

//	}

