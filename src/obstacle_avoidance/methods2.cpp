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
			publishData(cmd);
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
	pre_omega = robotOdom.pose.pose.orientation.z;
	//ROS_INFO("PRE omega=%f",pre_omega);

}

void obstacleAvoidance::potential(float& tagVel, float& tagAng){

//	ROS_ERROR("start!");

	//
	double dd = 180;
	double dt = M_PI/dd;

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

	int culs = clstr.size.data;
	//ROS_ERROR("cls = %d",culs);

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
	//
	double OBJX[culs],OBJY[culs];
	int OBJ_N=0;

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
				continue;
			}

		}
				

		OBJX[OBJ_N] = cos(omega)*x_c - sin(omega)*y_c;
		OBJY[OBJ_N] = sin(omega)*x_c + cos(omega)*y_c;
		OBJ_N = OBJ_N + 1;

	}

	OBJX.resize(OBJ_N);
	OBJY.resize(OBJ_N);
	double PG;
	double POJ,POA[dd];
	int IN_N[dd];
	
	double N_A,N_B,O_X,O_Y,O_XY,min_o = 1000000000000000000,min_N;

	

	for(int NNN = 0;NNN <= dd;NNN++){
		
		double ANG = dt*NNN;
		double DT_X = 0.1*cos(ANG);
		double DT_Y = 0.1*sin(ANG);

		N_A = GGX - (-robotOdom.pose.pose.position.y + DT_X);
		N_B = GGY - (robotOdom.pose.pose.position.x + DT_Y);
		IN_N[NNN] = 0;
		POJ = 0;

		PG = -1/sqrt(pow(N_A,2) + pow(N_B,2));

		for(int tt = 0;tt < OBJ_N; tt++){

			O_X = DT_X - OBJX[OBJ_N];
			O_Y = DT_Y - OBJY[OBJ_N];
			O_XY = sqrt(pow(O_X,2) + pow(O_Y,2));

			if(R_den > O_XY){
				POJ = POJ + 1/O_XY;
				IN_N[NNN] = IN_N[NNN] + 1;
			}
		
		}
		
		POA[NNN] = PG + POJ;

		if(min_o > POA[NNN]){
			min_N = NNN;
		}

	}

	
	if(U_all > Length_max){
		if(UY > 0){
			tagVel = MAX_S;
			tagAng = atan2(UY,UX)  - M_PI/2;
		}else{
			tagVel = 0;
			tagAng = 0.7*(atan2(UY,UX)  - M_PI/2);
		}
		

	}else{
		double at1 = U_all/Length_max;

		if(UY > 0){
			tagVel = at1*MAX_S;
			tagAng = atan2(UY,UX)  - M_PI/2;
		}else{
			tagVel = 0;
			tagAng = 0.7*(atan2(UY,UX)  - M_PI/2);
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
	float gainP = 1;
	//float tagAngVel = (M_PI*(tagAng-cur_ang)/180)*gainP;
	
	//A
	//float tagAngVel = (tagAng - cur_ang)*gainP;
	//B
	//float tagAngVel = (tagAng - robotOdom.twist.twist.angular.z)*gainP;
	float tagAngVel = (tagAng - robotOdom.pose.pose.orientation.z)*gainP;
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


