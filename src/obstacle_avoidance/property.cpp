#include<local_navigation/obstacleAvoidance.h>

void obstacleAvoidance::setLaunchParam(){
    
    ros::NodeHandle n("~");
	n.getParam("obstacleAvoidance/SEARCH_ONLY_ANGLE",SEARCH_ONLY_ANGLE);   
	n.getParam("obstacleAvoidance/rqt_reconfigure",rqt_reconfigure);    
	//ロボットパラメータ
	n.getParam("obstacleAvoidance/WheelD",d);
    n.getParam("obstacleAvoidance/angleMin", angle_min);
    n.getParam("obstacleAvoidance/angleMax", angle_max);
    n.getParam("obstacleAvoidance/angleDiv", angle_div);
    n.getParam("obstacleAvoidance/maxSpeed", max_speed);
    n.getParam("obstacleAvoidance/minSpeed", min_speed);
    n.getParam("obstacleAvoidance/defaultSpeed", default_speed);
    //ゴール位置
    n.getParam("obstacleAvoidance/goalX", goal_x);
    n.getParam("obstacleAvoidance/goalY", goal_y);
    goalOdom.pose.pose.position.x = goal_x;
    goalOdom.pose.pose.position.y = goal_y;
    goalOdom.pose.pose.position.z = 0;
    RECEIVED_GOAL_ODOM = true;//検討中
    //vfh
    n.getParam("obstacleAvoidance/distance_th", dis_th);
	//--k
    n.getParam("obstacleAvoidance/marginRadius", marginRadius);
	n.getParam("obstacleAvoidance/Kcp",k_cp);
    n.getParam("obstacleAvoidance/Kg",k_g);
    n.getParam("obstacleAvoidance/KcurAngle",k_curAngle);
    n.getParam("obstacleAvoidance/KprevAngle",k_prevAngle);
    //--eta
    n.getParam("obstacleAvoidance/EtaCp",eta_cp);
    n.getParam("obstacleAvoidance/EtaG",eta_g);
    n.getParam("obstacleAvoidance/EtaCurAngle",eta_curAngle);
    n.getParam("obstacleAvoidance/EtaPrevAngle",eta_prevAngle);
    //safe range
    n.getParam("obstacleAvoidance/safeRange",safe_range);
    //探査範囲
    n.getParam("obstacleAvoidance/searchRange_vel",dV_range);
    n.getParam("obstacleAvoidance/searchDiv_vel",dV_div);
    //デバッグ
    n.getParam("obstacleAvoidance/debugType",debugType);

    n.getParam("obstacleAvoidance/CONTROLLER/GAIN/P",CONTROLLER_GAIN_P);
    n.getParam("obstacleAvoidance/POTENTIAL/F1",POTENTIAL_F1);
    n.getParam("obstacleAvoidance/POTENTIAL/F1",POTENTIAL_F2);
    n.getParam("obstacleAvoidance/MOVE_MEAN/WINDOW_NUM",MOVE_MEAN_WINDOW_NUM);
}
void obstacleAvoidance::configCallback(local_navigation::obstacleAvoidanceConfig &config, uint32_t level) {
	// ROS_INFO("Reconfigure Request: %d %f %f %d", 
	// 	config.windowDivisionDegree, config.windowHeight,
	// 	config.windowWidth,config.windowMinPts
	// 	// config.str_param.c_str(), 
	// 	// config.bool_param?"True":"False", 
	// 	// config.size
	// 	);
    //評価式の重み
    k_cp = config.Kcp;
    k_g = config.Kg;
    k_curAngle = config.KcurAngle;
    k_prevAngle = config.KprevAngle;
    k_vel = config.Kv;
	//コスト関数のパラメータ
    eta_cp = config.EtaCp;
    eta_g = config.EtaG;
    eta_curAngle = config.EtaCurAngle;
    eta_prevAngle = config.EtaPrevAngle;
    eta_vel = config.EtaVel;
    //
    safe_range = config.safeRange;
    crossWeightX = config.crossWeightX;
    crossWeightY = config.crossWeightY;
    timeBias = config.timeBias;
	//デバッグ
    debugType = config.debugType;
    //クロスポイントチェッカー
    debugFlag_crossPointChecker = config.crossPointCheckerFlag;
    // debugEncoderVel_r = config.debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
    // debugEncoderVel_l = config.debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
    debugCur_vel = config.debugCur_vel;
    debugCur_angle_steer = config.debugCur_angle*M_PI/180;
    debugCmd_vel = config.debugCmd_vel;//ロボット目標速度
    debugCmd_angle = config.debugCmd_angle*M_PI/180;//ロボット目標角度 deg -> rad
    debugIndexRef = config.debugIndexRef;//障害物番号
    debugGpRef.x = config.debugGpRef_x;//クラスタ重心x
    debugGpRef.y = config.debugGpRef_y;//クラスタ重心y
    debugGpRef.z = config.debugGpRef_z;//クラスタ重心z
    debugTwistRef.linear.x = config.debugTwistRefLinear_x;//障害物速度linear x
    debugTwistRef.linear.y = config.debugTwistRefLinear_y;//障害物速度linear y
    debugTwistRef.linear.z = 0;//config.debugTwistRefLinear_z;//障害物速度linear z
    debugTwistRef.angular.x = 0;//障害物速度anguler x
    debugTwistRef.angular.y = 0;//障害物速度anguler y
    debugTwistRef.angular.z = config.debugTwistRefAngular_theta;//障害物速度anguler z
    debugObstacleRadius = config.debugObstacleRadius;//障害物半径
    debugRobotRadius = config.debugRobotRadius;//ロボット半径
    //checkCrossPoint
    //if(debugFlag_crossPointChecker){
    //    crossPointChecker();
    //}//20220916
    //ヒストグラムチェッカー
    debugHistgramCheckerFlag = config.debugHistgramCheckerFlag;
    debugObstacleNum = config.debugObstacleNum;
    debugObstacleX1 = config.debugObstacleX1;
    debugObstacleY1 = config.debugObstacleY1;
    debugObstacleSize1 = config.debugObstacleSize1;
    debugObstacleX2 = config.debugObstacleX2;
    debugObstacleY2 = config.debugObstacleY2;
    debugObstacleSize2 = config.debugObstacleSize2;
    debugObstacleX3 = config.debugObstacleX3;
    debugObstacleY3  = config.debugObstacleY3;
    debugObstacleSize3 = config.debugObstacleSize3;
    debugThresholdDistance = config.debugThresholdDistance;
    debugMinAngle = config.debugMinAngle;
    debugMaxAngle = config.debugMaxAngle;
    debugDivAngle = config.debugDivAngle;
    debugEtaG = config.debugEtaG;
    debugEtaCurAngle = config.debugEtaCurAngle;
    debugEtaPrevAngle = config.debugEtaPrevAngle;
    debugMarginRadius = config.debugMarginRadius;
    //if(debugHistgramCheckerFlag){
    //    histgramChecker();
    //}//20220916
    //出力チェッカー
    debugOutputVFHCheckerFlag = config.debugOutputVFHCheckerFlag;
    debugEtaG = config.debugEtaG;
    debugEtaCurAngle = config.debugEtaCurAngle;
    debugEtaPrevAngle = config.debugEtaPrevAngle;
    debugKg = config.debugKg;
    debugKcurAngle = config.debugKcurAngle;
    debugKprevAngle = config.debugKprevAngle;
    // debugGoalAng = config.debugGoalAng;
    debugGoalPosX = config.debugGoalPosX;
    debugGoalPosY = config.debugGoalPosY;
    debugCurAng = config.debugCurAng;
    debugPrevTagAng = config.debugPrevTagAng;
    // 
    debugRotOmega =config.debugRotOmega;
    //if(debugOutputVFHCheckerFlag){
    //   outputVFHChecker();
    //}//20220916
    //
    debugOutputCPVFHCheckerFlag = config.debugOutputCPVFHCheckerFlag;
    debugKcp = config.debugKcp;
    debugEtaCp = config.debugEtaCp;
    debugObstacleVx1 = config.debugObstacleVx1;
    debugObstacleVy1 = config.debugObstacleVy1;
    debugObstacleVx2 = config.debugObstacleVx2;
    debugObstacleVy2 = config.debugObstacleVy2;
    debugObstacleVx3 = config.debugObstacleVx3;
    debugObstacleVy3 = config.debugObstacleVy3;
    //位置
    debugGp1.x = debugObstacleX1;
    debugGp1.y = debugObstacleY1;
    debugGp1.z = 0;
    debugGp2.x = debugObstacleX2;
    debugGp2.y = debugObstacleY2;
    debugGp2.z = 0;
    debugGp3.x = debugObstacleX3;
    debugGp3.y = debugObstacleY3;
    debugGp3.z = 0;
    //速度
    debugTwist1.linear.x = debugObstacleVx1;
    debugTwist1.linear.y = debugObstacleVy1;
    debugTwist1.linear.z = 0;
    debugTwist2.linear.x = debugObstacleVx2;
    debugTwist2.linear.y = debugObstacleVy2;
    debugTwist2.linear.z = 0;
    debugTwist3.linear.x = debugObstacleVx3;
    debugTwist3.linear.y = debugObstacleVy3;
    debugTwist3.linear.z = 0; 
    //角速度
    debugTwist1.angular.x = 0;
    debugTwist1.angular.y = 0;
    debugTwist1.angular.z = 0;
    debugTwist2.angular.x = 0;
    debugTwist2.angular.y = 0;
    debugTwist2.angular.z = 0;
    debugTwist3.angular.x = 0;
    debugTwist3.angular.y = 0;
    debugTwist3.angular.z = 0;
    //障害物サイズ閾値
    debugObstacleSizeThreshold = config.debugObstacleSizeThreshold;
    //
    //if(debugOutputCPVFHCheckerFlag){
    //   outputCrossPointVFHChecker();
    //}//20220916
    //交差位置の表示有無
    display_output = config.displayOutput;
    //
    debugRotationVelocityCheckerFlag = config.debugRotationVelocityCheckerFlag;
    
}

void obstacleAvoidance::setDefaultCrossPointChecker(){
    //クロスポイントチェッカー入力
    // float debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
    // float debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
    // float debugCmd_vel;//ロボット目標速度
    // float debugCmd_angle;//ロボット目標角度
    // int debugIndexRef;//障害物番号
    // geometry_msgs::Point debugGpRef;//クラスタ重心
    // geometry_msgs::Twist debugTwistRef;//障害物速度
    // float debguObstacleRadius;//障害物半径
    // float debugRobotRadius;//ロボット半径
    debugFlag_crossPointChecker = false;
    debugEncoderVel_r = 0;//ロボットエンコーダ速度(右車輪)
    debugEncoderVel_l = 0;//ロボットエンコーダ速度(左車輪)
    debugCmd_vel = 0;//ロボット目標速度
    debugCmd_angle = 0;//ロボット目標角度
    debugIndexRef = 0;//障害物番号
    debugGpRef.x = 0;//クラスタ重心x
    debugGpRef.y = 0;//クラスタ重心y
    debugGpRef.z = 0;//クラスタ重心z
    debugTwistRef.linear.x = 0;//障害物速度linear x
    debugTwistRef.linear.y = 0;//障害物速度linear y
    debugTwistRef.linear.z = 0;//障害物速度linear z
    debugTwistRef.angular.x = 0;//障害物速度anguler x
    debugTwistRef.angular.y = 0;//障害物速度anguler y
    debugTwistRef.angular.z = 0;//障害物速度anguler z
    debugObstacleRadius = 1.0;//障害物半径
    debugRobotRadius = 1.0;//ロボット半径


}
