#include<local_navigation/obstacleAvoidance.h>

void obstacleAvoidance::setLaunchParam(){
    
    ros::NodeHandle n("~");
	//ロボットパラメータ
	n.getParam("obstacleAvoidance/WheelD",d);
    //評価式の重み
	n.getParam("obstacleAvoidance/Kcp",k_cp);
    n.getParam("obstacleAvoidance/Ko",k_o);
    n.getParam("obstacleAvoidance/Kg",k_g);
    n.getParam("obstacleAvoidance/Ktheta",k_theta);
    n.getParam("obstacleAvoidance/Komega",k_omega);
    //コスト関数のパラメータ
    n.getParam("obstacleAvoidance/EtaCp",eta_cp);
    n.getParam("obstacleAvoidance/EtaO",eta_o);
    n.getParam("obstacleAvoidance/EtaG",eta_g);
    n.getParam("obstacleAvoidance/EtaTheta",eta_theta);
    n.getParam("obstacleAvoidance/EtaOmega",eta_omega);
    //デバッグ
    n.getParam("obstacleAvoidance/debugType",debugType);
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
    k_o = config.Ko;
    k_g = config.Kg;
    k_theta = config.Ktheta;
    k_omega = config.Komega;
	//コスト関数のパラメータ
    eta_cp = config.EtaCp;
    eta_o = config.EtaO;
    eta_g = config.EtaG;
    eta_theta = config.EtaTheta;
    eta_omega = config.EtaOmega;
	//デバッグ
    debugType = config.debugType;
    //クロスポイントチェッカー
    debugFlag_crossPointChecker = config.crossPointCheckerFlag;
    // debugEncoderVel_r = config.debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
    // debugEncoderVel_l = config.debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
    debugCmd_vel = config.debugCmd_vel;//ロボット目標速度
    debugCmd_angle = config.debugCmd_angle;//ロボット目標角度
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
    if(debugFlag_crossPointChecker){
        crossPointChecker();
    }

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
    debugEtaTheta = config.debugEtaTheta;
    debugEtaOmega = config.debugEtaOmega;
    debugMarginRadius = config.debugMarginRadius;
    if(debugHistgramCheckerFlag){
        histgramChecker();
    }
    //出力チェッカー
    debugOutputCheckerFlag = config.debugOutputCheckerFlag;
    debugEtaCp = config.debugEtaCp;
    debugEtaO = config.debugEtaO;
    debugEtaG = config.debugEtaG;
    debugEtaTheta = config.debugEtaTheta;
    debugEtaOmega = config.debugEtaOmega;
    debugKcp = config.debugKcp;
    debugKo = config.debugKo;
    debugKg = config.debugKg;
    debugKtheta = config.debugKtheta;
    debugKomega = config.debugKomega;
    debugGoalAng = config.debugGoalAng;
    debugCurAng = config.debugCurAng;
    debugCurAngVel = config.debugCurAngVel;
    debugControlKp = config.debugControlKp;
    if(debugOutputCheckerFlag){
       outputChecker();
    }
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
