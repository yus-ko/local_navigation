#include<local_navigation/obstacleAvoidance.h>

void obstacleAvoidance::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //デバッグ
    n.getParam("measurementVelocity/debugType",debugType);
}
void obstacleAvoidance::configCallback(local_navigation::obstacleAvoidanceConfig &config, uint32_t level) {
	// ROS_INFO("Reconfigure Request: %d %f %f %d", 
	// 	config.windowDivisionDegree, config.windowHeight,
	// 	config.windowWidth,config.windowMinPts
	// 	// config.str_param.c_str(), 
	// 	// config.bool_param?"True":"False", 
	// 	// config.size
	// 	);
	//デバッグ
    debugType = config.debugType;
}