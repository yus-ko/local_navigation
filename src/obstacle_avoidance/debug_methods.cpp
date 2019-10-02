#include<local_navigation/obstacleAvoidance.h>

//デバッグ方法選択メソッド
void obstacleAvoidance::debug(){
    switch(debugType){
        case 1: showCrossPoints();break;
        default: ;
    }
}

// 交差位置をマーカーで表示する
void obstacleAvoidance::showCrossPoints(){
    //--sample
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = clstr.header.stamp;
    marker.ns = "my_namespace";
    // marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)clstr.data.size() * 2);
    int count = 0;
    //交差位置を取得
    std::vector<crossPoint> crsPts;
	crsPts.resize(clstr.data.size());

	crossPointsDetect(crsPts,debugCmd_vel,debugCmd_angle);//rqt_reconfiureの値を使用


    for(int k=0; k<clstr.data.size(); k++){
        marker.scale.x = debugObstacleRadius*2;
        marker.scale.y = debugObstacleRadius*2;
        marker.scale.z = 0.1;
        //position
        //定義済みの交差位置構造体から取得
	    crossPoint crsPt = crsPts[k];
        //危険, 安全障害物ともに同じように表示している
        //
        marker.pose.position.x = crsPt.x;
        marker.pose.position.y = crsPt.y;
        marker.pose.position.z = 0;
        marker.color.a = 1.0;
        marker.color.r = colors[k][0];
        marker.color.g = colors[k][1];
        marker.color.b = colors[k][2];
        marker.type = visualization_msgs::Marker::SPHERE;

        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
    }
    markerArray.markers.resize(count);
    // ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
    if(markerArray.markers.size()){
        pubDebMarkerArray.publish( markerArray );
    }
}
// クロスコストマップを表示する
void obstacleAvoidance::showCostMap(){
    //コストマップ生成のためのパラメータからコストマップをグラデーションで表現する
    //パラメータを取得
    //process

    //マップの全てのセルに対する評価値を取得する
    //process

    //マップを描画する
    //process

    //publishする

}
void obstacleAvoidance::crossPointChecker(){
    //入力
    // float debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
    // float debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
    // float debugCmd_vel;//ロボット目標速度
    // float debugCmd_angle;//ロボット目標角度
    // int debugIndexRef;//障害物番号
    // geometry_msgs::Point debugGpRef;//クラスタ重心
    // geometry_msgs::Twist debugTwistRef;//障害物速度
    // float debguObstacleRadius;//障害物半径
    // float debugRobotRadius;//ロボット半径
    //エンコーダ速度セット
    // robotEncoder.vel.r = debugEncoderVel_r;
    // robotEncoder.vel.l = debugEncoderVel_l;
    //交差位置を取得
    crossPoint crsPtTemp = getCrossPoint(debugIndexRef, debugGpRef,debugTwistRef,debugCmd_vel,debugCmd_angle);

    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    int k = 0;
    markerArray.markers.resize(3);
    marker.scale.x = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.y = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.z = 0.1;
    //position
    //定義済みの交差位置構造体から取得
    crossPoint crsPt = crsPtTemp;
    ROS_INFO("t_cross:%f",crsPt.t);
    //危険, 安全障害物ともに同じように表示している
    //
    marker.pose.position.x = crsPt.x;
    marker.pose.position.y = crsPt.y;
    marker.pose.position.z = 0;
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //障害物位置と速度ベクトル
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = debugObstacleRadius*2;
    marker.scale.y = debugObstacleRadius*2;
    marker.scale.z = 0.1;
    marker.pose.position.x = debugGpRef.x;
    marker.pose.position.y = debugGpRef.y;
    marker.pose.position.z = debugGpRef.z;
    //angle
    double yaw = std::atan2(debugTwistRef.linear.x,debugTwistRef.linear.y);
    //culc Quaternion
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    //
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.id = k;
    markerArray.markers[k++] = marker;
    //ロボットの目標速度と目標角度
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = debugRobotRadius*2;
    marker.scale.y = debugRobotRadius*2;
    marker.scale.z = 0.1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    //angle
    yaw = debugCmd_angle;
    //culc Quaternion
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    //
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.id = k;
    markerArray.markers[k++] = marker;

    pubDebMarker.publish( markerArray );
}