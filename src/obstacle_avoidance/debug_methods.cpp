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
    //座標変換 local to rviz
    // debugGpRef.x = 

    //交差位置を取得
    crossPoint crsPtTemp = getCrossPoint(debugIndexRef, debugGpRef,debugTwistRef,debugCmd_vel,debugCmd_angle);

    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    int k = 0;
    markerArray.markers.resize(100);
    marker.scale.x = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.y = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.z = 0.1;
    //position
    //定義済みの交差位置構造体から取得
    crossPoint crsPt = crsPtTemp;
    ROS_INFO("t_cross:%f",crsPt.t);
    //危険, 安全障害物ともに同じように表示している
    //
    marker.pose.position.x = crsPt.y;
    marker.pose.position.y = -crsPt.x;
    marker.pose.position.z = 0;
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //text crossPoint
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.4;
    marker.pose.position.z += 1.0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "xy:("+ std::to_string(crsPt.x) +","+ std::to_string(crsPt.y)+") \n t:"+std::to_string(crsPt.t);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //
    //障害物位置と速度ベクトル
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = debugObstacleRadius*2+abs(debugTwistRef.linear.y);
    marker.scale.y = debugObstacleRadius*2+abs(-debugTwistRef.linear.x);
    marker.scale.z = 0.1;
    // local -> rviz 
    marker.pose.position.x = debugGpRef.y;
    marker.pose.position.y = -debugGpRef.x;
    marker.pose.position.z = debugGpRef.z;
    //angle
    double yaw = std::atan2(-debugTwistRef.linear.x, debugTwistRef.linear.y);
    //culc Quaternion
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    //
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.id = k;
    markerArray.markers[k++] = marker;
    //text ObstaclePoint
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z += 2.0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "xy:("+ std::to_string(debugGpRef.x) +","+ std::to_string(debugGpRef.y)+")"+ "\n"
                    + "v,ang:("+ std::to_string(debugTwistRef.linear.x) +","+ std::to_string(debugTwistRef.linear.y)+")"+ "\n"
                    + "state:" + ( (crsPtTemp.safe == true) ? "SAFE" : "WARNING");
    marker.id = k;
    markerArray.markers[k++] = marker;
    //
    //ロボットの目標速度と目標角度
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = debugRobotRadius*2;
    marker.scale.y = debugRobotRadius*2;
    marker.scale.z = 0.1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    //angle
    // local -> rviz 
    yaw = debugCmd_angle - M_PI_2;
    //culc Quaternion
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    //
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.id = k;
    markerArray.markers[k++] = marker;
    //text RobotPoint
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z += 0.5;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "v,ang:("+ std::to_string(debugCmd_vel) +","+ std::to_string(debugCmd_angle)+")";
    marker.id = k;
    markerArray.markers[k++] = marker;

    markerArray.markers.resize(k);

    pubDebCross.publish( markerArray );
}
//ヒストグラムの出力結果を視覚的に表示する
void obstacleAvoidance::histgramChecker(){
    //VFHクラスを定義、ヒストグラム算出
    vfh vfh_check;
    //パラメータ設定
    vfh_check.set_histgram_param(debugMinAngle,debugMaxAngle,debugDivAngle);
	vfh_check.set_dis_threshold(debugThresholdDistance);
	vfh_check.set_eta(debugEtaG, debugEtaTheta, debugEtaOmega);
    //ヒストグラムにデータを追加
    float angleTemp;
    float disTemp;
    //obstacle1
    angleTemp = atan2(debugObstacleY1, debugObstacleX1)*180/M_PI;
    disTemp = sqrt(debugObstacleY1*debugObstacleY1 + debugObstacleX1*debugObstacleX1);
    vfh_check.add_histgram_dis(angleTemp, disTemp);
    //obstacle2
    angleTemp = atan2(debugObstacleY2, debugObstacleX2)*180/M_PI;
    disTemp = sqrt(debugObstacleY2*debugObstacleY2 + debugObstacleX2*debugObstacleX2);
    vfh_check.add_histgram_dis(angleTemp, disTemp);
    //obstacle3
    angleTemp = atan2(debugObstacleY3, debugObstacleX3)*180/M_PI;
    disTemp = sqrt(debugObstacleY3*debugObstacleY3 + debugObstacleX3*debugObstacleX3);
    vfh_check.add_histgram_dis(angleTemp, disTemp);
    //バイナリヒストグラムの作成 
    vfh_check.create_binary_histgram(debugRobotRadius, debugMarginRadius);
    //距離ヒストグラム
    std::vector<double> histgram_dis;
    vfh_check.get_histgram_dis(histgram_dis);
    //バイナリヒストグラム
    std::vector<bool> histgram_bi;
    vfh_check.get_histgram_bi(histgram_bi);
    // ROS_INFO_STREAM("histgram_bi:[");
    // for(int i=0;i<histgram_bi.size();i++){
    //     std::cout<<(angle_min + i * angle_div)<<": "<<histgram_bi[i]<<"\n";
    // }
    // ROS_INFO_STREAM("]histgram_end");
    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    
    int k = 0;
    markerArray.markers.resize((int)histgram_dis.size()*2);
    //
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    //距離ヒストグラム
    for(int i=0; i<histgram_dis.size();i++){
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle)*M_PI/180;//rad
        marker.color.a = 1.0;
        marker.color.r = 0;
        marker.color.g = 255;
        marker.color.b = 0;
        if(l ==-1){
            l =4;
            marker.color.r = 0;
            marker.color.g = 255;
            marker.color.b = 255;
        }
        //x, y 座標を算出
        double x = l*cos(ang);
        double y = l*sin(ang);
        //
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        marker.pose.position.z = 0;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }    

    //２値ヒストグラム
    for(int i=0; i<histgram_bi.size();i++){
        
        double l = debugThresholdDistance;
        double ang = (debugMinAngle + i * debugDivAngle)*M_PI/180;//rad
        marker.color.a = 1.0;
        marker.color.r = 255;
        marker.color.g = 255;
        marker.color.b = 255;

        if(!histgram_bi[i]){
            l = 1.0;
            marker.color.g = 0;
            marker.color.b = 0;
        }

        //x, y 座標を算出
        double x = l*cos(ang);
        double y = l*sin(ang);
        //
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        // marker.pose.position.z = 1.0;

        marker.id = k;
        markerArray.markers[k++] = marker;

    }
    markerArray.markers.resize(k);

    pubDebHst.publish( markerArray );
}
//出力（命令速度, 角度）を視覚的に表示する
void obstacleAvoidance::outputChecker(){
    //ヒストグラムチェッカーのパラメータを共有
    //--
    //VFHクラスを定義、ヒストグラム算出
    vfh vfh_check;
    //パラメータ設定
    vfh_check.set_histgram_param(debugMinAngle,debugMaxAngle,debugDivAngle);
	vfh_check.set_dis_threshold(debugThresholdDistance);
	vfh_check.set_eta(debugEtaG, debugEtaTheta, debugEtaOmega);
    //ヒストグラムにデータを追加
    float angleTemp;
    float disTemp;
    //obstacle1
    angleTemp = atan2(debugObstacleY1, debugObstacleX1)*180/M_PI;
    disTemp = sqrt(debugObstacleY1*debugObstacleY1 + debugObstacleX1*debugObstacleX1);
    vfh_check.add_histgram_dis(angleTemp, disTemp);
    //obstacle2
    angleTemp = atan2(debugObstacleY2, debugObstacleX2)*180/M_PI;
    disTemp = sqrt(debugObstacleY2*debugObstacleY2 + debugObstacleX2*debugObstacleX2);
    vfh_check.add_histgram_dis(angleTemp, disTemp);
    //obstacle3
    angleTemp = atan2(debugObstacleY3, debugObstacleX3)*180/M_PI;
    disTemp = sqrt(debugObstacleY3*debugObstacleY3 + debugObstacleX3*debugObstacleX3);
    vfh_check.add_histgram_dis(angleTemp, disTemp);
    //バイナリヒストグラムの作成 
    vfh_check.create_binary_histgram(debugRobotRadius, debugMarginRadius);
    //距離ヒストグラム
    std::vector<double> histgram_dis;
    vfh_check.get_histgram_dis(histgram_dis);
    //バイナリヒストグラム
    std::vector<bool> histgram_bi;
    vfh_check.get_histgram_bi(histgram_bi);
    //--
    double min_cost = 1;
    int min_num = -1;
    double max_cost = 0;
    int max_num = -1;
    //weight正規化
    double sum_weight = debugKg + debugKtheta + debugKomega;
    debugKg/=sum_weight;
    debugKtheta/=sum_weight;
    debugKomega/=sum_weight;
    // ROS_INFO("goal,angle,angleVel: %f, %f, %f", debugKg, debugKtheta, debugKomega);
    //コスト算出
    double sum_cost=0;
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle);//deg

        double difAng = debugGoalAng - ang;
        if(debugGoalAng < -90){
            debugGoalAng+=360;
            difAng = debugGoalAng - ang;
        }
        if (debugGoalAng < debugMinAngle )
        {
            debugGoalAng = debugMinAngle;
        }
        else if (debugGoalAng > debugMaxAngle)
        {
            debugGoalAng = debugMaxAngle;
        }
        

        double angVel = (ang - debugCurAng)*debugControlKp;
        float goalCost = vfh_check.cost_goalAngle(difAng);
        float angCost = vfh_check.cost_theta_depend_time(ang - debugCurAng);
        float angVelCost = vfh_check.cost_omega_depend_time(angVel - debugCurAngVel);
        double cost = debugKg * goalCost + debugKtheta * angCost + debugKomega * angVelCost;
        if(min_cost > cost){
            min_cost = cost;
            min_num = i;
        }
        if(max_cost < cost){
            max_cost = cost;
            max_num = i;
        }
        sum_cost+=cost;
    }
    // ROS_INFO("input:[goal,angle,angleVel]: [\n %f,\n %f,\n %f]", 
    //     debugGoalAng - (angle_min + min_num * angle_div)*180/M_PI, 
    //     (angle_min + min_num * angle_div)*180/M_PI - debugCurAng, 
    //     ((angle_min + min_num * angle_div)*180/M_PI - debugCurAng)*debugControlKp - debugCurAngVel
    //     );
    // ROS_INFO("cost:[goal,angle,angleVel]: [\n %f,\n %f,\n %f]", 
    //     vfh_check.cost_goalAngle(debugGoalAng - (angle_min + min_num * angle_div)*180/M_PI), 
    //     vfh_check.cost_theta_depend_time((angle_min + min_num * angle_div)*180/M_PI - debugCurAng), 
    //     vfh_check.cost_omega_depend_time(((angle_min + min_num * angle_div)*180/M_PI - debugCurAng)*debugControlKp - debugCurAngVel)
    //     );
    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    //目標矢印
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 122;
    int k = 0;
    markerArray.markers.resize((int)histgram_dis.size()*2);
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    double yaw = (debugMinAngle + min_num * debugDivAngle)*M_PI/180;
    // ROS_INFO("yaw:%f",yaw);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI_2);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //目標矢印（テキスト）
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "v,ang:("+ std::to_string(debugCmd_vel) +","+ std::to_string(yaw*180/M_PI)+")";
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = 0.5;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //
    //コストでグラデーション
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double l = 1.5;
        double ang = (debugMinAngle + i * debugDivAngle);//deg
        double angVel = (ang - debugCurAng)*debugControlKp;

        double difAng = debugGoalAng - ang;
        if(difAng < -180){
            debugGoalAng+=360;
        }
        if (debugGoalAng < debugMinAngle )
        {
            debugGoalAng = debugMinAngle;
        }
        else if (debugGoalAng > debugMaxAngle)
        {
            debugGoalAng = debugMaxAngle;
        }
        

        float goalCost = vfh_check.cost_goalAngle(difAng);
        float angCost = vfh_check.cost_theta_depend_time(ang - debugCurAng);
        float angVelCost = vfh_check.cost_omega_depend_time(angVel - debugCurAngVel);
        double cost = debugKg * goalCost + debugKtheta * angCost + debugKomega * angVelCost;
        
        marker.color.r = ((int)(goalCost/max_cost*255))*debugKg;
        marker.color.g = ((int)(angCost/max_cost*255))*debugKtheta;
        marker.color.b = ((int)(angVelCost/max_cost*255))*debugKomega;
        std::cout<< i<<": " <<marker.color.r<<", "<<marker.color.g<<", "<<marker.color.b<<std::endl;
        //x, y 座標を算出
        double x = l*cos(ang/180*M_PI);
        double y = l*sin(ang/180*M_PI);
        //
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        marker.pose.position.z = 0;
        marker.id = k;
        markerArray.markers[k++] = marker;


        double cost2 = cost/max_cost;
        marker.pose.position.z = cost2*5;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }

    //
    markerArray.markers.resize(k);
    pubDebOutput.publish( markerArray );
}