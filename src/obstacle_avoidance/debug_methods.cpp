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
    marker.text = "xy:("+ std::to_string(crsPt.x) +","+ std::to_string(crsPt.y)+") \n t:"+std::to_string(crsPt.t)+ "dis:"+std::to_string(crsPt.dis);
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
void obstacleAvoidance::outputVFHChecker(){
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
    debugGoalAng = atan2(debugGoalPosY,debugGoalPosX)*180/M_PI;
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
        double goalCost = vfh_check.cost_goalAngle(difAng);
        double angCost = vfh_check.cost_theta_depend_time(ang - debugCurAng);
        double angVelCost = vfh_check.cost_omega_depend_time(angVel - debugCurAngVel);
        double cost = debugKg * goalCost + debugKtheta * angCost + debugKomega * angVelCost;
        if(min_cost > cost){
            min_cost = cost;
            min_num = i;
        }
        if(max_cost < cost){
            max_cost = cost;
            max_num = i;
        }
    }
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
    markerArray.markers.resize((int)histgram_dis.size()*3);
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
    //目標位置
    //マーカー
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 0;
    marker.pose.position.x = debugGoalPosY;
    marker.pose.position.y = -debugGoalPosX;
    marker.pose.position.z = 0;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //参照矢印
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.4;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 122;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    yaw = debugGoalAng*M_PI/180;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI_2);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //参照矢印（テキスト）
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "goalAng:("+ std::to_string(debugGoalAng)+")";
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = -0.5;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //
    markerArray.markers.resize(k);
    pubDebOutput.publish( markerArray );
}
void obstacleAvoidance::outputCrossPointVFHChecker(){
    //VFHクラスを定義、ヒストグラム算出
    vfh vfh_check;
    //パラメータ設定 outputVFHパラメータを使用
    vfh_check.set_histgram_param(debugMinAngle,debugMaxAngle,debugDivAngle);
	vfh_check.set_dis_threshold(debugThresholdDistance);
	vfh_check.set_eta(debugEtaG, debugEtaTheta, debugEtaOmega);
    
    //ヒストグラムにデータを追加
    float angleTemp;
    float disTemp;
    //障害物1
    if((debugTwist1.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold){
        angleTemp = atan2(debugObstacleY1, debugObstacleX1)*180/M_PI;
        disTemp = sqrt(debugObstacleY1*debugObstacleY1 + debugObstacleX1*debugObstacleX1);
        vfh_check.add_histgram_dis(angleTemp, disTemp);
    }
    if((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold){
        angleTemp = atan2(debugObstacleY2, debugObstacleX2)*180/M_PI;
        disTemp = sqrt(debugObstacleY2*debugObstacleY2 + debugObstacleX2*debugObstacleX2);
        vfh_check.add_histgram_dis(angleTemp, disTemp); 
    }
    if((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold){
        angleTemp = atan2(debugObstacleY3, debugObstacleX3)*180/M_PI;
        disTemp = sqrt(debugObstacleY3*debugObstacleY3 + debugObstacleX3*debugObstacleX3);
        vfh_check.add_histgram_dis(angleTemp, disTemp);
    }

    //バイナリヒストグラムの作成 
    vfh_check.create_binary_histgram(debugRobotRadius, debugMarginRadius);
    //距離ヒストグラム
    std::vector<double> histgram_dis;
    vfh_check.get_histgram_dis(histgram_dis);
    //バイナリヒストグラム
    std::vector<bool> histgram_bi;
    vfh_check.get_histgram_bi(histgram_bi);
    //
    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    int k = 0;
    markerArray.markers.resize((int)histgram_dis.size()*10);
    // コストでグラデーション
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    //
    //ゴール角度
    debugGoalAng = atan2(debugGoalPosY,debugGoalPosX)*180/M_PI;
    //weight正規化
    double sum_weight = debugKg + debugKtheta + debugKomega + debugKcp;
    debugKg/=sum_weight;
    debugKtheta/=sum_weight;
    debugKomega/=sum_weight;
    debugKcp/=sum_weight;
    //コスト算出
    double min_cost = 1;
    int min_num = -1;
    double max_cost = 0;
    int max_num = -1;
    ROS_INFO("cost searching:n=%d", (int)histgram_bi.size());
    //コスト最小交差位置を取得
    std::vector<crossPoint> debugCrsPtsMin;
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle);//deg
        float cmd_ang = ang*M_PI/180;
        double difAng = debugGoalAng - ang;
        //-PI,PI系と0,PI系の問題を調整, 角度差の最大値補正
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
        ////障害物との交差位置を算出(障害物の重心位置のみを使用)
        //交差位置を取得
        std::vector<crossPoint> debugCrsPts;
        debugCrsPts.clear();
        debugCrsPts.resize(3);
        int ptNum =0;
        if(!((debugTwist1.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold)){
            debugCrsPts[ptNum] = getCrossPoint(ptNum, debugGp1,debugTwist1,debugCmd_vel,cmd_ang);
            ptNum++;
        }
        if(!((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold)){
            debugCrsPts[ptNum] = getCrossPoint(ptNum, debugGp2,debugTwist2,debugCmd_vel,cmd_ang);
            ptNum++;
        }
        if(!((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold)){
            debugCrsPts[ptNum] = getCrossPoint(ptNum, debugGp3,debugTwist3,debugCmd_vel,cmd_ang);
            ptNum++;
        }
        debugCrsPts.resize(ptNum);
        //
        double angVel = (ang - debugCurAng)*debugControlKp;
        double goalCost = vfh_check.cost_goalAngle(difAng);
        double angCost = vfh_check.cost_theta_depend_time(ang - debugCurAng);
        double angVelCost = vfh_check.cost_omega_depend_time(angVel - debugCurAngVel);
        double crossCost = getCrossPointCost(debugCrsPts,debugEtaCp);
        double cost = debugKg * goalCost + debugKtheta * angCost + debugKomega * angVelCost + debugKcp*crossCost;
        if(min_cost > cost){
            min_cost = cost;
            min_num = i;
            debugCrsPtsMin = debugCrsPts;
        }
        if(max_cost < cost){
            max_cost = cost;
            max_num = i;
        }
    }
    ROS_INFO("be visualable");
    //コストでグラデーション
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle);//deg
        float cmd_ang = ang*M_PI/180;
        double difAng = debugGoalAng - ang;
        //-PI,PI系と0,PI系の問題を調整, 角度差の最大値補正
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
        ////障害物との交差位置を算出(障害物の重心位置のみを使用)
        //交差位置を取得
        std::vector<crossPoint> debugCrsPts;
        debugCrsPts.clear();
        debugCrsPts.resize(3);
        int ptNum =0;
        if(!((debugTwist1.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold)){
            debugCrsPts[ptNum] = getCrossPoint(ptNum, debugGp1,debugTwist1,debugCmd_vel,cmd_ang);
            ptNum++;
        }
        if(!((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold)){
            debugCrsPts[ptNum] = getCrossPoint(ptNum, debugGp2,debugTwist2,debugCmd_vel,cmd_ang);
            ptNum++;
        }
        if(!((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold)){
            debugCrsPts[ptNum] = getCrossPoint(ptNum, debugGp3,debugTwist3,debugCmd_vel,cmd_ang);
            ptNum++;
        }
        debugCrsPts.resize(ptNum);
        //
        double angVel = (ang - debugCurAng)*debugControlKp;
        double goalCost = vfh_check.cost_goalAngle(difAng);
        double angCost = vfh_check.cost_theta_depend_time(ang - debugCurAng);
        double angVelCost = vfh_check.cost_omega_depend_time(angVel - debugCurAngVel);
        double crossCost = getCrossPointCost(debugCrsPts, debugEtaCp);
        double cost = debugKg * goalCost + debugKtheta * angCost + debugKomega * angVelCost + debugKcp*crossCost;
        std::cout<< i<<": " <<debugKg * goalCost<<", "<<debugKtheta* angCost <<", "<<debugKomega * angVelCost<<","<<debugKcp*crossCost<<std::endl;
        //コストでグラデーション
        marker.color.a = 1.0;
        marker.color.r = ((uint8_t)(goalCost/max_cost*255*50))*debugKg;//ゴール角度
        marker.color.g = ((uint8_t)(angCost/max_cost*255*50))*debugKtheta;//現在角度
        marker.color.b = ((uint8_t)(crossCost/max_cost*255*50))*debugKcp;//交差位置
        // std::cout<< i<<": " <<marker.color.r<<", "<<marker.color.g<<", "<<marker.color.b<<std::endl;
        //x, y 座標を算出
        if(l ==-1){
            l =4;
        }
        double x = l*cos(ang/180*M_PI);
        double y = l*sin(ang/180*M_PI);
        //
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        marker.pose.position.z = 0;
        marker.id = k;
        markerArray.markers[k++] = marker;
        double cost2 = (cost-min_cost)/min_cost ;
        if(min_cost<0){
            cost2 =-cost2;
        }
        marker.pose.position.z = cost2*5;//高さ補正
        std::cout<< i<<": " <<marker.pose.position.x<<", "<<marker.pose.position.y<<", "<<marker.pose.position.z<<std::endl;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    //目標位置
    //マーカー
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 0;
    marker.pose.position.x = debugGoalPosY;
    marker.pose.position.y = -debugGoalPosX;
    marker.pose.position.z = 0;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //参照矢印
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.4;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 122;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    double refyaw = debugGoalAng*M_PI/180;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(refyaw-M_PI_2);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //参照矢印（テキスト）
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "goalAng:("+ std::to_string(debugGoalAng)+")";
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = -0.5;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //
    //目標矢印
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 122;
    double tagyaw = (debugMinAngle + min_num * debugDivAngle)*M_PI/180;
    // ROS_INFO("yaw:%f",yaw);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(tagyaw-M_PI_2);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //目標矢印（テキスト）
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "v,ang:("+ std::to_string(debugCmd_vel) +","+ std::to_string(tagyaw*180/M_PI)+")\n cost:" + std::to_string(min_cost);
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = 0.5;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //障害物位置と速度ベクトル
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.z = 0.05;
    if(!((debugTwist1.linear.x ==0 && debugTwist1.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold)){
        
        marker.scale.x = debugObstacleSize1;
        marker.scale.y = debugObstacleSize1;
        marker.pose.position.x = debugGp1.y;
        marker.pose.position.y = -debugGp1.x;
        marker.pose.position.z = debugGp1.z;
        //angle
        double yaw = std::atan2(-debugTwist1.linear.x, debugTwist1.linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.color.a = 1.0;
        marker.color.r = colors[0][0];
        marker.color.g = colors[0][1];
        marker.color.b = colors[0][2];
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    if(!((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold)){
        
        marker.scale.x = debugObstacleSize2;
        marker.scale.y = debugObstacleSize2;
        marker.pose.position.x = debugGp2.y;
        marker.pose.position.y = -debugGp2.x;
        marker.pose.position.z = debugGp2.z;
        //angle
        double yaw = std::atan2(-debugTwist2.linear.x, debugTwist2.linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.color.a = 1.0;
        marker.color.r = colors[1][0];
        marker.color.g = colors[1][1];
        marker.color.b = colors[1][2];
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    if(!((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold)){
        
        marker.scale.x = debugObstacleSize3;
        marker.scale.y = debugObstacleSize3;
        marker.pose.position.x = debugGp3.y;
        marker.pose.position.y = -debugGp3.x;
        marker.pose.position.z = debugGp3.z;
        //angle
        double yaw = std::atan2(-debugTwist3.linear.x, debugTwist3.linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.color.a = 1.0;
        marker.color.r = colors[2][0];
        marker.color.g = colors[2][1];
        marker.color.b = colors[2][2];
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    //交差位置
    for(int n=0; n<debugCrsPtsMin.size();n++){
        marker.type = visualization_msgs::Marker::SPHERE; 
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.pose.position.x = debugCrsPtsMin[n].y;
        marker.pose.position.y = -debugCrsPtsMin[n].x;
        marker.pose.position.z = 0;
        marker.color.a = 1.0;
        marker.color.r = colors[n][0];
        marker.color.g = colors[n][1];
        marker.color.b = colors[n][2];
        marker.id = k;
        markerArray.markers[k++] = marker;
        //text crossPoint
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.3;
        marker.pose.position.z = 2.0+n*0.5;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "xy:("+ std::to_string(debugCrsPtsMin[n].x) +","+ std::to_string(debugCrsPtsMin[n].y)+") \n t:"+std::to_string(debugCrsPtsMin[n].t) + ", dis:"+std::to_string(debugCrsPtsMin[n].dis);
        marker.id = k;
        markerArray.markers[k++] = marker;
        //text ObstaclePoint
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.3;
        marker.pose.position.z = -1.0-n*0.5;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = std::string("state:") + ( (debugCrsPtsMin[n].safe == true) ? "SAFE" : "WARNING");
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    //
    markerArray.markers.resize(k);
    pubDebCPVFHOutput.publish( markerArray );

}   