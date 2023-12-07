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
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)clstr.data.size() * 3);
    int count = 0;
    //交差位置を取得
    std::vector<crossPoint> crsPts;
	crsPts.resize((int)clstr.data.size()*2);
    ROS_INFO("cur,cmd,ang : %f,%f,%f",debugCur_vel,debugCmd_vel,debugCmd_angle );
	// ROS_INFO_STREAM("in debug"<<":"<<debugCur_vel<<", "<<debugCur_angle_steer<<", "<<debugCmd_vel<<", "<<debugCmd_angle);
    crossPointsDetect(crsPts,debugCur_vel, debugCur_angle_steer, debugCmd_vel,debugCmd_angle);//rqt_reconfiureの値を使用
    
    for(int k=0; k<clstr.data.size(); k++){
        ROS_INFO("obst X(x,y), v(x,y) : X(%f,%f),v(%f,%f)",clstr.data[k].gc.x, clstr.data[k].gc.y, clstr.twist[k].linear.x,clstr.twist[k].linear.y );
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.3;//debugObstacleRadius*2+abs(clstr.twist[k].linear.y);
        marker.scale.y = 0.1;//debugObstacleRadius*2+abs(-clstr.twist[k].linear.x);
        marker.scale.z = 0.1;
        // local -> rviz 
        marker.pose.position.x = clstr.data[k].gc.y;
        marker.pose.position.y = -clstr.data[k].gc.x;
        marker.pose.position.z = clstr.data[k].gc.z;
        //angle
        double yaw = std::atan2(-clstr.twist[k].linear.x, clstr.twist[k].linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        //
        marker.id = count;
        markerArray.markers[count++] = marker;
        
        marker.type = visualization_msgs::Marker::SPHERE;
        //position
        //定義済みの交差位置構造体から取得
	    marker.color.a = 1.0;
        marker.color.r = colors[k][0];
        marker.color.g = colors[k][1];
        marker.color.b = colors[k][2];
        marker.pose.position.z = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        
        crossPoint crsPt = crsPts[k*2];
        std::cout<<"crsPt["<<k*2<<"]:("<<crsPts[k*2].x<<","<<crsPts[k*2].y<<","<<crsPts[k*2].t<<","<<crsPts[k*2].vx<<","<<crsPts[k*2].vy<<","<<crsPts[k*2].safe<<std::endl;
        //危険, 安全障害物ともに同じように表示している
        marker.pose.position.x = crsPt.y;
        marker.pose.position.y = -crsPt.x;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;

        crsPt = crsPts[k*2+1];
        std::cout<<"crsPt["<<k*2+1<<"]:("<<crsPts[k*2+1].x<<","<<crsPts[k*2+1].y<<","<<crsPts[k*2+1].t<<","<<crsPts[k*2+1].vx<<","<<crsPts[k*2+1].vy<<","<<crsPts[k*2+1].safe<<std::endl;
        //危険, 安全障害物ともに同じように表示している
        marker.pose.position.x = crsPt.y;
        marker.pose.position.y = -crsPt.x;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
        
    }
    markerArray.markers.resize(count);
    ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
    if(markerArray.markers.size()){
        pubDebMarkerArray.publish( markerArray );
    }
}
//出力と交差位置の表示
void obstacleAvoidance::showOutPut(std::vector<crossPoint>& crsPts, float v, int num){
    //--sample
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = clstr.header.stamp;
    marker.ns = "my_namespace";
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)crsPts.size() + (int)clstr.data.size() + 2+10 +1);
    int count = 0;
    for(int k=0; k<clstr.data.size(); k++){
        marker.ns = "obstacle_vec";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.3;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        // local -> rviz 
        marker.pose.position.x = clstr.data[k].gc.y;
        marker.pose.position.y = -clstr.data[k].gc.x;
        marker.pose.position.z = clstr.data[k].gc.z;
        //angle
        double yaw = std::atan2(-clstr.twist[k].linear.x, clstr.twist[k].linear.y);
        if(clstr.twist[k].linear.x==0 && clstr.twist[k].linear.y ==0){
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.5;   
            yaw = 0;
        }
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        //
        marker.id = count;
        markerArray.markers[count++] = marker;
        marker.ns = "obstacle_pos";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.3;//debugObstacleRadius*2+abs(clstr.twist[k].linear.y);
        marker.scale.y = 0.2;//debugObstacleRadius*2+abs(-clstr.twist[k].linear.x);
        marker.scale.z = 0.2;
        //position
        //定義済みの交差位置構造体から取得
        if((int)crsPts.size() == 0){
            continue;
        }
	    marker.color.a = 1.0;
        marker.color.r = colors[k][0];
        marker.color.g = colors[k][1];
        marker.color.b = colors[k][2];
        marker.pose.position.z = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.ns = "cross_points";
        crossPoint crsPt = crsPts[k*2];
        if(!crsPt.safe){
            marker.color.r =1.0;
            marker.color.g = 0;
            marker.color.b = 0;
            std::cout<<"crsPt["<<k*2<<"]:("<<crsPts[k*2].x<<","<<crsPts[k*2].y<<","<<crsPts[k*2].t<<","<<crsPts[k*2].vx<<","<<crsPts[k*2].vy<<std::endl;
        }
        //危険, 安全障害物ともに同じように表示している
        marker.pose.position.x = crsPt.y;
        marker.pose.position.y = -crsPt.x;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
        crsPt = crsPts[k*2+1];
        if(!crsPt.safe){
            marker.color.r =1.0;
            marker.color.g = 0;
            marker.color.b = 0;
            std::cout<<"crsPt["<<k*2+1<<"]:("<<crsPts[k*2+1].x<<","<<crsPts[k*2+1].y<<","<<crsPts[k*2+1].t<<","<<crsPts[k*2+1].vx<<","<<crsPts[k*2+1].vy<<std::endl;
        }
        //危険, 安全障害物ともに同じように表示している
        marker.pose.position.x = crsPt.y;
        marker.pose.position.y = -crsPt.x;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;

    }
    marker.ns = "robot";
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    //ロボットの命令値の出力を表示
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 122;
    double tagyaw = (angle_min + num * angle_div)*M_PI/180;
    // ROS_INFO("yaw:%f",yaw);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(tagyaw-M_PI_2);
    marker.id = count;
    markerArray.markers[count++] = marker;
    //目標矢印（テキスト）
    marker.ns = "text";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "v,ang:("+ std::to_string(v) +","+ std::to_string(tagyaw*180/M_PI)+")" ;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = 0.5;
    marker.id = count;
    markerArray.markers[count++] = marker;
    //
    //ゴール
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.ns = "goal";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.position.x = relationOdom.pose.pose.position.x;
    marker.pose.position.y = relationOdom.pose.pose.position.y;
    marker.pose.position.z = relationOdom.pose.pose.position.z;
    marker.id = count;
    markerArray.markers[count++] = marker;

    //
    markerArray.markers.resize(count);
    if(markerArray.markers.size()){
        pubDebBagOutput.publish( markerArray );
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
    int num=0;
    //実際の環境下では相対ベクトルが得られるため
    geometry_msgs::Twist relation_vel = debugTwistRef;
    // ROS_INFO("Origin: Vo(%f,%f)",debugTwistRef.linear.x,debugTwistRef.linear.y);
    relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
    relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);
    // ROS_INFO("Relation: Vo(%f,%f)",relation_vel.linear.x,relation_vel.linear.y);
             std::vector<crossPoint> crsPtTemp;
    crsPtTemp.resize(2);
    getCrossPoints(crsPtTemp[0],crsPtTemp[1], debugIndexRef, debugGpRef,relation_vel,debugCur_vel,debugCur_angle_steer, debugCmd_vel,debugCmd_angle);

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
    crossPoint crsPt = crsPtTemp[0];
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
    crsPt = crsPtTemp[1];
    ROS_INFO("t_cross:%f",crsPt.t);
    //危険, 安全障害物ともに同じように表示している
    //
    marker.scale.x = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.y = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.z = 0.1;
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
                    + "state:" + ( (crsPt.safe == true) ? "SAFE" : "WARNING");
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
	vfh_check.set_eta(debugEtaG, debugEtaCurAngle, debugEtaPrevAngle);
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
//ヒストグラムの出力結果を視覚的に表示する
void obstacleAvoidance::histgramCheckerEx(){
     //距離ヒストグラム
    std::vector<double> histgram_dis;
    vfh_c.get_histgram_dis(histgram_dis);
    //バイナリヒストグラム
    std::vector<bool> histgram_bi;
    vfh_c.get_histgram_bi(histgram_bi);
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
        
        double l = dis_th;
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
	vfh_check.set_eta(debugEtaG, debugEtaCurAngle, debugEtaPrevAngle);
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
    double sum_weight = debugKg + debugKcurAngle + debugKprevAngle;
    debugKg/=sum_weight;
    debugKcurAngle/=sum_weight;
    debugKprevAngle/=sum_weight;
    // ROS_INFO("goal,angle,angleVel: %f, %f, %f", debugKg, debugKcurAngle, debugKprevAngle);
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
        

        double goalCost = vfh_check.cost_goal_angle_deg(ang, debugGoalAng);
        double angCost = vfh_check.cost_current_angle_deg(ang, debugCurAng);
        double prevAngCost = vfh_check.cost_prev_select_angle_deg(ang, debugPrevTagAng);
        double cost = debugKg * goalCost + debugKcurAngle * angCost + debugKprevAngle * prevAngCost;
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
        

        float goalCost = vfh_check.cost_goal_angle_deg(ang, debugGoalAng);
        float angCost = vfh_check.cost_current_angle_deg(ang, debugCurAng);
        float prevAngCost = vfh_check.cost_prev_select_angle_deg(ang, debugPrevTagAng);
        double cost = debugKg * goalCost + debugKcurAngle * angCost + debugKprevAngle * prevAngCost;
        
        marker.color.r = ((int)(goalCost/max_cost*255))*debugKg;
        marker.color.g = ((int)(angCost/max_cost*255))*debugKcurAngle;
        marker.color.b = ((int)(prevAngCost/max_cost*255))*debugKprevAngle;
        // std::cout<< i<<": " <<marker.color.r<<", "<<marker.color.g<<", "<<marker.color.b<<std::endl;
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
	vfh_check.set_eta(debugEtaG, debugEtaCurAngle, debugEtaPrevAngle);
    
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
    double sum_weight = debugKg + debugKcurAngle + debugKprevAngle + debugKcp;
    debugKg/=sum_weight;
    debugKcurAngle/=sum_weight;
    debugKprevAngle/=sum_weight;
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
        ////障害物との交差位置を算出(障害物の重心位置のみを使用)
        //交差位置を取得
        std::vector<crossPoint> debugCrsPts;
        debugCrsPts.clear();
        debugCrsPts.resize(2*3);
        int ptNum =0;
        int num=0;
        if(!((debugTwist1.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum, debugGp1,debugTwist1,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist1;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp1,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;
        }
        if(!((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum,z debugGp2,debugTwist2,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist2;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp2,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;            
        }
        if(!((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum, debugGp3,debugTwist3,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist3;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp3,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;            
            
        }
        debugCrsPts.resize(num);
        //
        double goalCost = vfh_check.cost_goal_angle_deg(ang, debugGoalAng);
        double angCost = vfh_check.cost_current_angle_deg(ang, debugCurAng);
        double prevAngCost = vfh_check.cost_prev_select_angle_deg(ang, debugPrevTagAng);
        double crossCost = getCrossPointCost(debugCrsPts,debugEtaCp);
        double cost = debugKg * goalCost + debugKcurAngle * angCost + debugKprevAngle * prevAngCost + debugKcp*crossCost;
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
        ////障害物との交差位置を算出(障害物の重心位置のみを使用)
        //交差位置を取得
        std::vector<crossPoint> debugCrsPts;
        debugCrsPts.clear();
        debugCrsPts.resize(3*2);
        int num=0;
        if(!((debugTwist1.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum, debugGp1,debugTwist1,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist1;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp1,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;
        }
        if(!((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum,z debugGp2,debugTwist2,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist2;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp2,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;            
        }
        if(!((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum, debugGp3,debugTwist3,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist3;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp3,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;            
            
        }
        debugCrsPts.resize(num);
        //
        double goalCost = vfh_check.cost_goal_angle_deg(ang, debugGoalAng);
        double angCost = vfh_check.cost_current_angle_deg(ang, debugCurAng);
        double prevAngCost = vfh_check.cost_prev_select_angle_deg(ang, debugPrevTagAng);
        double crossCost = getCrossPointCost(debugCrsPts, debugEtaCp);
        double cost = debugKg * goalCost + debugKcurAngle * angCost + debugKprevAngle * prevAngCost + debugKcp*crossCost;
        // std::cout<< i<<": " <<debugKg * goalCost<<", "<<debugKcurAngle* angCost <<", "<<debugKprevAngle * prevAngCost<<","<<debugKcp*crossCost<<std::endl;
        //コストでグラデーション
        marker.color.a = 1.0;
        marker.color.r = ((uint8_t)(goalCost/max_cost*255*50))*debugKg;//ゴール角度
        marker.color.g = ((uint8_t)(angCost/max_cost*255*50))*debugKcurAngle;//現在角度
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
        // std::cout<< i<<": " <<marker.pose.position.x<<", "<<marker.pose.position.y<<", "<<marker.pose.position.z<<std::endl;
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
//回転による障害物速度の変化量を回転変化前と変化後を比較することで行う
void obstacleAvoidance::rotationVelocityChecker(double omega){
    //clstr: 障害物クラスタ（ロボット座標系(時刻t)）
    //rotClstr: 障害物クラスタ（ロボット座標系(時刻t)）
    //
    //平行移動
    //X_para = X - X_base (X ={x,y,th,vx,vy})
    //しかし,今回はX_baseがロボット座標であるため, 
    //X_para = X
    //となる
    //X_baseは基準座標であり, 今回変更したい座標の基準であるため
    //X_base={0,0,M_PI_2,0,0} (ロボット座標系)
    //
    //回転移動
    //X_rot = R(-delta_theta) X_para
    //
    //これにより, ロボット回転による障害物の速度が算出される
    //計算をロボット座標系基準で行う
    //クラスタのコピー
    rotClstr = clstr;
    //
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = clstr.header.stamp;
    marker.lifetime = ros::Duration(0.3);
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)clstr.data.size()*5);
    
    int count = 0;
    for(int k=0;k < clstr.data.size();k++){
        double v_rot_x, v_rot_y;
        double x_para_x = clstr.data[k].gc.x;
        double x_para_y = clstr.data[k].gc.y;
        //double x_para_theta = std::atan2(clstr.data[k].gc.y,clstr.data[k].gc.x)-M_PI_2;
        //
        double delta_r,delta_p,delta_yaw;
        tf::Quaternion quat_rot;
        quaternionMsgToTF(deltaRobotOdom.pose.pose.orientation, quat_rot);
        tf::Matrix3x3(quat_rot).getRPY(delta_r,delta_p,delta_yaw);
        //
        double x_para_theta = delta_yaw;//std::atan2(gpRef.y,gpRef.x)-M_PI_2;
        
        double x_para_vx = clstr.twist[k].linear.x;
        double x_para_vy = clstr.twist[k].linear.y;
        //回転算出
        // debug_trans_rotation_vel(v_rot_x,v_rot_y,x_para_x,x_para_y,x_para_theta,x_para_vx, x_para_vy,omega);
        //        
        double pos_x = clstr.data[k].gc.x;
        double pos_y = clstr.data[k].gc.y;
        double vel_x = clstr.twist[k].linear.x;
        double vel_y = clstr.twist[k].linear.y;
        double vr_x = cur_vel * cos(cur_angVel*delta_time+M_PI_2);//*delta_time);
        double vr_y = cur_vel * sin(cur_angVel*delta_time+M_PI_2);//*delta_time);
        double delta_pos_x = pos_x + cos(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + sin(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
        double delta_pos_y = pos_y - sin(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + cos(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
        // 速度
        float Vox = delta_pos_x/delta_time;
        float Voy = delta_pos_y/delta_time;
        //
        // rotClstr.twist[k].linear.x -= v_rot_x;
        // rotClstr.twist[k].linear.y -= v_rot_y;
        // rotClstr.twist[k].linear.x += v_rot_x;
        // rotClstr.twist[k].linear.y += v_rot_y;
        // rotClstr.twist[k].linear.x = v_rot_x + clstr.twist[k].linear.x;
        // rotClstr.twist[k].linear.y = v_rot_y + clstr.twist[k].linear.y;
        rotClstr.twist[k].linear.x = Vox;
        rotClstr.twist[k].linear.y = Voy;
        //
        marker.ns = "obstacle_vec_self";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.3;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        // local -> rviz 
        marker.pose.position.x = clstr.data[k].gc.y;
        marker.pose.position.y = -clstr.data[k].gc.x;
        marker.pose.position.z = clstr.data[k].gc.z;
        //angle
        double yaw = std::atan2(-clstr.twist[k].linear.x, clstr.twist[k].linear.y);
        // if(clstr.twist[k].linear.x==0 && clstr.twist[k].linear.y ==0){
        //     marker.type = visualization_msgs::Marker::SPHERE;
        //     marker.scale.x = 0.3;
        //     marker.scale.y = 0.3;
        //     marker.scale.z = 0.5;
        //     yaw = 0;
        // }

        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.id = count;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        markerArray.markers[count++] = marker;
        //
        marker.ns = "obstacle_difVel_self";
        std::cout<<k<<","<<x_para_theta*180/M_PI<<":bef,rot:("<<clstr.twist[k].linear.x<<","<<clstr.twist[k].linear.y<<"),("<<rotClstr.twist[k].linear.x<<","<<rotClstr.twist[k].linear.y<<")"<<std::endl;
        double yawRot = std::atan2(-rotClstr.twist[k].linear.x, rotClstr.twist[k].linear.y);
        std::cout<<k<<","<<x_para_theta*180/M_PI<<":bef,rot:("<<std::atan2(clstr.twist[k].linear.y, clstr.twist[k].linear.x)<<","<<std::atan2(rotClstr.twist[k].linear.y, rotClstr.twist[k].linear.x)<<std::endl;
        //
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yawRot);
        marker.id = count;
        marker.pose.position.z = clstr.data[k].gc.z+0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0;
        marker.color.b = 0;
        markerArray.markers[count++] = marker;
        //
        //rot
        marker.ns = "obstacle_rotVel_self";
        double yawRotVel = std::atan2(-v_rot_x, v_rot_y);
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yawRotVel);
        marker.id = count;
        marker.pose.position.z = clstr.data[k].gc.z+0.4;
        marker.scale.x = 0.3;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1;
        marker.color.b = 0;
        markerArray.markers[count++] = marker;
        marker.ns = "obstacle_rotVel_text";
        //text rotVel
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.4;
        marker.pose.position.z = clstr.data[k].gc.z+1.0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "xy:("+ std::to_string(v_rot_x) +","+ std::to_string(v_rot_y)+")" ;
        marker.id = count;
        markerArray.markers[count++] = marker;
    }
    //
    ROS_INFO("pubDebRotOutput: %d",count );
    markerArray.markers.resize(count);
    if(markerArray.markers.size()){
        pubDebRotOutput.publish( markerArray );
    }
}
void obstacleAvoidance::trans_rotation_vel(double& v_rot_x, double& v_rot_y, const double& x_para_x,const double& x_para_y,const double& x_para_vx,const double& x_para_vy){
    //変数作成
    double theta_base = M_PI_2;//正面方向
    double omega_base;
    //tf座標系とヨー方向回転軸は同じため,自己位置推定結果を使用
    //
    omega_base = deltaRobotOdom.twist.twist.angular.z;
    //計算
    v_rot_x = omega_base * (-sin(theta_base)*x_para_x - cos(theta_base)*x_para_y)
            + cos(theta_base)*x_para_vx - sin(theta_base)*x_para_vy;
    v_rot_y = omega_base * (-sin(theta_base)*x_para_x - cos(theta_base)*x_para_y)
            - sin(theta_base)*x_para_vx + cos(theta_base)*x_para_vy;
}
void obstacleAvoidance::debug_trans_rotation_vel(double& v_rot_x, double& v_rot_y, const double& x_para_x,const double& x_para_y,const double& x_para_theta,const double& x_para_vx,const double& x_para_vy, const double& omega_base){
    //変数作成
    // double theta_base = M_PI_2;//正面方向がベース
    //計算
    // v_rot_x = omega_base * (-sin(theta_base)*x_para_x - cos(theta_base)*x_para_y)
    //         + cos(theta_base)*x_para_vx - sin(theta_base)*x_para_vy;
    // v_rot_y = omega_base * (-sin(theta_base)*x_para_x - cos(theta_base)*x_para_y)
    //         - sin(theta_base)*x_para_vx + cos(theta_base)*x_para_vy;
    v_rot_x = omega_base * (-sin(x_para_theta)*x_para_x - cos(x_para_theta)*x_para_y)
            + cos(x_para_theta)*x_para_vx - sin(x_para_theta)*x_para_vy;
    v_rot_y = omega_base * (cos(x_para_theta)*x_para_x - sin(x_para_theta)*x_para_y)
            + sin(x_para_theta)*x_para_vx + cos(x_para_theta)*x_para_vy;
}