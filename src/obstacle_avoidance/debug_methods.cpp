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
    for(int k=0; k<clstr.data.size(); k++){
        float obstacleRadius = 1.0;
        marker.scale.x = obstacleRadius;
        marker.scale.y = obstacleRadius;
        marker.scale.z = obstacleRadius;
        //position
        //定義済みの交差位置構造体から取得
	    crossPoint crsPt;//仮
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
        pubDebMarker.publish( markerArray );
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
