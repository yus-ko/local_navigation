#include<local_navigation/simulater.h>
#include<visualization_msgs/MarkerArray.h>

class run_simulater : public simulater
{
    private:
        ros::NodeHandle nhPubDeb;
        ros::Publisher pubDeb1,pubDeb2,pubDeb3;
        
    public:
        run_simulater(){
            //
            pubDeb1 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_clstr", 1);
            pubDeb2 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_detected_clstr", 1);
            pubDeb3 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_odometries", 1);
            //
            if(SET_CONFIG){
                createObstacleData();
                createRobotData();
                createGoalOdom(goal_x,goal_y);
            }
        };
        ~run_simulater(){
            
        };
        bool get_set_config(){
            return SET_CONFIG;
        }
        bool get_received_cmd_vel(){
            return RECEIVED_CMD_VEl;
        }
        void reset_received_cmd_vel(){
            RECEIVED_CMD_VEl = false;
        }
        void run(){
            //受け取りフラグのリセット
            reset_received_cmd_vel();
            //データ更新
            updateRobotEncoder();
            updateObstacleState();
            updateRobotState();
            //視野角反映
            detectObstacle();
            //出力
            publishObstacleData();
            publishRobotData();
            publishTargetData();
            bool display_output = true;
            if(display_output){
                showOdometries();
                showClusters();
                showDetectedClusters();
            }
        }
        void showOdometries(){//pubDeb3
            //マーカー表示
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header= cur_robotOdom.header;
            marker.ns = "my_namespace";
            // marker.lifetime = ros::Duration(1.0);
            // marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            //
            markerArray.markers.resize(1+1);
            int count = 0;

            //ロボット自己位置
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.ns = "cur_robotOdom";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.15;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.pose.position.x = cur_robotOdom.pose.pose.position.x;
            marker.pose.position.y = cur_robotOdom.pose.pose.position.y;
            marker.pose.position.z = cur_robotOdom.pose.pose.position.z;
            marker.id = count;
            markerArray.markers[count++] = marker;
            //ゴール
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.ns = "goalOdom";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.pose.position.x = goalOdom.pose.pose.position.x;
            marker.pose.position.y = goalOdom.pose.pose.position.y;
            marker.pose.position.z = goalOdom.pose.pose.position.z;
            marker.id = count;
            markerArray.markers[count++] = marker;
            //
            //リサイズpublish
            markerArray.markers.resize(count);
            ROS_INFO("odom:markerArray.markers.size():%d",(int)markerArray.markers.size());
            if(markerArray.markers.size()){
                pubDeb3.publish( markerArray );
            }   

        }
        void showClusters(){//pubDeb1
            //マーカー表示
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header= cur_clstr.header;
            marker.ns = "my_namespace";
            // marker.lifetime = ros::Duration(1.0);
            // marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            //
            int marker_size=0;
            for(int i = 0; i<cur_clstr.data.size();i++){
                marker_size += (int)cur_clstr.data[i].pt.size();
            }
            marker_size += (int)cur_clstr.data.size();
            markerArray.markers.resize(marker_size);
            int count = 0;
            for(int i = 0; i<cur_clstr.data.size();i++){
                std::string clusterNumStr = "cluster: ";
                clusterNumStr = clusterNumStr + std::to_string(i);
                marker.ns = clusterNumStr;
                //text
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.text = "vx,vy:("+ std::to_string(cur_clstr.twist[i].linear.x) +","+ std::to_string(cur_clstr.twist[i].linear.y)+")" ;
                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.3;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.7;
                marker.pose.position.x = cur_clstr.data[i].gc.y;
                marker.pose.position.y = -cur_clstr.data[i].gc.x;
                marker.pose.position.z = cur_clstr.data[i].gc.z + 1.0;
                marker.id = count;
                markerArray.markers[count++] = marker;
                //position 
                double yaw = std::atan2(-cur_clstr.twist[i].linear.x, cur_clstr.twist[i].linear.y);
                if(cur_clstr.twist[i].linear.x==0 && cur_clstr.twist[i].linear.y ==0){
                   marker.type = visualization_msgs::Marker::SPHERE; 
                }
                else{
                    marker.type = visualization_msgs::Marker::ARROW;
                    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                }
                for(int k = 0; k<cur_clstr.data[i].pt.size();k++){
                    marker.pose.position.x = cur_clstr.data[i].pt[k].y;
                    marker.pose.position.y = -cur_clstr.data[i].pt[k].x;
                    marker.pose.position.z = cur_clstr.data[i].pt[k].z;
                    marker.color.a = 1.0;
                    marker.color.r = 255;
                    marker.color.g = 255;
                    marker.color.b = 255;
                    marker.scale.x = 0.1;
                    marker.scale.y = 0.05;
                    marker.scale.z = 0.05;
                    marker.id = count;
                    markerArray.markers[count++] = marker;
                }
            }
            //リサイズpublish
            markerArray.markers.resize(count);
            ROS_INFO("odom:markerArray.markers.size():%d",(int)markerArray.markers.size());
            if(markerArray.markers.size()){
                pubDeb1.publish( markerArray );
            }   
            
        }
        void showDetectedClusters(){//pubDeb2
            //マーカー表示
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header= detected_clstr.header;
            marker.ns = "my_namespace";
            // marker.lifetime = ros::Duration(1.0);
            // marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            int marker_size=0;
            for(int i = 0; i<detected_clstr.data.size();i++){
                marker_size += (int)detected_clstr.data[i].pt.size();
            }
            marker_size += (int)detected_clstr.data.size();
            markerArray.markers.resize(marker_size);
            ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
            //
            //
            int count = 0;
            for(int i = 0; i<detected_clstr.data.size();i++){
                std::string clusterNumStr = "cluster: ";
                clusterNumStr = clusterNumStr + std::to_string(i);
                marker.ns = clusterNumStr;
                //text
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.text = "vx,vy:("+ std::to_string(detected_clstr.twist[i].linear.x) +","+ std::to_string(detected_clstr.twist[i].linear.y)+")" ;
                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.3;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.7;
                marker.pose.position.x = detected_clstr.data[i].gc.y;
                marker.pose.position.y = -detected_clstr.data[i].gc.x;
                marker.pose.position.z = detected_clstr.data[i].gc.z + 1.0;
                marker.id = count;
                markerArray.markers[count++] = marker;
                //position 
                double yaw = std::atan2(-detected_clstr.twist[i].linear.x, detected_clstr.twist[i].linear.y);
                if(detected_clstr.twist[i].linear.x==0 && detected_clstr.twist[i].linear.y ==0){
                   marker.type = visualization_msgs::Marker::SPHERE; 
                }
                else{
                    marker.type = visualization_msgs::Marker::ARROW;
                    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                }
                for(int k = 0; k<detected_clstr.data[i].pt.size();k++){
                    marker.pose.position.x = detected_clstr.data[i].pt[k].y;
                    marker.pose.position.y = -detected_clstr.data[i].pt[k].x;
                    marker.pose.position.z = detected_clstr.data[i].pt[k].z;
                    marker.color.a = 1.0;
                    marker.color.r = 255;
                    marker.color.g = 255;
                    marker.color.b = 255;
                    marker.scale.x = 0.1;
                    marker.scale.y = 0.05;
                    marker.scale.z = 0.05;
                    marker.id = count;
                    markerArray.markers[count++] = marker;
                }
            }           
            //リサイズpublish
            markerArray.markers.resize(count);
            ROS_INFO("odom:markerArray.markers.size():%d",(int)markerArray.markers.size());
            if(markerArray.markers.size()){
                pubDeb2.publish( markerArray );
            }             
        }
};


int main(int argc,char **argv){
	ros::init(argc,argv,"simulater_node");
    
    ROS_INFO("constructer");
    run_simulater run_sim;
    //--process
    while(ros::ok()){
        ros::spinOnce();
        if(run_sim.get_received_cmd_vel() && run_sim.get_set_config()){
            run_sim.run();
        }
	}
    //--
	return 0;
}