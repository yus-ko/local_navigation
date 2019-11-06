#include<local_navigation/vfh_tdt.h>
#include <ros/callback_queue.h>
#include <beego_control/beego_encoder.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include<visualization_msgs/MarkerArray.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
#include<local_navigation/vfh_tdtConfig.h>
class run{
    private:
        //受信データ
        ros::NodeHandle nhSub1;
        ros::Subscriber sub1,sub2,sub3;
        local_navigation::ClassificationVelocityData clstr;//速度データ付きのクラスタデータ
        nav_msgs::Odometry robotOdom,goalOdom;
        beego_control::beego_encoder robotEncoder;
        float d;//wheels radius
        float cur_vel, cur_angVel;
        //rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<local_navigation::vfh_tdtConfig> server;
        dynamic_reconfigure::Server<local_navigation::vfh_tdtConfig>::CallbackType f;
        //送信データ
        ros::NodeHandle nhPub;
        ros::Publisher pub;    
        //データチェック用
        bool RECEIVED_CLUSTER;
        bool RECEIVED_GOAL_ODOM;
        bool RECEIVED_ROBOT_ODOM;
        bool RECEIVED_ROBOT_ENCODAR;
        //処理
        vfh_tdt vfhTDT; //
        cost_node best_node;
        float default_vel;
        float target_vel, target_angle;
        //制御
        float gainP;//Pゲイン
        //デバッグパラメータ
        float loopFPS;
        float marker_life_time;
        //表示用publisher
        ros::NodeHandle nhPubDeb;
        ros::Publisher pubDebugClstr,pubDebugNode;    
        //--goal
        float debugGoalPosX;
        float debugGoalPosY;
        //--robot
        float debugCurAng;
        float debugCmd_vel;
        float debugPrevTagAng;
        float debugRobotRadius;
        //--obstacle
        float debugObstacleX1;
        float debugObstacleY1;
        float debugObstacleSize1;
        float debugObstacleVx1;
        float debugObstacleVy1;
        local_navigation::ClassificationVelocityData debug_clstr;
        uint32_t debug_seq;
    public:
        run()
            :RECEIVED_CLUSTER(false),RECEIVED_GOAL_ODOM(false),RECEIVED_ROBOT_ODOM(false),
            debug_seq(0)
        {
            //subscriber
            sub1=nhSub1.subscribe("classificationDataEstimateVelocity",1,&run::cluster_callback,this);
            sub2=nhSub1.subscribe("robotOdometry",1,&run::robotOdom_callback,this);
            sub3=nhSub1.subscribe("goalOdometry",1,&run::goalOdom_callback,this);
            pub= nhPub.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            pubDebugClstr = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_clstr", 1);
            pubDebugNode = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_node", 1);
            //
            //rqt_reconfigure
            f = boost::bind(&run::configCallback, this, _1, _2);
            server.setCallback(f);
            ROS_INFO("ready");
        }
        ~run(){

        }
        void read_launch_file(){
            ros::NodeHandle n("~");
            // 変数定義
            //vfhパラメータ
            float k1_tmp, k2_tmp, k3_tmp; 
            float eta1_tmp, eta2_tmp, eta3_tmp;
            float angle_min_tmp, angle_max_tmp, angle_div_tmp;
            float distance_threshold_tmp;
            float marginRadius_tmp;
            //vfh*TDTパラメータ
            int node_depth;
            float delta_step_tmp;
            //ロボットパラメータ
            float robotRadius_tmp, steer_r_tmp;
            
            //vfhパラメータ
            n.getParam("vfh_tdt/k1",k1_tmp);
            n.getParam("vfh_tdt/k2",k2_tmp);
            n.getParam("vfh_tdt/k3",k3_tmp);
            n.getParam("vfh_tdt/eta1",eta1_tmp);
            n.getParam("vfh_tdt/eta2",eta2_tmp);
            n.getParam("vfh_tdt/eta3",eta3_tmp);
            n.getParam("vfh_tdt/angleMin",angle_min_tmp);
            n.getParam("vfh_tdt/angleMax",angle_max_tmp);
            n.getParam("vfh_tdt/angleDiv",angle_div_tmp);
            n.getParam("vfh_tdt/distanceThreshold",distance_threshold_tmp);
            n.getParam("vfh_tdt/marginRadius",marginRadius_tmp);
            n.getParam("vfh_tdt/node_depth",node_depth);
            n.getParam("vfh_tdt/delta_step",delta_step_tmp);
            //ロボットパラメータ
            n.getParam("vfh_tdt/robotRadius",robotRadius_tmp);
            n.getParam("vfh_tdt/robotSteerRadius",steer_r_tmp);
            n.getParam("vfh_tdt/controllerGainP",gainP);
            n.getParam("vfh_tdt/default_vel",default_vel);
            //デバッグ
            n.getParam("vfh_tdt/loopFPS",loopFPS);
            //セットアップ
            angle_min_tmp *= M_PI/180;
            angle_max_tmp *= M_PI/180;
            angle_div_tmp *= M_PI/180;
            vfhTDT.set_initial_velocity(default_vel);
            vfhTDT.setup_vfh_tdt_param(
                k1_tmp, k2_tmp, k3_tmp, 
                eta1_tmp, eta2_tmp, eta3_tmp,
                angle_min_tmp, angle_max_tmp, angle_div_tmp,
                distance_threshold_tmp, steer_r_tmp,
                robotRadius_tmp, marginRadius_tmp
            );
            d = steer_r_tmp;//エンコーダ->車速 変換用
            // node_depthをセット
            //ラムダをセット
            vfhTDT.set_lamda(node_depth);
            vfhTDT.set_delta_step(delta_step_tmp);
        }
        //コールバッグ関数
        void cluster_callback(const local_navigation::ClassificationVelocityData::ConstPtr& msg){
                //データをコピー
            	clstr = *msg;
                vfhTDT.set_cluster_data(clstr);
                RECEIVED_CLUSTER = false;
                main_loop();
        }
        void robotOdom_callback(const nav_msgs::Odometry::ConstPtr& msg){
            //データをコピー
        	robotOdom = *msg;
            RECEIVED_ROBOT_ODOM = false;
            main_loop();
        }
        void robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg){
            //データをコピー
            robotEncoder = *msg;
            cur_vel = (robotEncoder.vel.r + robotEncoder.vel.l)/2;
            cur_angVel = (robotEncoder.vel.r - robotEncoder.vel.l)/(2 * d);
            RECEIVED_ROBOT_ENCODAR = false;
            main_loop();
        }
        void goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg){
            //データをコピー
        	goalOdom = *msg;
            main_loop();
        }
        //--rqt_reconfigureからの読み込み
        void configCallback(local_navigation::vfh_tdtConfig &config, uint32_t level){

            //vfhパラメータ
            float k1_tmp, k2_tmp, k3_tmp; 
            float eta1_tmp, eta2_tmp, eta3_tmp;
            float angle_min_tmp, angle_max_tmp, angle_div_tmp;
            float distance_threshold_tmp;
            float marginRadius_tmp;
            //vfh*tdtパラメータ
            int node_depth_tmp;
            float delta_step_tmp;
            //ロボットパラメータ
            float robotRadius_tmp, steer_r_tmp;
            
            k1_tmp = config.K1;
            k2_tmp = config.K2;
            k3_tmp = config.K3;
            eta1_tmp = config.EtaG;
            eta2_tmp = config.EtaCurAngle;
            eta3_tmp = config.EtaPrevAngle;
            //
            angle_min_tmp = config.minAngle*M_PI/180;
            angle_max_tmp = config.maxAngle*M_PI/180;
            angle_div_tmp = config.divAngle*M_PI/180;
            distance_threshold_tmp = config.distanceThreshold;
            marginRadius_tmp = config.marginRadius;
            //
            node_depth_tmp = config.node_depth;
            delta_step_tmp = config.delta_step;
            //
            robotRadius_tmp = config.robotRadius;
            steer_r_tmp = config.robotSteerRadius;
            gainP = config.gain_p;
            default_vel = config.default_vel;
            vfhTDT.set_initial_velocity(default_vel);
            vfhTDT.setup_vfh_tdt_param(
                k1_tmp, k2_tmp, k3_tmp, 
                eta1_tmp, eta2_tmp, eta3_tmp,
                angle_min_tmp, angle_max_tmp, angle_div_tmp,
                distance_threshold_tmp, steer_r_tmp,
                robotRadius_tmp, marginRadius_tmp
            );
            // node_depthをセット
            //ラムダをセット
            ROS_INFO("node_depth_tmp - %d",node_depth_tmp); 
            vfhTDT.set_lamda(node_depth_tmp);
            //ds
            vfhTDT.set_delta_step(delta_step_tmp);
            //
            // ROS_INFO_STREAM("defaultVel->"<<config.default_vel);
            // ROS_INFO_STREAM("display->"<<config.displayDebugClstr<<"\n"
            //     <<"debugFlag->"<<config.debugFlag);
            ROS_INFO("display-> %s \n debugFlag-> %s", 
                config.displayDebugClstr? "True":"False",
                config.debugFlag? "True":"False"
            );
            marker_life_time = config.marker_life_time;
            if (config.displayDebugClstr)
            {
                set_debug_parameter(config);//データ取得
                //デバッグクラスタの作成
                create_debug_clstr();
                display_debug_clstr();
            }
            if(config.debugFlag){
                set_debug_parameter(config);//データ取得
                run_debug_process();//処理
                return ;
            }
            // main_loop();
        }
        //メインループ
        void main_loop(){
            if(data_check()){
                data_check_reset();
                process();
            }
        }
        bool data_check(){
            return(RECEIVED_CLUSTER && RECEIVED_GOAL_ODOM && RECEIVED_ROBOT_ODOM && RECEIVED_ROBOT_ENCODAR);
        }
        bool data_check_reset(){
            RECEIVED_CLUSTER = false;
            RECEIVED_ROBOT_ODOM = false;
            RECEIVED_ROBOT_ENCODAR = false;
        }
        //処理
        void process(){
            //スタートノードとゴールノードをセット
            vfhTDT.create_start_node();//
            vfhTDT.create_goal_node(2.0,6.0);//
            //スタートノードをオープンリストに追加
            vfhTDT.add_start_node();
            //A*アルゴリズムで探索を行っていく
            int best_node_num;
            int target_num;
            while(ros::ok()){
                //最小コストとなるノード番号を取得
                int node_num = vfhTDT.get_min_cost_node();
                if(vfhTDT.check_search_finish()){
                    best_node_num = node_num;
                    break;
                }
                //最小コストノードの子ノードを作成
                vfhTDT.add_node(vfhTDT.get_node(0));//先頭ノードを取得
                //
                ROS_INFO("Node:(open,closed):(%d,%d)",vfhTDT.get_open_node_size(), vfhTDT.get_closed_node_size());
            }
            //
            vfhTDT.cobine_open_close_node();
            target_num = vfhTDT.search_node_n(best_node_num);
            //最良ノードを格納
            best_node = vfhTDT.get_node(target_num);
            
            //目標角度, 速度を取得
            target_angle = atan2(best_node.dy,best_node.dx);//best_node.angle;//角度は算出したやつ
            target_vel = default_vel;//速度一定
        }
        geometry_msgs::Twist controler(float& tagVel, float& tagAng){
            //p制御
            double cur_ang = M_PI_2;//正面を向いているため
            float tagAngVel = (tagAng-cur_ang)*gainP;
            //
            geometry_msgs::Twist twist;
            twist.linear.x =tagVel; 
            twist.linear.y =0; 
            twist.linear.z =0;
            twist.angular.x =0;
            twist.angular.y =0;
            twist.angular.z =tagAngVel;

            return twist;
        }
        //デバッグ
        //ノードツリーを表示
        void showNodeTree(){

        }
        //最終的なノードと出力データを表示
        void showOutput(){
            
        }
        //
        void set_debug_parameter(local_navigation::vfh_tdtConfig &config){
            //goal
            debugGoalPosX = config.debugGoalPosX;
            debugGoalPosY = config.debugGoalPosY;
            //robot
            debugCurAng = config.debugCurAng;
            debugCmd_vel = config.debugCmd_vel;
            debugPrevTagAng = config.debugPrevTagAng;
            debugRobotRadius = config.debugRobotRadius;
            //obstacle
            debugObstacleX1 = config.debugObstacleX1;
            debugObstacleY1 = config.debugObstacleY1;
            debugObstacleSize1 = config.debugObstacleSize1;
            debugObstacleVx1 = config.debugObstacleVx1;
            debugObstacleVy1 = config.debugObstacleVy1;
            
        }
        void run_debug_process(){
            ROS_INFO("run_debug_process");
            //clear nodes
            vfhTDT.clear_node();
            //デバッグクラスタの作成
            create_debug_clstr();
            ROS_INFO("create_debug_clstr");
            //スタートノードとゴールノードをセット
            vfhTDT.create_start_node();//
            vfhTDT.create_goal_node(debugGoalPosX,debugGoalPosY);//
            ROS_INFO("create_start_node,create_goal_node");
            //スタートノードをオープンリストに追加
            vfhTDT.add_start_node();
            ROS_INFO("add_start_node");
            //A*アルゴリズムで探索を行っていく
            int best_node_num;
            int target_num;
            ROS_INFO("while");
            while(ros::ok()){
                //最小コストとなるノード番号を取得
                int node_num = vfhTDT.get_min_cost_node();
                // ROS_INFO("get_min_cost_node");
                if(vfhTDT.check_search_finish()){
                    best_node_num = node_num;
                    // ROS_INFO("check_search_finish:true");
                    break;
                }
                // ROS_INFO("check_search_finish:false");
                //最小コストノードの子ノードを作成
                vfhTDT.debug_add_node(vfhTDT.get_node(0), debug_clstr);//先頭ノードを取得
                //
                // ROS_INFO("debug_add_node");
                // ROS_INFO("Node:(open,closed):(%d,%d)",vfhTDT.get_open_node_size(), vfhTDT.get_closed_node_size());

            }
            //
            vfhTDT.cobine_open_close_node();
            ROS_INFO("cobine_open_close_node");
            target_num = vfhTDT.search_node_n(best_node_num);
            ROS_INFO("search_node_n");
            //最良ノードを格納
            best_node = vfhTDT.get_conbine_node(target_num);
            cost_node goalNode = vfhTDT.get_goalNode();
            ROS_INFO("get_best_node");
            std::cout<<"best_node:\n"
                <<"\tnum: "<<best_node.num<<std::endl
                <<"\tdepth: "<<best_node.depth<<std::endl
                <<"\tdx,dy: "<<best_node.dx<<","<<best_node.dy<<std::endl
                <<"\tv,ang: "<<best_node.v<<","<<best_node.angle<<std::endl
                <<"\tangT,angD: "<<best_node.target_angle<<","<<best_node.delta_angle<<std::endl
                <<"\tcost: "<<best_node.cost<<std::endl
                <<"\tgoal: "<<goalNode.dx-best_node.dx<<", "<<goalNode.dy-best_node.dy<<", "<<atan2(goalNode.dy-best_node.dy, goalNode.dx-best_node.dx)<<std::endl
            <<std::endl;
            //目標角度, 速度を取得
            target_angle = atan2(best_node.dy,best_node.dx);//best_node.angle;//角度は算出したやつ
            target_vel= default_vel;//速度一定
            ROS_INFO("v,ang:%f,%f", target_vel,target_angle*180/M_PI);
            //display node
            display_all_node(vfhTDT.get_open_node(), vfhTDT.get_closed_node());
        }
        void create_debug_clstr(){
            // 障害物情報
            // debugObstacleX1 
            // debugObstacleY1 
            // debugObstacleSize1
            // debugObstacleVx1
            // debugObstacleVy1
            debug_clstr.header.frame_id = "base_link";
            debug_clstr.header.seq = debug_seq++;
            debug_clstr.header.stamp = ros::Time::now();
            //クラスタデータ
            ROS_INFO("gc");
            debug_clstr.data.resize(1);
            debug_clstr.data[0].gc.x = debugObstacleX1;
            debug_clstr.data[0].gc.y = debugObstacleY1;
            debug_clstr.data[0].gc.z = 0;
            ROS_INFO("twist");
            debug_clstr.twist.resize(1);
            debug_clstr.twist[0].linear.x = debugObstacleVx1;
            debug_clstr.twist[0].linear.y = debugObstacleVy1;
            debug_clstr.twist[0].linear.z = 0;
            //
            debug_clstr.data[0].size.data = 1;
            debug_clstr.data[0].pt.resize(1);
            debug_clstr.data[0].pt[0].x = debugObstacleX1;
            debug_clstr.data[0].pt[0].y = debugObstacleY1;
            debug_clstr.data[0].pt[0].z = 0;
        }
        void display_debug_clstr(){
            ROS_INFO("display_debug_clstr");
            //マーカー表示
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header= debug_clstr.header;
            marker.ns = "my_namespace";
            marker.lifetime = ros::Duration(marker_life_time);
            // marker.type = visualization_msgs::Marker::ARROW;
            // marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            int marker_size=0;
            for(int i = 0; i<debug_clstr.data.size();i++){
                marker_size += (int)debug_clstr.data[i].pt.size();
            }
            marker_size += (int)debug_clstr.data.size();
            markerArray.markers.resize(marker_size);
            ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
            //
            //
            int count = 0;
            for(int i = 0; i<debug_clstr.data.size();i++){
                std::string clusterNumStr = "cluster: ";
                clusterNumStr = clusterNumStr + std::to_string(i);
                marker.ns = clusterNumStr;
                //text
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.text = "vx,vy:("+ std::to_string(debug_clstr.twist[i].linear.x) +","+ std::to_string(debug_clstr.twist[i].linear.y)+")" ;
                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.3;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.7;
                marker.pose.position.x = debug_clstr.data[i].gc.y;
                marker.pose.position.y = -debug_clstr.data[i].gc.x;
                marker.pose.position.z = debug_clstr.data[i].gc.z + 1.0;
                marker.id = count;
                markerArray.markers[count++] = marker;
                //position 
                double yaw = std::atan2(-debug_clstr.twist[i].linear.x, debug_clstr.twist[i].linear.y);
                if(debug_clstr.twist[i].linear.x==0 && debug_clstr.twist[i].linear.y ==0){
                   marker.type = visualization_msgs::Marker::SPHERE; 
                }
                else{
                    marker.type = visualization_msgs::Marker::ARROW;
                    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                }
                for(int k = 0; k<debug_clstr.data[i].pt.size();k++){
                    marker.pose.position.x = debug_clstr.data[i].pt[k].y;
                    marker.pose.position.y = -debug_clstr.data[i].pt[k].x;
                    marker.pose.position.z = debug_clstr.data[i].pt[k].z;
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
            if(markerArray.markers.size()){
                pubDebugClstr.publish( markerArray );
            }            
        }
        void display_all_node(const std::vector<cost_node>& open_node,const std::vector<cost_node>& closed_node){
            //マーカー表示
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header= debug_clstr.header;
            marker.ns = "my_namespace";
            marker.lifetime = ros::Duration(marker_life_time);
            // marker.type = visualization_msgs::Marker::ARROW;
            // marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            int marker_size=0;
            marker_size = (int)open_node.size()+(int)closed_node.size() + 2;
            markerArray.markers.resize(marker_size);
            ROS_INFO("Node:markerArray.markers.size():%d",(int)markerArray.markers.size());
            //
            //
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            marker.type = visualization_msgs::Marker::ARROW;
            int count = 0;
            for(int i=0;i<open_node.size();i++){
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.scale.x = 0.05;
                marker.scale.y = 0.02;
                marker.scale.z = 0.02;
                std::string namespace_str = "depth: ";
                namespace_str = namespace_str + std::to_string(open_node[i].depth);
                namespace_str = namespace_str + std::string("Open");
                marker.ns = namespace_str;
                marker.pose.position.x = open_node[i].dy;
                marker.pose.position.y = -open_node[i].dx;
                marker.pose.position.z = 0;
                double yaw = open_node[i].angle;
                marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI_2);
                if(best_node.num == open_node[i].num){
                    namespace_str = std::string("best");
                    marker.ns = namespace_str;
                    marker.scale.x = 0.2;
                    marker.scale.y = 0.05;
                    marker.scale.z = 0.05;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                }
                else if(i%(best_node.depth*20)!= 0 ){
                    continue;
                }
                marker.id = count;
                markerArray.markers[count++] = marker;
                
            }
            marker.scale.x = 0.15;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.type = visualization_msgs::Marker::ARROW;
            for(int i=0;i<closed_node.size();i++){
                std::string namespace_str = "depth: ";
                namespace_str = namespace_str + std::to_string(closed_node[i].depth);
                namespace_str = namespace_str + std::string("Closed");
                marker.ns = namespace_str;
                marker.pose.position.x = closed_node[i].dy;
                marker.pose.position.y = -closed_node[i].dx;
                marker.pose.position.z = 0;
                double yaw = closed_node[i].angle;
                marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI_2);
                marker.id = count;
                markerArray.markers[count++] = marker;
                
            }
            //最終的な命令方向を
            marker.scale.x = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.ns =std::string("target_vec");;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            double yaw = target_angle;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI_2);
            marker.id = count;
            markerArray.markers[count++] = marker;
            //
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.ns = "text";
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.text = "v,ang:("+ std::to_string(target_vel) +","+ std::to_string(target_angle*180/M_PI)+")" ;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.3;
            marker.pose.position.z =- 0.5;
            marker.id = count;
            markerArray.markers[count++] = marker;
            
            //
            markerArray.markers.resize(count);
            ROS_INFO("Comp:markerArray.markers.size():%d",(int)markerArray.markers.size());
            if(markerArray.markers.size()){
                pubDebugNode.publish( markerArray );
            }   
        }
};

int main(int argc,char **argv){
	ros::init(argc,argv,"vfh_tdt_node");
    
    ROS_INFO("constructer");
    run runCls;
    ros::spin();
	//--process
    
    //--
	return 0;
}