#include<local_navigation/vfh_tdt.h>
#include <ros/callback_queue.h>
#include <beego_control/beego_encoder.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include<visualization_msgs/MarkerArray.h>
#include<std_msgs/Empty.h>
#include <ros/callback_queue.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
#include<local_navigation/vfh_tdtConfig.h>
class run{
    private:
        //受信データ
        ros::NodeHandle nhSub1;
        ros::Subscriber sub1,sub2,sub3,sub4;
        local_navigation::ClassificationVelocityData clstr;//速度データ付きのクラスタデータ
        nav_msgs::Odometry robotOdom,goalOdom,relationOdom;
        beego_control::beego_encoder robotEncoder;
        float goal_x,goal_y;
        float d;//wheels radius
        float cur_vel, cur_angVel;
        ros::Time cur_time,pre_time;
        ros::Duration delta_time_ros;
        double delta_time;
        bool PROCESS_ONCE;
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
        nav_msgs::Odometry debugRobotOdom,pre_robotOdom,deltaRobotOdom,debugGoalOdom,debugRelationOdom;
        float debugRobotYaw;
        float loopFPS;
        float marker_life_time;
        float display_fps;
        bool display_searching_process;
        int sparse_rate;
        std::vector<int> debugNextNodeIndex;
        //表示用publisher
        ros::NodeHandle nhPubDeb;
        ros::Publisher pubDebugClstr,pubDebugNode;    
        //--goal
        float debugGoalPosX;
        float debugGoalPosY;
        //--robot
        float debugRobotX1;
        float debugRobotY1;
        float debugCurAng;
        float debugCmd_vel;
        float debugPrevTagAng;
        float debugRobotRadius;
        //--obstacle
        float debugObstacleX1;
        float debugObstacleY1;
        float debugObstacleW1;
        float debugObstacleH1;
        float debugObstacleVx1;
        float debugObstacleVy1;
        float debugObstacleX2;
        float debugObstacleY2;
        float debugObstacleW2;
        float debugObstacleH2;
        float debugObstacleVx2;
        float debugObstacleVy2;
        local_navigation::ClassificationVelocityData debug_clstr;
        uint32_t debug_seq;
    public:
        run()
            :RECEIVED_CLUSTER(false),RECEIVED_GOAL_ODOM(false),RECEIVED_ROBOT_ODOM(false),
            debug_seq(0),PROCESS_ONCE(false)
        {
            //subscriber
            sub1=nhSub1.subscribe("classificationDataEstimateVelocity",1,&run::cluster_callback,this);
            sub2=nhSub1.subscribe("zed_node/odom",1,&run::robotOdom_callback,this);
            sub3=nhSub1.subscribe("goalOdometry",1,&run::goalOdom_callback,this);
            sub4=nhSub1.subscribe("/encoder",1,&run::robotEncoder_callback,this);
            pub= nhPub.advertise<geometry_msgs::Twist>("/beego/cmd_vel", 1);
            pubDebugClstr = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_clstr", 1);
            pubDebugNode = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_node", 1);
            //
            read_launch_file();
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
            //ゴール位置
            if(n.getParam("vfh_tdt/goalX",goal_x) && n.getParam("vfh_tdt/goalY",goal_y) ){
                RECEIVED_GOAL_ODOM = true;
                goalOdom.pose.pose.position.x = goal_x;
                goalOdom.pose.pose.position.y = goal_y;
                goalOdom.pose.pose.position.z = 0;
                tf::Quaternion quat=tf::createQuaternionFromYaw(M_PI_2);
                quaternionTFToMsg(quat, goalOdom.pose.pose.orientation);
            }
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
                RECEIVED_CLUSTER = true;
                main_process();
        }
        void robotOdom_callback(const nav_msgs::Odometry::ConstPtr& msg){
            //データをコピー
        	robotOdom = *msg;
            RECEIVED_ROBOT_ODOM = true;
            main_process();
        }
        void robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg){
            //データをコピー
            robotEncoder = *msg;
            cur_vel = ((-robotEncoder.vel.r)  + robotEncoder.vel.l)/2;
            cur_angVel = ((-robotEncoder.vel.r)  - robotEncoder.vel.l)/( d);
            RECEIVED_ROBOT_ENCODAR = true;
            main_process();
        }
        void goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg){
            //データをコピー
        	goalOdom = *msg;
            RECEIVED_GOAL_ODOM = true;
            main_process();
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
            display_fps = config.display_fps;
            display_searching_process = config.display_searching_process;
            sparse_rate = config.sparse_rate;
            if (config.displayDebugClstr)
            {
                set_debug_parameter(config);//データ取得
                //デバッグクラスタの作成
                create_debug_clstr();
                display_debug_clstr();
            }
            if(config.debugFlag){
                set_debug_parameter(config);//データ取得
                update_debug_goal_position();
                run_debug_process();//処理
                return ;
            }
            // main_process();
        }
        void culc_delta_robotOdom(){
            deltaRobotOdom.pose.pose.position.x = robotOdom.pose.pose.position.x - pre_robotOdom.pose.pose.position.x;
            deltaRobotOdom.pose.pose.position.y = robotOdom.pose.pose.position.y - pre_robotOdom.pose.pose.position.y;
            deltaRobotOdom.pose.pose.position.z = robotOdom.pose.pose.position.z - pre_robotOdom.pose.pose.position.z;
            deltaRobotOdom.twist.twist.linear.x = (robotOdom.pose.pose.position.x - pre_robotOdom.pose.pose.position.x ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
            deltaRobotOdom.twist.twist.linear.y = (robotOdom.pose.pose.position.y - pre_robotOdom.pose.pose.position.y ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
            deltaRobotOdom.twist.twist.linear.z = (robotOdom.pose.pose.position.z - pre_robotOdom.pose.pose.position.z ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
            // deltaRobotOdom = robotOdom - pre_robotOdom;
            //回転
            tf::Quaternion quat;
            double r_cur,p_cur,y_cur;
            quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(r_cur,p_cur,y_cur);
            double r_pre,p_pre,y_pre;
            quaternionMsgToTF(pre_robotOdom.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(r_pre,p_pre,y_pre);
            double delta_theta = y_cur - y_pre;
            double omega = delta_theta/( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
            //
            tf::Quaternion quatTheta=tf::createQuaternionFromYaw(delta_theta);//debugRobotYaw-M_PI_2);
            quaternionTFToMsg(quatTheta, deltaRobotOdom.pose.pose.orientation);
            ROS_INFO("omega=%f",omega);
            deltaRobotOdom.twist.twist.angular.x = 0;
            deltaRobotOdom.twist.twist.angular.y = 0;
            deltaRobotOdom.twist.twist.angular.z = omega;
            
            pre_robotOdom = robotOdom;

        }

        //メインループ
        void main_process(){
            // ROS_INFO("main_process");
            //仮
            // tf::Quaternion quat;
            // quat=tf::createQuaternionFromYaw(M_PI_2);
            // geometry_msgs::Quaternion geoQua;
            // quaternionTFToMsg(quat, geoQua);
            // ROS_INFO_STREAM("quaternion ="<<std::endl
            //     <<geoQua.x<<std::endl
            //     <<geoQua.y<<std::endl
            //     <<geoQua.z<<std::endl
            //     <<geoQua.w<<std::endl
            // );
            if(data_check()){
                ROS_INFO("data_check");
                get_time();
                data_check_reset();
                ROS_INFO("data_check_reset");
                if(!culc_delta_time()){
        			pre_robotOdom = robotOdom;
                    return;
                }
                //ロボットグローバル速度算出
                culc_delta_robotOdom();
                trans_obstacle_velocity();
                ROS_INFO("trans_obstacle_velocity");
                update_goal_position();
                ROS_INFO("update_goal_position");
                process();
                ROS_INFO("process");
                publish_cmd_vel();
            }
        }
        bool data_check(){
            if(RECEIVED_CLUSTER&& RECEIVED_GOAL_ODOM&& RECEIVED_ROBOT_ODOM 
                && RECEIVED_ROBOT_ENCODAR){
                    ROS_INFO_STREAM("received data");
                }
            // ROS_INFO_STREAM(
            //     (RECEIVED_CLUSTER ? "RECEIVED_CLUSTER" : "NOT CLUSTER") <<","<<
            //     (RECEIVED_GOAL_ODOM ? "RECEIVED_GOAL_ODOM" : "NOT GOAL_ODOM") <<","<<
            //     (RECEIVED_ROBOT_ODOM ? "RECEIVED_ROBOT_ODOM" : "NOT ROBOT_ODOM") <<","<<
            //     (RECEIVED_ROBOT_ENCODAR ? "RECEIVED_ROBOT_ENCODAR" : "NOT ROBOT_ENCODAR") 
            // );
            return(
                RECEIVED_CLUSTER&& RECEIVED_GOAL_ODOM&& RECEIVED_ROBOT_ODOM 
                && RECEIVED_ROBOT_ENCODAR
            );
        }
        void get_time(){
            cur_time = ros::Time::now();
        }
        bool culc_delta_time(){
            if(!PROCESS_ONCE){
                delta_time_ros = cur_time - pre_time;
                delta_time = delta_time_ros.toSec();
                pre_time = cur_time;
                PROCESS_ONCE = false;
                return true;
            }
            //
            else{
                pre_time = cur_time;
                PROCESS_ONCE = false;
                return false;
            }
        }
        void trans_obstacle_velocity(){
            std::cout<<"v:"<<cur_vel<<", angvel"<<cur_angVel<<std::endl;
            double vy_r = cur_vel*cos(cur_angVel * delta_time);
            double vx_r = -cur_vel*sin(cur_angVel * delta_time);
            //
            double delta_r,delta_p,delta_yaw;
            tf::Quaternion quat_rot;
            quaternionMsgToTF(deltaRobotOdom.pose.pose.orientation, quat_rot);
            tf::Matrix3x3(quat_rot).getRPY(delta_r,delta_p,delta_yaw);
            //
            std::cout<<"vr:("<<vx_r<<","<<vy_r<<std::endl;
            for(int i = 0; i<clstr.twist.size();i++){
                double pos_x = clstr.data[i].gc.x;
                double pos_y = clstr.data[i].gc.y;
                double vel_x = clstr.twist[i].linear.x;
                double vel_y = clstr.twist[i].linear.y;
                double delta_pos_x = pos_x + cos(delta_yaw)*(vel_x*delta_time - pos_x + vx_r*delta_time) + sin(delta_yaw)*(vel_y*delta_time - pos_y + vy_r*delta_time);
        	    double delta_pos_y = pos_y - sin(delta_yaw)*(vel_x*delta_time - pos_x + vx_r*delta_time) + cos(delta_yaw)*(vel_y*delta_time - pos_y + vy_r*delta_time);
                // 速度
                float Vox = delta_pos_x/delta_time;
                float Voy = delta_pos_y/delta_time;
                //回転
                tf::Quaternion quat;
                double r_cur,p_cur,y_cur;
                quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
                tf::Matrix3x3(quat).getRPY(r_cur,p_cur,y_cur);
                double r_pre,p_pre,y_pre;
                quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
                tf::Matrix3x3(quat).getRPY(r_pre,p_pre,y_pre);
                // double delta_theta = y_cur - y_pre;
                double delta_theta = cur_angVel*delta_theta;
                double V = std::sqrt(std::pow(Vox,2.0)+std::pow(Voy,2.0));
                double theta = std::atan2(Voy,Vox);
                double theta_rot = theta - delta_theta;
                Vox = V*cos(theta_rot);
                Voy = V*sin(theta_rot);
                //
                clstr.twist[i].linear.x = Vox;
                clstr.twist[i].linear.y = Voy;
                // clstr.twist[i].linear.x = clstr.twist[i].linear.x + vx_r;
                // clstr.twist[i].linear.y = clstr.twist[i].linear.y + vy_r;
            }
        }
        void data_check_reset(){
            RECEIVED_CLUSTER = false;
            RECEIVED_ROBOT_ODOM = false;
            RECEIVED_ROBOT_ENCODAR = false;
        }
        void update_goal_position(){
            //自己位置姿勢とゴール位置からロボット座標軸上でのゴール座標を算出する
            //robotOdom, goalOdom -> relationOdom
            //ゴール位置のフレームIDをマップに設定してgoalOdomをbase_linkに座標変化すればいいのでは
            // tf::TransformListener listener_;
            // tf::Transform cam_to_target;
            // tf::poseMsgToTF(p->pose.pose, cam_to_target);
            // tf::StampedTransform req_to_cam;
            // listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
            //面倒なので位置の差と回転行列だけで良さそう

            //位置差
            // std::cout<<goalOdom.pose.pose.position.x<<","<<goalOdom.pose.pose.position.y<<","<<goalOdom.pose.pose.position.z<<std::endl;
            // std::cout<<robotOdom.pose.pose.position.x<<","<<robotOdom.pose.pose.position.y<<","<<robotOdom.pose.pose.position.z<<std::endl;

            relationOdom.pose.pose.position.x = goalOdom.pose.pose.position.y - robotOdom.pose.pose.position.x;
            relationOdom.pose.pose.position.y = (-goalOdom.pose.pose.position.x) - robotOdom.pose.pose.position.y;
            relationOdom.pose.pose.position.z = goalOdom.pose.pose.position.z - robotOdom.pose.pose.position.z;
            // tf::Quaternion quatGoal=tf::createQuaternionFromYaw(M_PI_2);//debugRobotYaw-M_PI_2);
            // quaternionTFToMsg(quatGoal, goalOdom.pose.pose.orientation);
            //角度差はロボット姿勢角度 
            tf::Quaternion quat;
            double r,p,y;
            quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(r, p, y);
            //ロボット座標系の軸を揃える
            // y += M_PI_2;//90deg回転

            double theta_goal = atan2(relationOdom.pose.pose.position.y,relationOdom.pose.pose.position.x);
            double theta_robot = y;
            // std::cout<<"theta_goal - theta_robot = "<<theta_goal - theta_robot<<std::endl;
            double theta_relation = theta_goal - theta_robot;
            double length = std::sqrt(std::pow(relationOdom.pose.pose.position.x,2.0) + std::pow(relationOdom.pose.pose.position.y,2.0));
            //グロ-バル座標軸で算出
            // std::cout<< length <<"*"<< "cos("<<theta_relation<<");"<<std::endl;
            // std::cout<< length <<"*"<< cos(theta_relation)<<";"<<std::endl;
            relationOdom.pose.pose.position.x = length * cos(theta_relation);
            relationOdom.pose.pose.position.y = length * sin(theta_relation);
            relationOdom.pose.pose.position.z = 0;
            // std::cout<< length <<"*"<< "sin("<<theta_relation<<");"<<std::endl;
            // std::cout<< length <<"*"<< sin(theta_relation)<<";"<<std::endl;
            // std::cout<<"relationOdom.pose.pose.position.x="<< length * cos(theta_relation)<<std::endl;
            // std::cout<<"relationOdom.pose.pose.position.y="<< length * sin(theta_relation)<<std::endl;
            // std::cout<<"relationOdom.pose.pose.position.x="<< relationOdom.pose.pose.position.x<<std::endl;
            // std::cout<<"relationOdom.pose.pose.position.y="<< relationOdom.pose.pose.position.y<<std::endl;
            quat=tf::createQuaternionFromYaw(theta_relation);
            quaternionTFToMsg(quat, relationOdom.pose.pose.orientation);
        }
        //処理
        void process(){
            //clear nodes
            vfhTDT.clear_node();
            //スタートノードとゴールノードをセット
            vfhTDT.create_start_node();//
            // std::cout<<"hoge"<<std::endl;
            // std::cout<<"relationOdom.pose.pose.position.x="<< relationOdom.pose.pose.position.x<<std::endl;
            // std::cout<<"relationOdom.pose.pose.position.y="<< relationOdom.pose.pose.position.y<<std::endl;
            std::cout<<"create_goal_node("<< -relationOdom.pose.pose.position.y<<","<< relationOdom.pose.pose.position.x<<");"<<std::endl;
            double goal_x = -relationOdom.pose.pose.position.y;
            double goal_y = relationOdom.pose.pose.position.x;
            vfhTDT.create_goal_node(goal_x ,goal_y);//
            //スタートノードをオープンリストに追加
            // std::cout<<"create_goal_node"<<std::endl;
            vfhTDT.add_start_node();
            //A*アルゴリズムで探索を行っていく
            int best_node_num;
            int target_num;
            // std::cout<<"add_start_node"<<std::endl;
            while(ros::ok()){
                //最小コストとなるノード番号を取得
                // ROS_INFO("get_min_cost_node");
                int node_num = vfhTDT.get_min_cost_node();
                // ROS_INFO("check_search_finish");
                if(vfhTDT.check_search_finish()){
                    best_node_num = vfhTDT.get_best_node();
                    // best_node_num = node_num;
                    break;
                }
                // ROS_INFO("add_node");
                //最小コストノードの子ノードを作成
                // vfhTDT.add_node(vfhTDT.get_node(0));//先頭ノードを取得
                vfhTDT.debug_add_node(vfhTDT.get_node(0), clstr);//先頭ノードを取得
                // 
                // ROS_INFO("Node:(open,closed):(%d,%d)",vfhTDT.get_open_node_size(), vfhTDT.get_closed_node_size());
            }
            //
            std::cout<<"cobine_open_close_node"<<std::endl;
            vfhTDT.cobine_open_close_node();
            std::cout<<"search_node_n"<<std::endl;
            if(best_node_num ==0){
                target_num = best_node_num;
                target_angle = M_PI_2;
                return;
            }
            else{
                target_num = vfhTDT.search_node_n(best_node_num);
            }
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
            // display_all_node(vfhTDT.get_open_node(), vfhTDT.get_closed_node());
            display_all_node(clstr,vfhTDT.get_open_node(), vfhTDT.get_closed_node());
            // display_clstr(clstr);
        }
        void publish_cmd_vel(){
            geometry_msgs::Twist cmd_vel = controler(target_vel,target_angle);
            pub.publish(cmd_vel);
        }
        geometry_msgs::Twist controler(float& tagVel, float& tagAng){
            //p制御
            double cur_ang = M_PI_2;//正面を向いているため
            float tagAngVel = (tagAng-cur_ang)*180/M_PI*gainP;
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
            debugRobotX1 = config.debugRobotX1;            
            debugRobotY1 = config.debugRobotY1;
            debugRobotYaw = config.debugRobotYaw*M_PI/180;
            debugCurAng = config.debugCurAng;
            debugCmd_vel = config.debugCmd_vel;
            debugPrevTagAng = config.debugPrevTagAng;
            debugRobotRadius = config.debugRobotRadius;
            //obstacle
            debugObstacleX1 = config.debugObstacleX1;
            debugObstacleY1 = config.debugObstacleY1;
            debugObstacleW1 = config.debugObstacleW1;
            debugObstacleH1 = config.debugObstacleH1;
            debugObstacleVx1 = config.debugObstacleVx1;
            debugObstacleVy1 = config.debugObstacleVy1;
            //obstacle
            debugObstacleX2 = config.debugObstacleX2;
            debugObstacleY2 = config.debugObstacleY2;
            debugObstacleW2 = config.debugObstacleW2;
            debugObstacleH2 = config.debugObstacleH2;
            debugObstacleVx2 = config.debugObstacleVx2;
            debugObstacleVy2 = config.debugObstacleVy2;
            //set odometry data
            debugGoalOdom.pose.pose.position.x = debugGoalPosX;//debugGoalPosY;
            debugGoalOdom.pose.pose.position.y = debugGoalPosY;//-debugGoalPosX;
            tf::Quaternion quat=tf::createQuaternionFromYaw(M_PI_2);//debugRobotYaw-M_PI_2);
            quaternionTFToMsg(quat, debugGoalOdom.pose.pose.orientation);
            
            debugGoalOdom.pose.pose.position.z = 0;
            debugRobotOdom.pose.pose.position.x = debugRobotX1;//debugRobotY1;
            debugRobotOdom.pose.pose.position.y = debugRobotY1;//-debugRobotX1;
            debugRobotOdom.pose.pose.position.z = 0;
            
            quat=tf::createQuaternionFromYaw(debugRobotYaw);//debugRobotYaw-M_PI_2);
            quaternionTFToMsg(quat, debugRobotOdom.pose.pose.orientation);
        }
        void update_debug_goal_position(){
            //自己位置姿勢とゴール位置からロボット座標軸上でのゴール座標を算出する
            //robotOdom, goalOdom -> relationOdom
            //ゴール位置のフレームIDをマップに設定してgoalOdomをbase_linkに座標変化すればいいのでは
            // tf::TransformListener listener_;
            // tf::Transform cam_to_target;
            // tf::poseMsgToTF(p->pose.pose, cam_to_target);
            // tf::StampedTransform req_to_cam;
            // listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
            //面倒なので位置の差と回転行列だけで良さそう
            //位置差
            debugRelationOdom.pose.pose.position.x = debugGoalOdom.pose.pose.position.x - debugRobotOdom.pose.pose.position.x;
            debugRelationOdom.pose.pose.position.y = debugGoalOdom.pose.pose.position.y - debugRobotOdom.pose.pose.position.y;
            debugRelationOdom.pose.pose.position.z = debugGoalOdom.pose.pose.position.z - debugRobotOdom.pose.pose.position.z;
            //角度差はロボット姿勢角度
            tf::Quaternion quat;
            double r,p,y;
            quaternionMsgToTF(debugRobotOdom.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(r, p, y);
            double theta_goal = atan2(debugRelationOdom.pose.pose.position.y,debugRelationOdom.pose.pose.position.x);
            double theta_robot = y;
            double theta_relation = theta_goal - theta_robot;
            double length = std::sqrt(std::pow(debugRelationOdom.pose.pose.position.x,2.0) + std::pow(debugRelationOdom.pose.pose.position.y,2.0));
            debugRelationOdom.pose.pose.position.x = length * cos(theta_relation);
            debugRelationOdom.pose.pose.position.y = length * sin(theta_relation);
            quat=tf::createQuaternionFromYaw(theta_relation);
            quaternionTFToMsg(quat, debugRelationOdom.pose.pose.orientation);
        }
        void run_debug_process(){
            ROS_INFO("run_debug_process");
            //clear nodes
            vfhTDT.clear_node();
            //デバッグクラスタの作成
            create_debug_clstr();
            // ROS_INFO("create_debug_clstr");
            //スタートノードとゴールノードをセット
            vfhTDT.create_start_node();//
            vfhTDT.create_goal_node(-debugRelationOdom.pose.pose.position.y,debugRelationOdom.pose.pose.position.x);//
            // ROS_INFO("create_start_node,create_goal_node");
            //スタートノードをオープンリストに追加
            vfhTDT.add_start_node();
            // ROS_INFO("add_start_node");
            //A*アルゴリズムで探索を行っていく
            int best_node_num;
            int target_num;
            ROS_INFO("while");
            
            ros::Rate rate(display_fps);
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
                //
                if(display_searching_process){
                    display_all_node(vfhTDT.get_open_node(), vfhTDT.get_closed_node(), 1.0/display_fps);
                    rate.sleep();
                }
                
            }
            //
            vfhTDT.cobine_open_close_node();
            ROS_INFO("cobine_open_close_node");
            target_num = vfhTDT.search_node_n(best_node_num, debugNextNodeIndex);
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
            debug_clstr.data.resize(2);
            debug_clstr.data[0].gc.x = debugObstacleX1;
            debug_clstr.data[0].gc.y = debugObstacleY1;
            debug_clstr.data[0].gc.z = 0;
            ROS_INFO("twist");
            debug_clstr.twist.resize(2);
            debug_clstr.twist[0].linear.x = debugObstacleVx1;
            debug_clstr.twist[0].linear.y = debugObstacleVy1;
            debug_clstr.twist[0].linear.z = 0;
            //
            double W0_0 = debugObstacleX1 - debugObstacleW1/2;
            double H0_0 = debugObstacleY1 - debugObstacleH1/2;
            double W1_0 = debugObstacleX1 + debugObstacleW1/2;
            double H1_0 = debugObstacleY1 + debugObstacleH1/2;
            debug_clstr.data[0].size.data = (int)(11*11);
            debug_clstr.data[0].pt.resize(debug_clstr.data[0].size.data);
            int count0=0;
            for(double h =H0_0; h <= H1_0; h+=debugObstacleH1/10.0){
                for(double w =W0_0; w<= W1_0; w+=debugObstacleW1/10.0){
                    debug_clstr.data[0].pt[count0].x = w; 
                    debug_clstr.data[0].pt[count0].y = h; 
                    debug_clstr.data[0].pt[count0].z = 0;

                    count0++;
                }
            }
            debug_clstr.data[0].pt.resize(count0);

            ROS_INFO("gc");
            debug_clstr.data[1].gc.x = debugObstacleX2;
            debug_clstr.data[1].gc.y = debugObstacleY2;
            debug_clstr.data[1].gc.z = 0;
            ROS_INFO("twist");
            debug_clstr.twist[1].linear.x = debugObstacleVx2;
            debug_clstr.twist[1].linear.y = debugObstacleVy2;
            debug_clstr.twist[1].linear.z = 0;
            //
            double W0 = debugObstacleX2 - debugObstacleW2/2;
            double H0 = debugObstacleY2 - debugObstacleH2/2;
            double W1 = debugObstacleX2 + debugObstacleW2/2;
            double H1 = debugObstacleY2 + debugObstacleH2/2;
            debug_clstr.data[1].size.data = (int)(11*11);
            debug_clstr.data[1].pt.resize(debug_clstr.data[1].size.data);
            int count=0;
            for(double h =H0; h <= H1; h+=debugObstacleH2/10.0){
                for(double w =W0; w<= W1; w+=debugObstacleW2/10.0){
                    debug_clstr.data[1].pt[count].x = w; 
                    debug_clstr.data[1].pt[count].y = h; 
                    debug_clstr.data[1].pt[count].z = 0;
                    // std::cout<<"pt:"<<std::endl
                    //     <<debug_clstr.data[1].pt[count].x<<std::endl
                    //     <<debug_clstr.data[1].pt[count].y<<std::endl
                    //     ;

                    count++;
                }
            }
            debug_clstr.data[1].pt.resize(count);
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
        void display_clstr(const local_navigation::ClassificationVelocityData& temp_clstr){
            ROS_INFO("display_debug_clstr");
            //マーカー表示
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header.frame_id= "base_link";//;temp_clstr.header;
            marker.ns = "my_namespace";
            marker.lifetime = ros::Duration(marker_life_time);
            // marker.type = visualization_msgs::Marker::ARROW;
            // marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            int marker_size=0;
            for(int i = 0; i<temp_clstr.data.size();i++){
                marker_size += (int)temp_clstr.data[i].pt.size();
            }
            marker_size += (int)temp_clstr.data.size();
            markerArray.markers.resize(marker_size);
            ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
            //
            //
            int count = 0;
            for(int i = 0; i<temp_clstr.data.size();i++){
                std::string clusterNumStr = "cluster: ";
                clusterNumStr = clusterNumStr + std::to_string(i);
                marker.ns = clusterNumStr;
                //text
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.text = "vx,vy:("+ std::to_string(temp_clstr.twist[i].linear.x) +","+ std::to_string(temp_clstr.twist[i].linear.y)+")" ;
                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.3;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.7;
                marker.pose.position.x = temp_clstr.data[i].gc.y;
                marker.pose.position.y = -temp_clstr.data[i].gc.x;
                marker.pose.position.z = temp_clstr.data[i].gc.z + 1.0;
                marker.id = count;
                markerArray.markers[count++] = marker;
                //position 
                double yaw = std::atan2(-temp_clstr.twist[i].linear.x, temp_clstr.twist[i].linear.y);
                if(temp_clstr.twist[i].linear.x==0 && temp_clstr.twist[i].linear.y ==0){
                   marker.type = visualization_msgs::Marker::SPHERE; 
                }
                else{
                    marker.type = visualization_msgs::Marker::ARROW;
                    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                }
                for(int k = 0; k<temp_clstr.data[i].pt.size();k++){
                    marker.pose.position.x = temp_clstr.data[i].pt[k].y;
                    marker.pose.position.y = -temp_clstr.data[i].pt[k].x;
                    marker.pose.position.z = temp_clstr.data[i].pt[k].z;
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
            marker_size = (int)open_node.size()+(int)closed_node.size() + 3 + (int)debugNextNodeIndex.size();
            markerArray.markers.resize(marker_size);
            ROS_INFO("Node:markerArray.markers.size():%d",(int)markerArray.markers.size());
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
                else if(i%(sparse_rate)!= 0 ){
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
            //ゴール
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.ns = "goal";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.pose.position.x = debugRelationOdom.pose.pose.position.x;
            marker.pose.position.y = debugRelationOdom.pose.pose.position.y;
            marker.pose.position.z = debugRelationOdom.pose.pose.position.z;
            marker.id = count;
            markerArray.markers[count++] = marker;
            
            //debugNextNodeIndex
            marker.type = visualization_msgs::Marker::ARROW;
            for(int k=0; k<debugNextNodeIndex.size();k++){
                //条件に合うノードまで進める
                int nodeNum = debugNextNodeIndex[k];
                //
                //
                cost_node node_temp = vfhTDT.get_conbine_node(nodeNum);
                //条件を超える角度の時
                if(node_temp.angle > M_PI_2 + M_PI_4 || node_temp.angle < M_PI_2 - M_PI_4){
                    //探索終了
                    break;
                }
                else{
                    marker.pose.position.x = node_temp.dy;
                    marker.pose.position.y = -node_temp.dx;
                    marker.pose.position.z = 0;
                    double y = node_temp.angle;
                    marker.pose.orientation = tf::createQuaternionMsgFromYaw(y-M_PI_2);
                    marker.scale.x = 0.2;
                    marker.scale.y = 0.05;
                    marker.scale.z = 0.05;
                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.id = count;
                    markerArray.markers[count++] = marker;

                }
            }
            //
            markerArray.markers.resize(count);
            ROS_INFO("Comp:markerArray.markers.size():%d",(int)markerArray.markers.size());
            if(markerArray.markers.size()){
                pubDebugNode.publish( markerArray );
            }   
        }

        void display_all_node(const std::vector<cost_node>& open_node,const std::vector<cost_node>& closed_node, float life_time){
            //マーカー表示
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header= debug_clstr.header;
            marker.ns = "my_namespace";
            marker.lifetime = ros::Duration(life_time);
            // marker.type = visualization_msgs::Marker::ARROW;
            // marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            int marker_size=0;
            marker_size = (int)open_node.size()+(int)closed_node.size() + 3;
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
                else if(i%(sparse_rate)!= 0 ){
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
            marker.ns =std::string("target_vec");
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
            marker.pose.position.z = 0;
            marker.id = count;
            markerArray.markers[count++] = marker;
            //
            markerArray.markers.resize(count);
            ROS_INFO("Comp:markerArray.markers.size():%d",(int)markerArray.markers.size());
            if(markerArray.markers.size()){
                pubDebugNode.publish( markerArray );
            }   
        }
        void display_all_node(const local_navigation::ClassificationVelocityData& temp_clstr, const std::vector<cost_node>& open_node,const std::vector<cost_node>& closed_node){
            //マーカー表示
            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header.frame_id= "base_link";//temp_clstr.header;
            std::cout<<"frame_id = "<<temp_clstr.header.frame_id<<std::endl;
            marker.ns = "my_namespace";
            marker.lifetime = ros::Duration(marker_life_time);
            // marker.type = visualization_msgs::Marker::ARROW;
            // marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            int marker_size=0;
            marker_size = (int)open_node.size()+(int)closed_node.size() + 3;
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
                else if(i%(sparse_rate)!= 0 ){
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
            marker.ns =std::string("target_vec");
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
            marker.pose.position.z = 0;
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

bool PROCESS_START;
//switch関数
void callback_function(const std_msgs::Empty::ConstPtr& msg)
{
	ROS_INFO("Turn On");
	PROCESS_START =true;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"vfh_tdt_node");
    
    ROS_INFO("constructer");
    run runCls;
    //idle process
	std_msgs::Empty empty_msg;
	ros::NodeHandle nh_s;
	ros::Publisher pub_s;
	pub_s = nh_s.advertise<std_msgs::Empty>("/robot1/mobile_base/commands/reset_odometry", 1);

	ros::Subscriber sub_s;
	ros::CallbackQueue queue;

	nh_s.setCallbackQueue(&queue);
	sub_s=nh_s.subscribe("start_swich",1,callback_function);

	PROCESS_START=false;
	while(ros::ok()&&!PROCESS_START)
	{
		ROS_INFO("Waiting switch msg");
		pub_s.publish(empty_msg);
		queue.callOne(ros::WallDuration(0.1));
	}
	ROS_INFO("start");
    //
    ros::spin();
	//--process
    
    //--
	return 0;
}
