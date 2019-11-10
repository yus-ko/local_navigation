//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <local_navigation/ClassificationVelocityData.h>
#include <beego_control/beego_encoder.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <local_navigation/simulaterConfig.h>
#include <tf/transform_broadcaster.h>

class simulater{
    protected:
        //受信データ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
        geometry_msgs::Twist cmd_vel;
        //--rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<local_navigation::simulaterConfig> server;
        dynamic_reconfigure::Server<local_navigation::simulaterConfig>::CallbackType f;
        //送信データ
        ros::NodeHandle nhPub;
        ros::Publisher pub1,pub2,pub3,pub4;
    	local_navigation::ClassificationVelocityData cur_clstr,pre_clstr;//速度データ付きのクラスタデータ
    	local_navigation::ClassificationVelocityData detected_clstr;//視認可能 
        nav_msgs::Odometry pre_robotOdom,cur_robotOdom,goalOdom;
        beego_control::beego_encoder robotEncoder;
        //処理用
        bool RECEIVED_CMD_VEl;
        bool SET_CONFIG;
        //ロボットデータ
        double sensor_angle_min,sensor_angle_max;
        float wheel_d;//車輪間隔の半分
        double max_speed;
        //
        double t;//シミュレータ内の時間
        //障害物データ
        double obstacleX1,obstacleY1;
        double obstacleW1,obstacleH1;
        double obstacleVx1,obstacleVy1;
        //ロボット初期位置
        double robot_init_x;
        double robot_init_y;
        double robot_init_angle;
        double robot_init_v;
        double robot_init_w;
        //ゴールデータ
        double goal_x,goal_y;
    public:
        simulater()
            :RECEIVED_CMD_VEl(false),SET_CONFIG(false)
        {
            //subscriber
            sub = nhSub.subscribe("/beego/cmd_vel",1,&simulater::cmd_vel_callback,this);
            pub1 = nhPub.advertise<local_navigation::ClassificationVelocityData>("classificationDataEstimateVelocity", 1);
            pub2 = nhPub.advertise<nav_msgs::Odometry>("zed_node/odom", 1);
            pub3 = nhPub.advertise<nav_msgs::Odometry>("goalOdometry", 1);
            pub4 = nhPub.advertise<beego_control::beego_encoder>("encoder", 1);
            //
            // read_launch_file();
            //rqt_reconfigure
            f = boost::bind(&simulater::configCallback, this, _1, _2);
            server.setCallback(f);
            ROS_INFO("ready");

        }
        ~simulater(){

        };
        //入出力
        //--入力
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
            cmd_vel = *msg;
            RECEIVED_CMD_VEl =true;
        }
        //---rqt_reconfigureからの読み込み
        void configCallback(local_navigation::simulaterConfig &config, uint32_t level){
            //セットフラグ
            SET_CONFIG =true;
            //障害物
            obstacleX1 = config.obstacleX1;
            obstacleY1 = config.obstacleY1;
            obstacleW1 = config.obstacleW1;
            obstacleH1 = config.obstacleH1;
            obstacleVx1 = config.obstacleVx1;
            obstacleVy1 = config.obstacleVy1;
            //ロボット
            //--パラメータ
            wheel_d = config.wheel_2d/2.0;
            sensor_angle_min = config.sensor_angle_min*M_PI/180;
            sensor_angle_max = config.sensor_angle_max*M_PI/180;
            max_speed = config.max_speed;
            //--位置
            robot_init_x = config.robot_init_x;
            robot_init_y = config.robot_init_y;
            robot_init_angle = config.robot_init_angle;
            robot_init_v = config.robot_init_v;
            robot_init_w = config.robot_init_w;
            //ゴール
            goal_x = config.goal_x;
            goal_y = config.goal_y;
        }
        //--出力
        void publishObstacleData(){//クラスタの出力
            pub1.publish(cur_clstr);
        }
        void publishRobotData(){//ロボットオドメトリを出力
            pub2.publish(cur_robotOdom);
        }
        void publishTargetData(){//目標位置(ゴール位置)を出力
            pub3.publish(goalOdom);
        }
        //データ作成
        void createObstacleData(){
            //障害物の個数            
            //障害物の位置角度速度
            //障害物の大きさ（半径）
            //障害物のマップセル位置とセルに含まれるデータ数
            //rosヘッダ
            cur_clstr.header.frame_id= "base_link";
            cur_clstr.header.seq = 0;
            cur_clstr.header.stamp = ros::Time::now();
            //マップ実測サイズ
            cur_clstr.width.data = 8.0;
            cur_clstr.height.data = 8.0;
            cur_clstr.res.data = 0.05;
            cur_clstr.widthInt.data = cur_clstr.width.data/cur_clstr.res.data;
            cur_clstr.heightInt.data = cur_clstr.height.data/cur_clstr.res.data;
            // マップの中心位置
            cur_clstr.cp.x = cur_clstr.width.data /2.0;
            cur_clstr.cp.y = cur_clstr.height.data /2.0;
            cur_clstr.cp.z = 0;
            //クラスタデータ
            //リサイズ
            cur_clstr.size.data = 1;
            cur_clstr.data.resize(cur_clstr.size.data);
            cur_clstr.twist.resize(cur_clstr.size.data);
            //
            //ロボット,グローバル座標系で作成
            //クラスタ1
            ROS_INFO("set cur_clstr 1");
            cur_clstr.data[0].gc.x = obstacleX1;
            cur_clstr.data[0].gc.y = obstacleY1;
            cur_clstr.data[0].gc.z = 0;
            cur_clstr.twist[0].linear.x = obstacleVx1;
            cur_clstr.twist[0].linear.y = obstacleVy1;
            cur_clstr.twist[0].linear.z = 0;
            double W0 = obstacleX1 - obstacleW1/2;
            double H0 = obstacleY1 - obstacleH1/2;
            double W1 = obstacleX1 + obstacleW1/2;
            double H1 = obstacleY1 + obstacleH1/2;
            cur_clstr.data[0].size.data = (int)(11*11);
            cur_clstr.data[0].pt.resize(cur_clstr.data[0].size.data);
            int count0=0;
            for(double h =H0; h <= H1; h+=obstacleH1/10.0){
                for(double w =W0; w<= W1; w+=obstacleW1/10.0){
                    cur_clstr.data[0].pt[count0].x = w; 
                    cur_clstr.data[0].pt[count0].y = h; 
                    cur_clstr.data[0].pt[count0].z = 0;
                    count0++;
                }
            }
            cur_clstr.data[0].pt.resize(count0);
            //クラスタ2
            //
            //---
            //

        }
        void createRobotData(){
            ros::Time set_time = ros::Time::now();
            // 必要データ
            // ロボットの幅
            // d = 0.3124/2.0;
            // センサ視野角度
            // sensor_angle_min = M_PI_4;
            // sensor_angle_max = M_PI_4*3;
            // ロボットの最高速度
            // float max_speed = 0.6;
            // ロボットの位置,角度
            // float robot_x = 0.0;
            // float robot_y = 0.0;
            // float robot_angle = 0.0;
            // ロボットの初期速度
            // float robot_v = 0.0;//移動速度
            // float robot_w = 0.0;//回転角速度
            //オドメトリ
            //tf座標系で作成
            cur_robotOdom.header.frame_id= "base_link";
            cur_robotOdom.header.seq = 0;
            cur_robotOdom.header.stamp = set_time;
            cur_robotOdom.pose.pose.position.x = robot_init_y;
            cur_robotOdom.pose.pose.position.y = -robot_init_x;
            cur_robotOdom.pose.pose.position.z = 0;
            cur_robotOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);//正面方向
            //エンコーダー
            robotEncoder.header.frame_id= "base_link";
            robotEncoder.header.seq = 0;
            robotEncoder.header.stamp = set_time;
            robotEncoder.vel.r = robot_init_v + wheel_d * robot_init_w;
            robotEncoder.vel.l = robot_init_v - wheel_d * robot_init_w;


        }
        void createGoalOdom(double goal_x_temp,double goal_y_temp){
            //goalOdom
            ros::Time set_time = ros::Time::now();
             //tf座標系で作成
            goalOdom.header.frame_id= "base_link";
            goalOdom.header.seq = 0;
            goalOdom.header.stamp = set_time;
            goalOdom.pose.pose.position.x =  goal_y_temp;
            goalOdom.pose.pose.position.y = -goal_x_temp;
            goalOdom.pose.pose.position.z = 0;
            goalOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);//正面方向
            
        }
        //データ更新
        void updateRobotEncoder(){//ロボットオドメトリ, 命令速度依存
            //ノイズ無し
            robotEncoder.vel.r = cmd_vel.linear.x + wheel_d * cmd_vel.angular.z;
            robotEncoder.vel.l = cmd_vel.linear.x - wheel_d * cmd_vel.angular.z;
        }
        void updateObstacleState(){//一つ前のロボット姿勢, 現時点のエンコーダ使用
            //障害物の個数
            //障害物の位置角度速度
            //障害物の大きさ（半径）
            //ヘッダ
            cur_clstr.header.frame_id = pre_clstr.header.frame_id;
            cur_clstr.header.stamp = ros::Time::now();
            cur_clstr.header.seq = pre_clstr.header.seq + 1;
            //経過時間算出
            ros::Duration delta_time_ros = cur_clstr.header.stamp - pre_clstr.header.stamp;
            double delta_time = delta_time_ros.toSec();
            //データ更新
            //--ロボットの移動方向と姿勢を取得
            double v = (robotEncoder.vel.r+robotEncoder.vel.l)/2.0;
            double w = (robotEncoder.vel.r-robotEncoder.vel.l)/(2.0*wheel_d);
            tf::Quaternion quat;
            double r,p,y;
            quaternionMsgToTF(pre_robotOdom.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(r, p, y);
            y = y + M_PI_2;//tf座標系からグローバル座標系へ
            //--ロボット速度を取得(グローバル座標系)
            double vy_r = v*sin(w * delta_time + y);
            double vx_r = v*cos(w * delta_time + y);
            std::cout<<"vr:("<<vx_r<<","<<vy_r<<std::endl;
            //--速度
            for(int i = 0; i<cur_clstr.twist.size();i++){
                cur_clstr.twist[i].linear.x = pre_clstr.twist[i].linear.x + vx_r;
                cur_clstr.twist[i].linear.y = pre_clstr.twist[i].linear.y + vy_r;
            }
            //--位置
            //グローバル座標系で
            for(int i = 0; i<cur_clstr.data.size();i++){
                cur_clstr.data[i].gc.x = pre_clstr.data[i].gc.x + vx_r;
                cur_clstr.data[i].gc.y = pre_clstr.data[i].gc.y + vy_r;
                for(int k = 0; k<cur_clstr.data[i].pt.size();k++){
                    cur_clstr.data[i].pt[k].x = pre_clstr.data[i].pt[k].x + vx_r*delta_time;
                    cur_clstr.data[i].pt[k].y = pre_clstr.data[i].pt[k].y + vy_r*delta_time;
                }
            }
        }
        void updateRobotState(){//ロボット速度使用
            // ロボットの位置,角度
            // ロボットの速度
            cur_robotOdom.header.frame_id = pre_robotOdom.header.frame_id;
            cur_robotOdom.header.stamp = ros::Time::now();
            cur_robotOdom.header.seq = pre_robotOdom.header.seq + 1;
            //経過時間算出
            ros::Duration delta_time_ros = cur_robotOdom.header.stamp - pre_robotOdom.header.stamp;
            double delta_time = delta_time_ros.toSec();
            //速度算出
            double v = (robotEncoder.vel.r+robotEncoder.vel.l)/2.0;
            double w = (robotEncoder.vel.r-robotEncoder.vel.l)/(2.0*wheel_d);
            //--姿勢取得(tf座標系)
            tf::Quaternion quat;
            double r,p,y;
            quaternionMsgToTF(pre_robotOdom.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(r, p, y);
            //--ロボット速度算出(tf座標系)
            double vx_r = v*cos(w * delta_time + y);
            double vy_r = v*sin(w * delta_time + y);
            //位置更新(tf座標系)
            cur_robotOdom.pose.pose.position.x = cur_robotOdom.pose.pose.position.x + vx_r *delta_time;
            cur_robotOdom.pose.pose.position.y = cur_robotOdom.pose.pose.position.y + vy_r *delta_time;
            cur_robotOdom.pose.pose.position.z = cur_robotOdom.pose.pose.position.z;
            double yaw = w * delta_time + y;
            cur_robotOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            
        }
        //視野角反映
        void detectObstacle(){//ロボットから視認できる障害物を抽出
            //ロボットと障害物の角度, ロボット姿勢から視野角内の点群を抽出
            //
            //1.ロボットと障害物の角度, ロボット姿勢をグローバル座標系に変換する
            //2.障害物の各点との角度を算出して視野角内かを確認
            //3.視野角内の点を追加
            //重心位置と障害物点群の取り扱い: 重心が見えている障害物のみを移動障害物と視認可能
            //
            // クラスタのコピー
            detected_clstr = cur_clstr;
            //
            //ロボットの姿勢を取得
            //tf座標系
            double robot_roll,robot_pitch,robot_yaw;
            tf::Quaternion quat;
            quaternionMsgToTF(cur_robotOdom.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(robot_roll,robot_pitch,robot_yaw);
            //--グローバル座標系に変換
            robot_yaw = robot_yaw + M_PI_2;//90度ずらす
            //ロボットの位置を取得
            double robot_x, robot_y, robot_z;
            //--グローバル座標系に変換
            robot_x = -cur_robotOdom.pose.pose.position.y;
            robot_y = cur_robotOdom.pose.pose.position.x;
            //
            //障害物位置を取得
            for(int i = 0; i<cur_clstr.data.size();i++){
                //重心位置が視認可能かを確認
                double delta_gcx = cur_clstr.data[i].gc.x - robot_x;
                double delta_gcy = cur_clstr.data[i].gc.y - robot_y;
                double angle_gc = atan2(delta_gcy,delta_gcx);
                if(angle_gc < sensor_angle_min || angle_gc > sensor_angle_max){
                    //センサ視野外->静止障害物
                    detected_clstr.twist[i].linear.x = 0;
                    detected_clstr.twist[i].linear.y = 0;
                }
                //点群データを空にしてリサイズ
                detected_clstr.data[i].pt.clear();
                detected_clstr.data[i].pt.resize(cur_clstr.data[i].pt.size());
                int itr =0;
                //点群の各点が視認可能かを確認
                for(int k = 0; k<cur_clstr.data[i].pt.size();k++){
                    double delta_ptx = cur_clstr.data[i].pt[k].x - robot_x;
                    double delta_pty = cur_clstr.data[i].pt[k].y - robot_y;
                    double angle_pt = atan2(delta_pty,delta_ptx);
                    if(angle_pt < sensor_angle_min || angle_pt > sensor_angle_max){
                        //センサ視野外->静止障害物
                        //スキップ
                    }
                    else{
                        detected_clstr.data[i].pt[itr].x = cur_clstr.data[i].pt[k].x;
                        detected_clstr.data[i].pt[itr].y = cur_clstr.data[i].pt[k].y;
                        detected_clstr.data[i].pt[itr].z = cur_clstr.data[i].pt[k].z;
                        itr++;
                    }
                }
                //再度リサイズ
                detected_clstr.data[i].pt.resize(itr);
            }
        }

};