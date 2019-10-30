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

class simulater{
    private:
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
        ros::Publisher pub;
    	local_navigation::ClassificationVelocityData clstr;//速度データ付きのクラスタデータ
        nav_msgs::Odometry robotOdom,goalOdom;
        beego_control::beego_encoder robotEncoder;
        //処理用
        //ロボットデータ
        float d;//車輪間隔の半分
        //
        double t;//シミュレータ内の時間
        //
    public:
        simulater();
        ~simulater();
        //入出力
        //--入力
        void callback(const geometry_msgs::Twist::ConstPtr& msg);
        //---rqt_reconfigureからの読み込み
        // void configCallback(config_msg &config, uint32_t level);
        //--出力
        void publishObstacleData();//クラスタの出力
        void publishRobotData();//ロボットオドメトリを出力
        void publishTargetData();//目標位置(ゴール位置)を出力
        // void cluster_callback(const local_navigation::ClassificationVelocityData::ConstPtr& msg);
        // void robotOdom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        // void robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg);
        // void goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        //データ作成
        void createObstacleData(){
            //障害物の個数            
            //障害物の位置角度速度
            //障害物の大きさ（半径）
            //障害物のマップセル位置とセルに含まれるデータ数
            //rosヘッダ
            clstr.header.frame_id= "base_link";
            clstr.header.seq = 0;
            clstr.header.stamp = ros::Time::now();
            //マップ実測サイズ
            clstr.width.data = 8.0;
            clstr.height.data = 8.0;
            clstr.res.data = 0.05;
            clstr.widthInt.data = clstr.width.data/clstr.res.data;
            clstr.heightInt.data = clstr.height.data/clstr.res.data;
            // マップの中心位置
            clstr.cp.x = clstr.width.data /2.0;
            clstr.cp.y = clstr.height.data /2.0;
            clstr.cp.z = 0;
            //クラスタデータ
            clstr.size.data = 1;
            clstr.data.resize(clstr.size.data);
            clstr.twist.resize(clstr.size.data);
        }
        void createRobotData(){
            // 必要データ
            // ロボットの幅
            float d = 0.3124/2.0;
            // センサ視野角度
            float sensor_angle_min = M_PI_4;
            float sensor_angle_max = M_PI_4*3;
            // ロボットの最高速度
            float max_speed = 0.6;
            // ロボットの位置,角度
            float robot_x = 0.0;
            float robot_y = 0.0;
            float robot_angle = 0.0;
            // ロボットの初期速度
            float robot_v = 0.0;
        }
        //データ更新
        void updateObstacleState(){
            //障害物の個数            
            //障害物の位置角度速度
            //障害物の大きさ（半径）
            //
        }
        void updateRobotState(){
            // ロボットの位置,角度
            // ロボットの速度

        }
        
};