//多重インクルード防止
#ifndef INCLUDE_OBSTACLE_AVOIDANCE_CLASS
#define INCLUDE_OBSTACLE_AVOIDANCE_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
// msg
#include <local_navigation/ClassificationVelocityData.h>
#include <beego_control/beego_encoder.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
//追加（デバッグ用）
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include<visualization_msgs/MarkerArray.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
#include <local_navigation/obstacleAvoidanceConfig.h>
#include <local_navigation/struct.h>
//クラスの定義
class obstacleAvoidance{
    private:
        //受信データ
		ros::NodeHandle nhSub1;
		ros::Subscriber sub1,sub2,sub3;
    	local_navigation::ClassificationVelocityData clstr;//速度データ付きのクラスタデータ
        nav_msgs::Odometry robotOdom,goalOdom;
        beego_control::beego_encoder robotEncoder;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        // 処理
        //ロボットデータ
        float d;
        //回避コストパラメータ
        float k_cp, eta_cp;//交差位置に対する重み
        float k_o, eta_o;//静止障害物に対する重み
        float k_g, eta_g;//ゴール位置への角度と目標角度に対する重み
        float k_theta, eta_theta;//現在の角度と目標角度に対する重み
        float k_omega, eta_omega;//現在の角速度と目標角速度に対する重み
        
        // デバッグ用
		ros::NodeHandle nhDeb;
        ros::Publisher pubDebPcl,pubDebMarker;
        int debugType;
        float timeRange, timeInteval;//表示時間範囲(~秒後まで表示),表示時間間隔(~秒ごとに表示)
        float colors[12][3] ={{1.0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,0},{0,1.0,0},{0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
        //--rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<local_navigation::obstacleAvoidanceConfig> server;
        dynamic_reconfigure::Server<local_navigation::obstacleAvoidanceConfig>::CallbackType f;
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        obstacleAvoidance();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~obstacleAvoidance();
        //
        //メソッド:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルからの探索窓パラメータ書き込み
        //ゲット：内部パラメータの読み込み
        //
        //in methods.cpp
        //その他メソッド
        //--メソッド管理
        void manage();
        //--センサーデータ受信
        void cluster_callback(const local_navigation::ClassificationVelocityData::ConstPtr& msg);
        void robotOdom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg);
        void goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        //--rqt_reconfigureからの読み込み
        void configCallback(local_navigation::obstacleAvoidanceConfig &config, uint32_t level);
        //処理
        crossPoint getCrossPoint(int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cmd_vel, float& cmd_angle);
        void crossPointsDetect(std::vector<crossPoint>& crsPts, float& cmd_vel, float& cmd_angle);
        float culcCrossPointCost(crossPoint& crsPt);
        float getCrossPointCost(float& cmd_vel, float& cmd_angle);
        void labelObstacles();
        void evaluation(float& angle);
        void searchProcess();
        void setCmdVel();
        void setCmdAngle();
        // データ送信
        void publishData();//データ送信
        //デバッグ用のメソッド
        void debug();
        void showCrossPoints();
        void showCostMap();
};
#endif