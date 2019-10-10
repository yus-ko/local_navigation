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
#include <local_navigation/vfh.h>
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
        float d;//車輪間隔の半分
        //回避コストパラメータ
        float k_cp, eta_cp;//交差位置に対する重み
        float k_o, eta_o;//静止障害物に対する重み
        float k_g, eta_g;//ゴール位置への角度と目標角度に対する重み
        float k_theta, eta_theta;//現在の角度と目標角度に対する重み
        float k_omega, eta_omega;//現在の角速度と目標角速度に対する重み
        //
    	std::vector<crossPoint> crsPts;
        float goal_angle;
        float pre_angle;
        float angle_min,angle_max, angle_dev;
        float dis_th;//距離ヒストグラムの閾値
        std::vector<double> hst_dis;//ヒストグラム配列(距離)
        std::vector<bool> hst_bi;//ヒストグラム配列(２値化後)
        vfh vfh_c;//vfhクラス
        // デバッグ用
		ros::NodeHandle nhDeb;
        ros::Publisher pubDebPcl,pubDebMarker,pubDebMarkerArray;
        int debugType;
        //カラーリスト
        float colors[12][3] ={{1.0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,0},{0,1.0,0},{0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
        //クロスポイントチェッカー入力
        bool debugFlag_crossPointChecker;
        float debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
        float debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
        float debugCmd_vel;//ロボット目標速度
        float debugCmd_angle;//ロボット目標角度
        int debugIndexRef;//障害物番号
        geometry_msgs::Point debugGpRef;//クラスタ重心
        geometry_msgs::Twist debugTwistRef;//障害物速度
        float debugObstacleRadius;//障害物半径
        float debugRobotRadius;//ロボット半径

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
        void setDefaultCrossPointChecker();//デバッグ用のパラメータ初期化
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
        void crossPointsDetect(float& cmd_vel, float& cmd_angle);
        void crossPointsDetect(std::vector<crossPoint>& crsPts, float& cmd_vel, float& cmd_angle);
        float generalCostFunction(float& eta, float& value);
        float costCrossPoint(crossPoint& crsPt);
        float getCrossPointCost();//交差位置コスト
        bool checkSafetyObstacle(float& t, float& angle, float& x, float& y);
        double evaluation(float& vel, float& angle);
        void searchProcess();
        void setCmdVel();
        void setCmdAngle();
        //vfh+
        void create_histgram();
		void create_binary_histgram();
        void setHistgramParam();
        void setHistgramData();
        float costVFHGoalAngle(float goalAngle);//vfh+第1項
        float costVFHDeltaAngle(float delAngle);//vfh+第2項
        float costVFHDeltaOmega(float delOmega);//vfh+第3項
        // データ送信
        void publishData();//データ送信
        //デバッグ用のメソッド
        void debug();
        void showCrossPoints();
        void showCostMap();
        void crossPointChecker();
};
#endif