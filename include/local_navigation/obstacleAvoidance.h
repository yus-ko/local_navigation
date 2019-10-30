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
        //launch ファイル
        //--ロボットパラメータ
        float d;//車輪間隔の半分
        float robotRadius;//ロボット半径
        //--vfh
        float marginRadius;//マージン半径
        float dis_th;//距離ヒストグラムの閾値
        float k_cp, eta_cp;//交差位置に対する重み
        float k_g, eta_g;//ゴール位置への角度と目標角度に対する重み
        float k_curAngle, eta_curAngle;//現在の角度と目標角度に対する重み
        float k_prevAngle, eta_prevAngle;//現在の角速度と目標角速度に対する重み
        //
    	std::vector<crossPoint> crsPts;
        float goal_x, goal_y;
        float cur_angle;
        float prev_tagAng;
        float angle_min,angle_max, angle_div;
        float cur_vel,cur_angVel;
        float dV_range, dV_div;
        //
        std::vector<double> hst_dis;//ヒストグラム配列(距離)
        std::vector<bool> hst_bi;//ヒストグラム配列(２値化後)
        vfh vfh_c;//vfhクラス
        // デバッグ用
		ros::NodeHandle nhDeb;
        ros::Publisher pubDebPcl,pubDebCross,pubDebMarkerArray, pubDebHst,pubDebOutput,pubDebCPVFHOutput;
        int debugType;
        //カラーリスト
        float colors[12][3] ={{1.0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,0},{0,1.0,0},{0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
        //クロスポイントチェッカー入力
        bool debugFlag_crossPointChecker;
        float debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
        float debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
        float debugCur_vel;//ロボット速度
        float debugCur_angle_steer;//現在のロボットステアリング角度（進行方向）
        float debugCmd_vel;//ロボット目標速度
        float debugCmd_angle;//ロボット目標角度
        int debugIndexRef;//障害物番号
        geometry_msgs::Point debugGpRef;//クラスタ重心
        geometry_msgs::Twist debugTwistRef;//障害物速度
        float debugObstacleRadius;//障害物半径
        float debugRobotRadius;//ロボット半径（ヒストグラムチェッカーでも使用）
        //ヒストグラムチェッカー入力
        bool debugHistgramCheckerFlag;
        int debugObstacleNum;
        float debugObstacleX1;
        float debugObstacleY1;
        float debugObstacleSize1;
        float debugObstacleX2;
        float debugObstacleY2;
        float debugObstacleSize2;
        float debugObstacleX3;
        float debugObstacleY3;
        float debugObstacleSize3;
        float debugThresholdDistance;
        float debugMinAngle;
        float debugMaxAngle;
        float debugDivAngle;
        float debugMarginRadius;//マージン半径
        //VFH出力チェッカー
        bool debugOutputVFHCheckerFlag;
        float debugKg, debugEtaG;//ゴール位置への角度と目標角度に対する重み
        float debugKcurAngle, debugEtaCurAngle;//現在の角度と目標角度に対する重み
        float debugKprevAngle, debugEtaPrevAngle;//現在の角速度と目標角速度に対する重み
        float debugGoalAng;//目標角度 
        float debugGoalPosX;//目標位置X
        float debugGoalPosY;//目標位置Y 
        float debugCurAng;//現在の角度
        float debugPrevTagAng;//角速度を取得
        //CP-VFH出力チェッカー
        bool debugOutputCPVFHCheckerFlag;
        float debugKcp, debugEtaCp;//交差位置に対する重み
        float debugObstacleVx1;
        float debugObstacleVy1;
        float debugObstacleVx2;
        float debugObstacleVy2;
        float debugObstacleVx3;
        float debugObstacleVy3;
        float debugObstacleSizeThreshold;
        geometry_msgs::Point debugGp1,debugGp2,debugGp3;//クラスタ重心
        geometry_msgs::Twist debugTwist1,debugTwist2,debugTwist3;//障害物速度
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
        void update_goal_position();
        //処理
        crossPoint getCrossPoint(int& cp_num, std::vector<crossPoint>& crsPts, int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cur_vel, float& cmd_dV, float& cmd_dAng);
        void getCrossPoints(crossPoint& crsPt_x0, crossPoint& crsPt_y0, int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang);
        void crossPointsDetect(float& cmd_vel, float& cmd_angle);
        void crossPointsDetect(std::vector<crossPoint>& crsPts, float& cur_vel_temp, float& cur_angle_temp, float& cmd_dV, float& cmd_dAng);
        float generalCostFunction(float& eta, float& value);
        double costCrossPoint(crossPoint& crsPt, float eta_cp);
        double getCrossPointCost(std::vector<crossPoint>& crsPts, float eta_cp);//交差位置コスト
        bool checkSafetyObstacle(float& t, float& angle, float& x, float& y);
        void searchProcess(float& tagVel, float& tagAng);
        void search_vel_ang(float& target_vel, float& target_angle);        
        float vfh_angleSearch(float& target_angle_temp, float& cur_vel_temp, float& cmd_dV);//return cost
        void setCmdVel();
        void setCmdAngle();
        geometry_msgs::Twist controler(float& tagVel, float& tagAng);
        //vfh+
        void create_histgram();
		void create_binary_histgram(float& robotRadius, float& marginRadius);
        void setHistgramParam();
        void setHistgramData();
        // データ送信
        void publishData(geometry_msgs::Twist& pubData);//データ送信
        //デバッグ用のメソッド
        void debug();
        void showCrossPoints();
        void showCostMap();
        void crossPointChecker();
        void histgramChecker();
        void outputVFHChecker();
        void outputCrossPointVFHChecker();
};
#endif