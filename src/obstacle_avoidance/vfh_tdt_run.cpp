#include<vfh_tdt.h>
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

class run{
    private:
        //受信データ
        ros::NodeHandle nhSub1;
        ros::Subscriber sub1,sub2,sub3;
        local_navigation::ClassificationVelocityData clstr;//速度データ付きのクラスタデータ
        nav_msgs::Odometry robotOdom,goalOdom;
        beego_control::beego_encoder robotEncoder;
        float cur_vel, cur_angVel;
        //rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<local_navigation::obstacleAvoidanceConfig> server;
        dynamic_reconfigure::Server<local_navigation::obstacleAvoidanceConfig>::CallbackType f;
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
        float taeget_vel, taeget_angle;
        //制御
        float gainP;//Pゲイン
    public:
        run()
        RECEIVED_CLUSTER(false),RECEIVED_GOAL_ODOM(false),RECEIVED_ROBOT_ODOM(false)        
        {
            //subscriber
            sub1=nhSub1.subscribe("classificationDataEstimateVelocity",1,&run::cluster_callback,this);
            sub2=nhSub1.subscribe("robotOdometry",1,&run::robotOdom_callback,this);
            sub3=nhSub1.subscribe("goalOdometry",1,&run::goalOdom_callback,this);
            pub= nhPub.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            //
            //rqt_reconfigure
            f = boost::bind(&obstacleAvoidance::configCallback, this, _1, _2);
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
            //ロボットパラメータ
            n.getParam("vfh_tdt/robotRadius",robotRadius_tmp);
            n.getParam("vfh_tdt/robotSteerRadius",steer_r_tmp);
            n.getParam("vfh_tdt/controllerGainP",gainP);
            n.getParam("vfh_tdt/default_vel",default_vel);
            //セットアップ
            vfhTDT.setup_vfh_tdt_param(
                k1_tmp, k2_tmp, k3_tmp, 
                eta1_tmp, eta2_tmp, eta3_tmp,
                angle_min_tmp, angle_max_tmp, angle_div_tmp,
                distance_threshold_tmp, steer_r_tmp
            );
            // node_depthをセット
            //ラムダをセット
            vfhTDT.set_lamda();
            //ロボットパラメータをセット
            vfhTDT.set_robot_param(robotRadius_tmp, steer_r_tmp);
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
        void configCallback(local_navigation::obstacleAvoidanceConfig &config, uint32_t level){

            k1_tmp = config.K1;
            k2_tmp = config.K2;
            k3_tmp = config.K3;
            eta1_tmp = config.EtaG;
            eta2_tmp = config.EtaCurAngle;
            eta3_tmp = config.EtaPrevAngle;

            angle_min_tmp = config.minAngle;
            angle_max_tmp = config.maxAngle;
            angle_div_tmp = config.divAngle;
            distance_threshold_tmp = config.distanceThreshold;
            marginRadius = config.marginRadius;

            robotRadius = config. robotRadius_tmp;
            robotSteerRadius = config.steer_r_tmp;
            gainP = config.gain_p;
            default_vel = config.default_vel;

            vfhTDT.setup_vfh_tdt_param(
                k1_tmp, k2_tmp, k3_tmp, 
                eta1_tmp, eta2_tmp, eta3_tmp,
                angle_min_tmp, angle_max_tmp, angle_div_tmp,
                distance_threshold_tmp, marginRadius_tmp
            );
            vfhTDT.set_lamda();
            vfhTDT.set_robot_param(robotRadius_tmp, steer_r_tmp);
            main_loop();
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
            }
            //
            vfhTDT.cobine_open_close_node();
            target_num = vfhTDT.search_node_n(best_node_num);
            //最良ノードを格納
            best_node = get_node(target_node);
            
            //目標角度, 速度を取得
            target_vel = best_node.angle;//角度は算出したやつ
            target_angle = default_vel;//速度一定
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
};

int main(int argc,char **argv){
	ros::init(argc,argv,"vfh_tdt_node");
	
    run runCls;
    ros::spin();
	//--process
    
    //--
	return 0;
}