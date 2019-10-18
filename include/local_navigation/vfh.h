//多重インクルード防止
#ifndef INCLUDE_VFH_CLASS
#define INCLUDE_VFH_CLASS
#include<ros/ros.h>

class vfh{
    private:
        std::vector<double> hst_dis;//距離ヒストグラム
        std::vector<bool> hst_bi;//バイナリヒストグラム
        float initDis;
        float angle_min, angle_max;//センサ測定可能角度
        float angle_div;//ヒストグラム解像度
        double min_cost;//最小コスト
        float selected_angle;//最小コスト角度
        float dis_threshold;//距離閾値: バイナリ作成用
        //cost 
        float eta_goal, k_goal;
        float eta_theta, k_theta;
        float eta_omega, k_omega;
    public:
        vfh(){//コンストラクタ
        }
        ~vfh(){//デストラクタ
        }
        //transform
        int transform_angle_RobotToNum(float& angle){
            return ( (int)((angle - angle_min)/angle_div) );
        }
        float transform_numToAngle(int& num){
            return ( angle_min + num * angle_div) ;
        }
        void check_goalAng(double& goalAng){//-PI,PI系と0,PI系の問題を調整, 角度差の最大値補正
            if(goalAng < -90){
                goalAng+=360;
            }
            if (goalAng < angle_min )
            {
                goalAng = angle_min;
            }
            else if (goalAng > angle_max)
            {
                goalAng = angle_max;
            }
        }
        //add histgram element
        void add_histgram_dis(float& angle, float& dis){
            int num = transform_angle_RobotToNum(angle);
            //番号チェック
            if(num >= 0 && num < (int)(hst_dis.size())){
                //最小値なら格納
                if(hst_dis[num] > dis || hst_dis[num] == initDis){
                    hst_dis[num] = dis;
                }
            }
        }
        //clear
        void clear_histgram_dis(){
            hst_dis.clear();
        }
        //resize
        void resize_histgram_dis(int size){
            initDis = -1;
            hst_dis.resize(size, initDis);
        }
        void resize_histgram_dis(int size, float initValue){
            initDis = initValue;
            hst_dis.resize(size,initDis);
        }
        // create binary histgram
        void create_binary_histgram(float& robotRadius, float& marginRadius){//要修正：ロボットと障害物の幅を考慮
            hst_bi.clear();
            hst_bi.resize(hst_dis.size(), true);
            for(int k=0; k < hst_dis.size(); k++){
                if(!hst_bi[k]){
                    continue;
                }
                if(hst_dis[k] <= dis_threshold && hst_dis[k]!=initDis ){
                    hst_bi[k] = false;
                    //block width
                    double blockAng = atan2((robotRadius+marginRadius), hst_dis[k])*180/M_PI;
                    int blockNum = (int)(blockAng/angle_div)+1;
                    // ROS_INFO("blockAng, blockNum:(%f,%d)",blockAng,blockNum);
                    for(int i = k-blockNum/2; i <= k+blockNum/2; i++){
                        if(i<0 || i>(int)hst_dis.size()){
                            continue;
                        }
                        hst_bi[i] = false;
                    }
                }
            }
        }
        //cost function 
        double angleCostFunction(float& eta, float value){
            return (pow(value/180.0/eta,2.0));
            // return (value/180.0/eta);
        }
        double cost_goalAngle(float deltaAngle){
            return angleCostFunction(eta_goal, deltaAngle);
        }
        double cost_theta_depend_time(float deltaAngle){
            return angleCostFunction(eta_theta, deltaAngle);
        }
        double cost_omega_depend_time(float omegaAngle){
            return angleCostFunction(eta_omega, omegaAngle);
        }
        //property
        void set_histgram_param(float& angMin, float& angMax, float& angDiv){
            angle_min = angMin;
            angle_max = angMax;
            angle_div = angDiv;
            clear_histgram_dis();
            resize_histgram_dis( (int)((angle_max - angle_min)/angle_div) );
        }
        void set_dis_threshold(float& data){
            dis_threshold = data;    
        }
        void set_eta(float& goal, float& theta, float& omega){
                eta_goal = goal;
                eta_theta = theta;
                eta_omega = omega;
        }
        void set_k(float& goal, float& theta, float& omega){
            k_goal = goal;
            k_theta = theta;
            k_omega = omega;
        }
        void get_histgram_dis(std::vector<double>& data){
            data = hst_dis;
        }
        void get_histgram_bi(std::vector<bool>& data){
            data = hst_bi;
        }
        double get_min_cost(){
            return min_cost;
        }
        float get_selected_angle(){
            return selected_angle;
        }
};
#endif