//多重インクルード防止
#ifndef INCLUDE_VFH_CLASS
#define INCLUDE_VFH_CLASS
#include<ros/ros.h>

class vfh{
    protected:
        std::vector<double> hst_dis;//距離ヒストグラム
        std::vector<bool> hst_bi;//バイナリヒストグラム
        float initDis;
        float angle_min, angle_max;//センサ測定可能角度
        float angle_center;//センサ中心角度
        float angle_div;//ヒストグラム解像度
        double min_cost;//最小コスト
        float selected_angle;//最小コスト角度
        float dis_threshold;//距離閾値: バイナリ作成用
        //cost 
        float eta_goal, k_goal;
        float eta_curAngle, k_curAngle;
        float eta_prevAngle, k_prevAngle;
    public:
        vfh(){//コンストラクタ
        }
        ~vfh(){//デストラクタ
        }
        //property
        void set_histgram_param(float& angMin, float& angMax, float& angDiv){
            angle_min = angMin;
            angle_max = angMax;
            angle_div = angDiv;
            angle_center = (angle_min+angle_max)/2.0;
            clear_histgram_dis();
            resize_histgram_dis( (int)((angle_max - angle_min)/angle_div) );
        }
        void set_dis_threshold(float& data){
            dis_threshold = data;    
        }
        void set_eta(float& goal, float& theta, float& omega){
                eta_goal = goal;
                eta_curAngle = theta;
                eta_prevAngle = omega;
        }
        void set_k(float& goal, float& theta, float& omega){
            k_goal = goal;
            k_curAngle = theta;
            k_prevAngle = omega;
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
        //transform
        int transform_angle_RobotToNum(float& angle){
            return ( (int)((angle - angle_min)/angle_div) );
        }
        float transform_numToAngle(int& num){
            return ( angle_min + num * angle_div) ;
        }
        float transform_angleFromCenter(float& angle){
            return (angle - angle_center);
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
        // create binary histgram
        void create_binary_histgram(float& robotRadius, float& marginRadius){
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
        float min_dif_angle_rad(const float& angle1, const float& angle2){
            float dif_angle_temp = angle1 - angle2;
            float dif_angle_p360 = std::abs(dif_angle_temp + 2*M_PI);
            float dif_angle_n360 = std::abs(dif_angle_temp - 2*M_PI);
            float dif_angle = std::abs(dif_angle_temp);
            float min_angle = dif_angle;
            if(min_angle > dif_angle_p360){
                min_angle = dif_angle_p360;
            }
            if(min_angle > dif_angle_n360){
                min_angle = dif_angle_n360;
            }
            return min_angle;
        }
        float min_dif_angle_deg(const float& angle1, const float& angle2){
            float dif_angle_temp = angle1 - angle2;
            float dif_angle_p360 = std::abs(dif_angle_temp + 360);
            float dif_angle_n360 = std::abs(dif_angle_temp - 360);
            float dif_angle = std::abs(dif_angle_temp);
            float min_angle = dif_angle;
            if(min_angle > dif_angle_p360){
                min_angle = dif_angle_p360;
            }
            if(min_angle > dif_angle_n360){
                min_angle = dif_angle_n360;
            }
            return min_angle;
        }
        //cost function 
        virtual double angleCostFunction_deg(float& eta, float value){
            return (pow(value/180.0/eta,2.0));
            // return (value/180.0/eta);
        }
        double cost_goal_angle_deg(const float& angle, float goal_angle){
            float dif_angle = min_dif_angle_deg(angle,goal_angle);
            return angleCostFunction_deg(eta_goal, dif_angle);
        }
        double cost_current_angle_deg(const float& angle, const float& cur_angle){
            float dif_angle = min_dif_angle_deg(angle, cur_angle);
            return angleCostFunction_deg(eta_curAngle, dif_angle);
        }
        double cost_prev_select_angle_deg(const float& angle, const float& pre_target_angle){
            float dif_angle = min_dif_angle_deg(angle, pre_target_angle);
            return angleCostFunction_deg(eta_prevAngle, dif_angle);
        }
        virtual double angleCostFunction_rad(float& eta, float value){
            return (pow(value/eta,2.0));
            // return (value/180.0/eta);
        }
        double cost_goal_angle_rad(const float& angle, float goal_angle){
            float dif_angle = min_dif_angle_rad(angle,goal_angle);
            return angleCostFunction_rad(eta_goal, dif_angle);
        }
        double cost_current_angle_rad(const float& angle, const float& cur_angle){
            float dif_angle = min_dif_angle_rad(angle, cur_angle);
            return angleCostFunction_rad(eta_curAngle, dif_angle);
        }
        double cost_prev_select_angle_rad(const float& angle, const float& pre_target_angle){
            float dif_angle = min_dif_angle_rad(angle, pre_target_angle);
            return angleCostFunction_rad(eta_prevAngle, dif_angle);
        }
        //
        double getCost_deg(float tagAng, float goalAng, float curAng, float prevTagAng){//to vfh class
            double goal_cost = cost_goal_angle_deg(tagAng, goalAng);
            double ang_cost = cost_current_angle_deg(tagAng, curAng);
            double prevAng_cost = cost_prev_select_angle_deg(tagAng, prevTagAng);
            double cost = k_goal*goal_cost + k_curAngle*ang_cost + k_prevAngle*prevAng_cost;
            return (cost);
        }
        double getCost_rad(float tagAng, float goalAng, float curAng, float prevTagAng){//to vfh class
            double goal_cost = cost_goal_angle_rad(tagAng, goalAng);
            double ang_cost = cost_current_angle_rad(tagAng, curAng);
            double prevAng_cost = cost_prev_select_angle_rad(tagAng, prevTagAng);
            double cost = k_goal*goal_cost + k_curAngle*ang_cost + k_prevAngle*prevAng_cost;
            return (cost);
        }
};
#endif