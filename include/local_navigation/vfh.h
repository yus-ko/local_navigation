//多重インクルード防止
#ifndef INCLUDE_VFH_CLASS
#define INCLUDE_VFH_CLASS
#include<ros/ros.h>

class vfh{
    private:
        std::vector<float> hst_dis;//距離ヒストグラム
        std::vector<bool> hst_bi;//バイナリヒストグラム
        float initDis;
        float angle_min, angle_max;//センサ測定可能角度
        float angle_div;//ヒストグラム解像度
        float min_cost;//最小コスト
        float selected_angle;//最小コスト角度
        float dis_threshold;//距離閾値: バイナリ作成用
    public:
        vfh();//コンストラクタ
        ~vfh(){//デストラクタ
        }
        //transform
        int transform_angle_RobotToNum(float& angle){
            return ( (int)((angle - angle_min)/angle_div) );
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
        void create_binary_histgram(){
            hst_bi.clear();
            hst_bi.resize(hst_dis.size(), true);
            for(int k=0; k < hst_dis.size(); k++){
                if(hst_dis[k] <= dis_threshold){
                    hst_bi[k] = false;
                }
            }
        }
        void set_histgram_param(float& angMin, float& angMax, float& angDiv){
            angle_min = angMin;
            angle_max = angMax;
            angle_div = angDiv;
            clear_histgram_dis();
            resize_histgram_dis( ((angle_max - angle_min)/angle_div) );
        }
        
};