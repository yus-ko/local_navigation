//多重インクルード防止
#ifndef INCLUDE_VFH_TDT_CLASS
#define INCLUDE_VFH_TDT_CLASS
#include<local_navigation/vfh.h>
#include <local_navigation/ClassificationVelocityData.h>
struct cost_node{
    int num;//ノード番号
    int parent_node;//親ノード
    int depth;//ノードの深さ
    double dx;//始点ノードからの移動距離x
    double dy;//始点ノードからの移動距離y
    double target_angle;//目標角度
    double delta_angle;//親ノードで選択した角度変化量angle
    double angle;//ノードの角度
    double v;//ロボットの速度
    double cost;//総コスト
};
class vfh_tdt : public vfh
{
    private:
        float timeRange;
        float timeDiv;
        int branch_num;
        int node_size;
        float angleThresholdMin;
        float angleThresholdMax;
        float ds;//１ステップの距離: 1ステップの経過時間 ts = ds/ロボットの速度
        float initVel;//初期速度
        float time_range;//探索を行う最大時間
        int node_depth;//ノードの深さ
        float distance_threshold;//バイナリヒストグラムの距離閾値
        float marginRadius;//距離マージン
        float angle_min, angle_max, angle_div;//ヒストグラム角度範囲
        float k1,k2,k3;//goal, theta, angleVel
        float eta1,eta2,eta3;//goal, theta, angleVel
        //ロボットパラメータ
        float robotRadius, steer_r;//ロボット半径, ステアリング半径
        std::vector<float> lamda_array;//ラムダ
        std::vector<cost_node> openNode;
        std::vector<cost_node> closedNode;
        std::vector<cost_node> conbineNode;
        cost_node startNode;
        cost_node goalNode;
        //クラスターデータ
        local_navigation::ClassificationVelocityData clstr;
    public:
        vfh_tdt(){
            // set_k(k1,k2,k3);
            // set_eta(eta1,eta2,eta3);
            // set_histgram_param(angle_min, angle_max, angle_div);
            // set_dis_threshold(distance_threshold);
            // steer_r = 0.31425;
        }
        vfh_tdt(float& k1_tmp, float& k2_tmp, float& k3_tmp, 
            float& eta1_tmp, float& eta2_tmp, float& eta3_tmp,
            float& angle_min_tmp, float& angle_max_tmp, float& angle_div_tmp,
            float& distance_threshold_tmp,float& steer_r_tmp,
            float& robotRadius_tmp, float& marginRadius_tmp
            ){
                setup_vfh_tdt_param(
                    k1_tmp, k2_tmp, k3_tmp, 
                    eta1_tmp, eta2_tmp, eta3_tmp,
                    angle_min_tmp, angle_max_tmp, angle_div_tmp,
                    distance_threshold_tmp, steer_r_tmp,
                    robotRadius_tmp, marginRadius_tmp
                );
        }
        ~vfh_tdt(){

        }
        void setup_vfh_tdt_param(
            float& k1_tmp, float& k2_tmp, float& k3_tmp, 
            float& eta1_tmp, float& eta2_tmp, float& eta3_tmp,
            float& angle_min_tmp, float& angle_max_tmp, float& angle_div_tmp,
            float& distance_threshold_tmp, float& steer_r_tmp,
            float& robotRadius_tmp, float& marginRadius_tmp
            ){

            k1 = k1_tmp;
            k2 = k2_tmp;
            k3 = k3_tmp;
            eta1 = eta1_tmp;
            eta2 = eta2_tmp;
            eta3 = eta3_tmp;
            angle_min = angle_min_tmp;
            angle_max = angle_max_tmp;
            angle_div = angle_div_tmp;
            distance_threshold = distance_threshold_tmp;
            marginRadius = marginRadius_tmp;
            set_robot_param(robotRadius_tmp,steer_r_tmp);
            set_k(k1,k2,k3);
            set_eta(eta1,eta2,eta3);
            set_histgram_param(angle_min, angle_max, angle_div);
            set_dis_threshold(distance_threshold);
            // steer_r = 0.31425;

        }
        void set_lamda(int node_depth_tmp){
            node_depth = node_depth_tmp;
            lamda_array.resize(node_depth);
            for(int k=0;k<lamda_array.size();k++){
                //線形
                lamda_array[k] =1 - k * 1.0/( (float)lamda_array.size()) ;  
                // std::cout<<"lamda_array["<<k<<"] = "<<lamda_array[k]<<std::endl;
            }
        }
        void set_initial_velocity(float init_vel_tmp){
            initVel = init_vel_tmp;
        }
        void set_delta_step(float delta_step_tmp){
            ds = delta_step_tmp;
            //ターゲット角度閾値も決める
            angleThresholdMax = M_PI_2 + M_PI_4;//M_PI_2 + ds/ steer_r;
            angleThresholdMin = M_PI_2 - M_PI_4;//M_PI_2 - ds/ steer_r;
        }
        void set_robot_param(float& robotRadius_tmp, float& steer_r_tmp){
            robotRadius = robotRadius_tmp;
            steer_r = steer_r_tmp;
        }
        void set_cluster_data(local_navigation::ClassificationVelocityData& cluster_data){
            clstr = cluster_data;
        }
        void create_start_node(){
            startNode.num = 0;
            startNode.parent_node = -1;
            startNode.depth = 0;
            startNode.dx = 0;
            startNode.dy = 0;
            startNode.target_angle = M_PI_2;
            startNode.delta_angle = 0;
            startNode.angle = M_PI_2;
            startNode.v = initVel;
            startNode.cost = 0;
        }
        void create_goal_node(float goal_x, float goal_y){
            goalNode.num = -1;
            goalNode.parent_node = -1;
            goalNode.depth = 0;
            goalNode.dx = goal_x;
            goalNode.dy = goal_y;
            startNode.target_angle = M_PI_2;
            goalNode.delta_angle = 0;
            goalNode.angle = M_PI_2;
            goalNode.v = initVel;
            goalNode.cost = 0;
        }
        void create_goal_node(double goal_x, double goal_y){
            goalNode.num = -1;
            goalNode.parent_node = -1;
            goalNode.depth = 0;
            goalNode.dx = goal_x;
            goalNode.dy = goal_y;
            startNode.target_angle = M_PI_2;
            goalNode.delta_angle = 0;
            goalNode.angle = M_PI_2;
            goalNode.v = initVel;
            goalNode.cost = 0;
        }
        void add_start_node(){
            openNode.emplace_back(startNode);
            node_size = 1;
        }
        int get_min_cost_node(){
            int n;
            //Openノードをコストの大きさで昇順ソート
            std::sort(openNode.begin(),openNode.end(), [](const cost_node& a, const cost_node& b) {
                return (a.cost < b.cost);
            });//昇順ソート
            //先頭要素のノード番号を取得
            n=openNode[0].num;
            return n;
        }
        bool check_search_finish(){
            //条件
            //get_min_cost_nodeで配列をソート後に行う
            //先頭ノードが最小コストノード(次に子ノードを作成すべき親ノード)
            //になっているため, 先頭ノードの深さが探索最大深度であるとき
            //そのノードが最良ノードであるといえる
            // std::cout<<"openNode[0].depth:"<<openNode[0].depth<<std::endl;
            // std::cout<<"node_depth:"<<node_depth<<std::endl;
            if((int)openNode.size()==0){
                return true;
            }
            return ( openNode[0].depth + 1 >= node_depth ? true : false);
        }
        int get_best_node(){
            if((int)openNode.size()==0){
                //クローズドノード内で最も深いノードを取得
                //深さでソート
                std::sort(closedNode.begin(),closedNode.end(), [](const cost_node& a, const cost_node& b) {
                    return (a.depth > b.depth);
                });//降順ソート
                //ノード深く、コストが最も低いノードのベストノードとする
                int count=0;
                int best_node_num = closedNode[count].num;
                double best_node_cost = closedNode[count].cost;
                int max_depth = closedNode[count++].depth;
                if(max_depth == 0){
                    return(0);
                }
                else{
                    int ref_depth = closedNode[count].depth;
                    while(ros::ok()&&max_depth == ref_depth){
                        //コスト比較
                        int ref_node_cost = closedNode[count].cost;
                        if(best_node_cost > ref_node_cost){
                            best_node_num = closedNode[count].num;
                            best_node_cost = closedNode[count].cost;
                        }
                        //次の参照値
                        count++;
                        ref_depth = closedNode[count].depth;
                    }
                }
                return best_node_num;
            }
            else{
                return(openNode[0].num);
            }
        }
        cost_node& get_node(int Num){
            return openNode[Num];
        }
        cost_node& get_conbine_node(int Num){
            return conbineNode[Num];
        }
        std::vector<cost_node>& get_open_node(){
            return openNode;
        }
        std::vector<cost_node>& get_closed_node(){
            return closedNode;
        }
        std::vector<cost_node>& get_conbine_node(){
            return conbineNode;
        }
        cost_node& get_goalNode(){
            return goalNode;
        }
        
        void add_node(cost_node& parent_node){
            //距離ヒストグラムを作成
            // create_histgram_dis(parent_node);
            //バイナリヒストグラムを作成
            // create_binary_histgram(float& robotRadius, float& marginRadius);
            create_binary_histgram(parent_node, robotRadius, marginRadius);
            // std::vector<bool> hst_bi;
            // get_histgram_bi(hst_bi);
            //
            //追加予定数を算出
            int add_node_size = 0;
            for(int k=0; k<hst_bi.size();k++){
                if(hst_bi[k]){
                    add_node_size ++;
                }
            }
            //resize
            int bef_size = (int)openNode.size();
            openNode.resize((int)openNode.size()+add_node_size);
            auto itr = openNode.begin()+bef_size;//bef_size番目のノードを指す
            //code
            for(int k=0; k<hst_bi.size();k++){
                if(!hst_bi[k]){
                    continue;
                }
                //深さゼロでのロボット座標
                float goal_angle = atan2(goalNode.dy-parent_node.dy, goalNode.dx-parent_node.dx);
                //vfhコスト算出
                float target_angle = transform_numToAngle(k);//深さnでのロボット座標系
                float delta_angle = transform_angleFromCenter(target_angle);
                //座標変換 taget_angle(n) = delta target_angle(n) from cp(n) + angle of parent_node 
                target_angle = delta_angle + parent_node.angle;//深さn=0でのロボット座標系
                //
                double cost = getCost(target_angle, goal_angle, parent_node, delta_angle);
                //状態遷移のための位置, 姿勢角度を算出
                cost_node node_temp;
                node_temp.num = node_size++;
                node_temp.parent_node = parent_node.num;
                node_temp.depth = parent_node.depth + 1;
                node_temp.angle = transform_AnglePrevToCur(parent_node.angle, delta_angle);
                node_temp.dx = parent_node.dx + parent_node.v*cos(parent_node.angle + delta_angle)*ds/parent_node.v;
                node_temp.dy = parent_node.dy + parent_node.v*sin(parent_node.angle + delta_angle)*ds/parent_node.v;
                node_temp.v = parent_node.v;
                node_temp.target_angle = target_angle;
                node_temp.delta_angle = delta_angle;
                node_temp.cost = parent_node.cost + lamda_array[node_temp.depth] * cost + huristic_function(parent_node,node_temp,goal_angle);
                //オープンノードリストに追加
                // openNode.emplace_back(node_temp);
                *itr = node_temp;
                itr++;
            }
            //親ノード(この関数の引数のノード)をクローズリストに移動
            closedNode.emplace_back(openNode[0]);//親ノードは最小コストノード(先頭ノード)[0]
            //openNodeから移動したノードを削除
            openNode.erase(openNode.begin());
        }
        float transform_AnglePrevToCur(const float& previous_angle, const float& delta_angle){
            // float ds;//１ステップの距離: 1ステップの経過時間 ts = ds/ロボットの速度
            // steer_r:最小ステアリング半径
            // 
            if(std::abs(delta_angle) < ds / steer_r){
                return previous_angle + delta_angle;
            }
            else if(delta_angle > 0){
                return previous_angle + ds / steer_r;
            }
            else{
                return previous_angle - (ds / steer_r);
            }
        }
        //--予測コスト(ヒューリスティック関数)
        double huristic_function(const cost_node& parent_node, const cost_node& node, const float& goal_angle){
            //ベクトルktとの角度差を用いる
            //kt：ゴールまでの角度
            //lamda:ノードの深さ（時間）に対するカットオフ重み
            //0<lamda<=1, ノードが深いほど小さくする
            //コスト関数
            //lamda * 
            // ( k1 * (時刻tでのgoalと時刻t+1での予測角度の角度差) 
            // + k2 * (時刻tでのgoalと選択角度の角度差) 
            // + k3 * (時刻tでのgoalと時刻t-1での選択角度の角度差) 
            //)
            float cost1 = min_dif_angle_rad(goal_angle, node.angle);
            float cost2 = min_dif_angle_rad(goal_angle, node.target_angle);
            float cost3 = min_dif_angle_rad(goal_angle, parent_node.target_angle);
            double cost = lamda_array[node.depth] * 
                ( k1 * cost1
                + k2 * cost2
                + k3 * cost3
                );
            return (cost);
        }
        //クローズノードとオープンノードを統合し, ノード番号でソートする
        //親ノードを参照するときに, いちいち探索をするのではなく, cobine_node[parent_node]で
        //参照すれば良くなる
        void cobine_open_close_node(){
            //code
            //openNodeコピー
            conbineNode.resize(openNode.size());
            std::copy(openNode.begin(), openNode.end(), conbineNode.begin());
            //closedNode挿入
            conbineNode.reserve(openNode.size() + closedNode.size()); //キャパ確保
            std::copy(closedNode.begin(), closedNode.end(), std::back_inserter(conbineNode));
            //cobineNodeをソート
            //ノード番号で昇順ソート
            std::sort(conbineNode.begin(),conbineNode.end(), [](const cost_node& a, const cost_node& b) {
                return (a.num < b.num);
            });//昇順ソート
        }
        //最小コストノードから目標位置と角度を見つける
        //選択可能条件(旋回可能上限角度)を満たしてるノードまで参照していく
        //角度条件を満たすノードを出力
        //cobine_open_close_nodeを行った後に実行
        int search_node_n(int& min_node_num){
            //conbineNodeからノードをさかのぼり, 
            //最終的な目標位置, 角度を取得
            cost_node min_node = conbineNode[min_node_num];
            int nodeNum = min_node.num;
            int preNodeNum = min_node.parent_node;
            int selectNodeNum;
            std::vector<int> nextNodeIndex;
            nextNodeIndex.reserve(node_depth+1);
            //先頭から挿入
            auto it = nextNodeIndex.begin();
            it = nextNodeIndex.insert(it, nodeNum);
            if(preNodeNum == -1){
                return nodeNum;
            }
            //先頭からのインデックスを作成
            while (preNodeNum != 0)
            {
                //親ノードを取得
                // ROS_INFO("node[%d] --> node[%d]: (%f,%f,%f)-->(%f,%f,%f)",
                //     nodeNum,preNodeNum,
                //     conbineNode[nodeNum].dx, conbineNode[nodeNum].dy, conbineNode[nodeNum].angle,
                //     conbineNode[preNodeNum].dx, conbineNode[preNodeNum].dy, conbineNode[preNodeNum].angle
                // );
                nodeNum = conbineNode[preNodeNum].num;
                it = nextNodeIndex.insert(it, nodeNum);
                preNodeNum = conbineNode[preNodeNum].parent_node;
            }
            //先頭ノード0から最小コストノードまでのインデックスが完成
            //nextNodeIndex[0]:深さ1のノード -> nextNodeIndex[1]:深さ2のノード...
            for(int k=0; k<nextNodeIndex.size();k++){
                //条件に合うノードまで進める
                int nodeNum = nextNodeIndex[k];
                //
                std::cout<<"to node "<<nodeNum <<":"<<std::endl
                    <<"\tparent: "<<conbineNode[nodeNum].parent_node<<std::endl
                    <<"\tnum: "<<conbineNode[nodeNum].num<<std::endl
                    <<"\tdepth: "<<conbineNode[nodeNum].depth<<std::endl
                    <<"\tdx,dy: "<<conbineNode[nodeNum].dx<<","<<conbineNode[nodeNum].dy<<std::endl
                    <<"\tv,ang: "<<conbineNode[nodeNum].v<<","<<conbineNode[nodeNum].angle<<std::endl
                    <<"\tangT,angD: "<<conbineNode[nodeNum].target_angle<<","<<conbineNode[nodeNum].delta_angle<<std::endl
                    <<"\tcost: "<<conbineNode[nodeNum].cost<<std::endl
                    <<"\tgoal: "<<goalNode.dx-conbineNode[nodeNum].dx<<", "<<goalNode.dy-conbineNode[nodeNum].dy<<", "<<atan2(goalNode.dy-conbineNode[nodeNum].dy, goalNode.dx-conbineNode[nodeNum].dx)<<std::endl
                <<std::endl;
                //
                //条件を超える角度の時
                if(conbineNode[nodeNum].angle > angleThresholdMax || conbineNode[nodeNum].angle < angleThresholdMin){
                    selectNodeNum = conbineNode[nodeNum].parent_node;//1つ前のノードがベスト
                    //探索終了
                    ROS_INFO("inIf selectNodeNum:%d",selectNodeNum);
                    return selectNodeNum;
                }
            }
            //最後のノードまで条件を超えなかった時
            selectNodeNum = conbineNode[nextNodeIndex[(int)nextNodeIndex.size() - 1]].num;//
            ROS_INFO("selectNodeNum:%d",selectNodeNum);
            return selectNodeNum;            
        }
        int search_node_n(int& min_node_num, std::vector<int>& nextNodeIndex){
            //conbineNodeからノードをさかのぼり, 
            //最終的な目標位置, 角度を取得
            nextNodeIndex.clear();
            cost_node min_node = conbineNode[min_node_num];
            int nodeNum = min_node.num;
            int preNodeNum = min_node.parent_node;
            int selectNodeNum;
            // std::vector<int> nextNodeIndex;
            nextNodeIndex.reserve(node_depth+1);
            //先頭から挿入
            auto it = nextNodeIndex.begin();
            it = nextNodeIndex.insert(it, nodeNum);
            //先頭からのインデックスを作成
            while (preNodeNum != 0)
            {
                //親ノードを取得
                // ROS_INFO("node[%d] --> node[%d]: (%f,%f,%f)-->(%f,%f,%f)",
                //     nodeNum,preNodeNum,
                //     conbineNode[nodeNum].dx, conbineNode[nodeNum].dy, conbineNode[nodeNum].angle,
                //     conbineNode[preNodeNum].dx, conbineNode[preNodeNum].dy, conbineNode[preNodeNum].angle
                // );
                nodeNum = conbineNode[preNodeNum].num;
                it = nextNodeIndex.insert(it, nodeNum);
                preNodeNum = conbineNode[preNodeNum].parent_node;
            }
            //先頭ノード0から最小コストノードまでのインデックスが完成
            //nextNodeIndex[0]:深さ1のノード -> nextNodeIndex[1]:深さ2のノード...
            for(int k=0; k<nextNodeIndex.size();k++){
                //条件に合うノードまで進める
                int nodeNum = nextNodeIndex[k];
                //
                std::cout<<"to node "<<nodeNum <<":"<<std::endl
                    <<"\tparent: "<<conbineNode[nodeNum].parent_node<<std::endl
                    <<"\tnum: "<<conbineNode[nodeNum].num<<std::endl
                    <<"\tdepth: "<<conbineNode[nodeNum].depth<<std::endl
                    <<"\tdx,dy: "<<conbineNode[nodeNum].dx<<","<<conbineNode[nodeNum].dy<<std::endl
                    <<"\tv,ang: "<<conbineNode[nodeNum].v<<","<<conbineNode[nodeNum].angle<<std::endl
                    <<"\tangT,angD: "<<conbineNode[nodeNum].target_angle<<","<<conbineNode[nodeNum].delta_angle<<std::endl
                    <<"\tcost: "<<conbineNode[nodeNum].cost<<std::endl
                    <<"\tgoal: "<<goalNode.dx-conbineNode[nodeNum].dx<<", "<<goalNode.dy-conbineNode[nodeNum].dy<<", "<<atan2(goalNode.dy-conbineNode[nodeNum].dy, goalNode.dx-conbineNode[nodeNum].dx)<<std::endl
                <<std::endl;
                //
                //条件を超える角度の時
                if(conbineNode[nodeNum].angle > angleThresholdMax || conbineNode[nodeNum].angle < angleThresholdMin){
                    selectNodeNum = conbineNode[nodeNum].parent_node;//1つ前のノードがベスト
                    //探索終了
                    ROS_INFO("inIf selectNodeNum:%d",selectNodeNum);
                    return selectNodeNum;
                }
            }
            //最後のノードまで条件を超えなかった時
            selectNodeNum = conbineNode[nextNodeIndex[(int)nextNodeIndex.size() - 1]].num;//
            ROS_INFO("selectNodeNum:%d",selectNodeNum);
            return selectNodeNum;            
        }

        //protected vfh class method
        // create binary histgram
        void create_binary_histgram(cost_node& pNode, float& robotRadius, float& marginRadius){
            hst_bi.clear();
            hst_bi.resize((int)((angle_max - angle_min)/angle_div), true);
            //経過時間 ds/pNode.v * pNode.depthを考慮(ds/pNode.v == ts)
            //クラスタ変数をどこで管理するかが未定
            //
            //ロボット位置
            float point_xr = pNode.dx;
            float point_yr = pNode.dy;
            //
            for(int k =0; k < clstr.data.size(); k++){//クラスタ数
                //各クラスタに含まれる点群を取得しヒストグラムを作成
                for(int m = 0; m < clstr.data[k].pt.size(); m++){
                    //障害物
                    float point_x = clstr.data[k].pt[m].x + clstr.twist[k].linear.x * ds/pNode.v*pNode.depth;
                    float point_y = clstr.data[k].pt[m].y + clstr.twist[k].linear.y * ds/pNode.v*pNode.depth;
                    float point_difx = point_x - point_xr;
                    float point_dify = point_y - point_yr;
                    float angleTemp = atan2(point_dify, point_difx);
                    float disTemp = sqrt(pow(point_difx,2.0) + pow(point_dify,2.0));
                    //距離に応じてブロック範囲を変更する(ノード深さゼロを除く)
                    int blockNum;
                    if(disTemp > distance_threshold){
                        continue;
                    }
                    if(disTemp > robotRadius+marginRadius || pNode.depth==0){
                        double blockAng = atan2((robotRadius+marginRadius), disTemp);
                        blockNum = (int)(blockAng/angle_div)+1;
                    }
                    else if(disTemp > robotRadius){
                        double blockAng = M_PI_2;
                        blockNum = (int)(blockAng/angle_div)+1;
                    }
                    else{
                        double blockAng = M_PI;
                        blockNum = (int)(blockAng/angle_div)+1;
                    }
                    int n = transform_angle_RobotToNum(angleTemp);
                    for(int i = n-blockNum/2; i <= n+blockNum/2; i++){
                        if(i<0 || i>=(int)hst_bi.size()){
                            continue;
                        }
                        hst_bi[i] = false;
                    }
                }
            }
        }
        double getCost(const float& tagAng, const float& goalAng, const cost_node& parent_node, const float& cur_target_delta_angle){//要修正
            // ベクトルktとの角度差を用いる
            //kt：ゴールまでの角度
            //ノード深さ = 0 : 重み lamda = 1
            // ノードの深さ > 0 :重みlamdaを用いる
            //lamda:ノードの深さ（時間）に対するカットオフ重み
            //0<lamda<=1, ノードが深いほど小さくする
            //コスト関数
            //lamda * 
            // ( k1 * ( (時刻tでのgoalと時刻t+1での予測角度の角度差) or (時刻tでのgoalと選択角度の角度差) の大きい方)
            // + k2 * (現在の角度と選択角度の角度差) 
            // + k3 * (現在の選択角度と1つ前の選択角度の角度差) 
            //)
            //ノード深さ = 0のときの第1項は
            // ( k1 * (時刻tでのgoalと選択角度の角度差)
            double goal_cost = cost_goal_angle(tagAng, parent_node.angle + cur_target_delta_angle, goalAng);
            double ang_cost = cost_current_angle_rad(tagAng, parent_node.angle);
            double prevAng_cost = cost_prev_select_angle_rad(cur_target_delta_angle, parent_node.delta_angle);
            double cost = lamda_array[parent_node.depth+1]*( k1*goal_cost + k2*ang_cost + k3*prevAng_cost );
            return cost;
        }
        double cost_goal_angle(const float& target_angle,const float angle, const float& goal_angle){
            // std::cout<<"min_dif_angle_rad("<<target_angle<<","<<goal_angle<<") "<<std::endl;
            // std::cout<<"min_dif_angle_rad("<<angle<<","<<goal_angle<<") "<<std::endl;
            float dif_angle1 = min_dif_angle_rad(target_angle,goal_angle);
            float dif_angle2 = min_dif_angle_rad(angle,goal_angle);
            // if(dif_angle1 > dif_angle2){
            // std::cout<<"if("<<"abs("<<dif_angle1<<") > abs("<<dif_angle2<<") )"<<std::endl;
            if(std::abs(dif_angle1) > std::abs(dif_angle2)){
                return angleCostFunction(eta_goal, dif_angle1);
            }
            else{
                return angleCostFunction(eta_goal, dif_angle2);
            }
        }
        //論文通り線形誤差を使用
        double angleCostFunction(float& eta, float value){
            // return (pow(value/M_PI/eta,2.0));
            return (value);
        }
        void clear_node(){
            openNode.clear();
            closedNode.clear();
            conbineNode.clear();
            // openNode.reserve(100000);
            // closedNode.reserve(100000);
            // conbineNode.reserve(100000);
        }
        //get
        int get_open_node_size(){
            return (int)openNode.size();
        }
        int get_closed_node_size(){
            return (int)closedNode.size();
        }
        //デバッグ
        void debug_add_node(cost_node parent_node, local_navigation::ClassificationVelocityData &debug_clstr){
            //距離ヒストグラムを作成
            // std::cout<<"parent_node:\n"
            //     <<"\tnum: "<<parent_node.num<<std::endl
            //     <<"\tdepth: "<<parent_node.depth<<std::endl
            //     <<"\tdx,dy: "<<parent_node.dx<<","<<parent_node.dy<<std::endl
            //     <<"\tv,ang: "<<parent_node.v<<","<<parent_node.angle<<std::endl
            //     <<"\tangT,angD: "<<parent_node.target_angle<<","<<parent_node.delta_angle<<std::endl
            //     <<"\tds,cost: "<<ds<<","<<parent_node.cost<<std::endl
            //     <<"\tgoal: "<<goalNode.dx-parent_node.dx<<", "<<goalNode.dy-parent_node.dy<<", "<<atan2(goalNode.dy-parent_node.dy, goalNode.dx-parent_node.dx)<<std::endl
            // <<std::endl;
            // create_histgram_dis(parent_node);
            //バイナリヒストグラムを作成
            // create_binary_histgram(float& robotRadius, float& marginRadius);
            debug_create_binary_histgram(parent_node, robotRadius, marginRadius,debug_clstr);
            // std::vector<bool> hst_bi;
            // get_histgram_bi(hst_bi);
            //追加予定数を算出
            int add_node_size = 0;
            for(int k=0; k<hst_bi.size();k++){
                if(hst_bi[k]){
                    add_node_size ++;
                }
            }
            //resize
            int bef_size = (int)openNode.size();
            openNode.resize((int)openNode.size()+add_node_size);
            auto itr = openNode.begin()+bef_size;//bef_size番目のノードを指す
            // code
            for(int k=0; k<hst_bi.size();k++){
                if(!hst_bi[k]){
                    continue;
                }
                //深さゼロでのロボット座標
                float goal_angle = atan2(goalNode.dy-parent_node.dy, goalNode.dx-parent_node.dx);
                //vfhコスト算出
                float target_angle = transform_numToAngle(k);//深さnでのロボット座標系
                float delta_angle = transform_angleFromCenter(target_angle);
                //座標変換 taget_angle(n) = delta target_angle(n) from cp(n) + angle of parent_node 
                target_angle = delta_angle + parent_node.angle;//深さn=0でのロボット座標系
                //
                double cost = getCost(target_angle, goal_angle, parent_node, delta_angle);
                //状態遷移のための位置, 姿勢角度を算出
                cost_node node_temp;
                node_temp.num = node_size++;
                node_temp.parent_node = parent_node.num;
                node_temp.depth = parent_node.depth + 1;
                node_temp.angle = transform_AnglePrevToCur(parent_node.angle, delta_angle);
                node_temp.dx = parent_node.dx + parent_node.v*cos(parent_node.angle + delta_angle)*ds/parent_node.v;
                node_temp.dy = parent_node.dy + parent_node.v*sin(parent_node.angle + delta_angle)*ds/parent_node.v;
                node_temp.v = parent_node.v;
                node_temp.target_angle = target_angle;
                node_temp.delta_angle = delta_angle;
                node_temp.cost = parent_node.cost + lamda_array[node_temp.depth] * cost + huristic_function(parent_node,node_temp,goal_angle);
                //オープンノードリストに追加
                // openNode.emplace_back(node_temp);
                // openNode[itr++] = node_temp;
                *itr = node_temp;
                itr++;
            }
            //親ノード(この関数の引数のノード)をクローズリストに移動
            closedNode.emplace_back(openNode[0]);//親ノードは最小コストノード(先頭ノード)[0]
            //openNodeから移動したノードを削除
            openNode.erase(openNode.begin());
        }
        void debug_create_binary_histgram(cost_node& pNode, float& robotRadius, float& marginRadius, local_navigation::ClassificationVelocityData &debug_clstr){
            hst_bi.clear();
            hst_bi.resize((int)((angle_max - angle_min)/angle_div), true);
            //経過時間 ds/pNode.v * pNode.depthを考慮(ds/pNode.v == ts)
            //クラスタ変数をどこで管理するかが未定
            //
            //ロボット位置
            float point_xr = pNode.dx;
            float point_yr = pNode.dy;
            // std::cout<<"depth:"<<pNode.depth<<"--("<<point_xr<<","<<point_yr<<") ["<< ds/pNode.v*pNode.depth<<"]\n";
            //
            for(int k =0; k < debug_clstr.data.size(); k++){//クラスタ数
                //各クラスタに含まれる点群を取得しヒストグラムを作成
                // float gcX = debug_clstr.data[k].gc.x + debug_clstr.twist[k].linear.x * ds/pNode.v*pNode.depth;
                // float gcY = debug_clstr.data[k].gc.y + debug_clstr.twist[k].linear.y * ds/pNode.v*pNode.depth;
                // std::cout<<"(gc),(Xr)--(dx)"<<k<<":("<<gcX<<","<<gcY<<"),("<<point_xr<<","<<point_yr<<")--("<<gcX-point_xr<<","<<gcY-point_yr<<")\n";
                // std::cout<<"dis,angle clstr "<<k<<":("<<sqrt(pow(gcX-point_xr,2.0) + pow(gcY-point_yr,2.0))<<","<<atan2(gcY-point_yr,gcX-point_xr)<<")\n";
                for(int m = 0; m < debug_clstr.data[k].pt.size(); m++){
                    //障害物
                    // std::cout<< debug_clstr.data[k].pt[m].x<<","<< debug_clstr.data[k].pt[m].y<<" -- "<<ds/pNode.v*pNode.depth<<std::endl;
                    float point_x = debug_clstr.data[k].pt[m].x + debug_clstr.twist[k].linear.x * ds/pNode.v*pNode.depth;
                    float point_y = debug_clstr.data[k].pt[m].y + debug_clstr.twist[k].linear.y * ds/pNode.v*pNode.depth;
                    // float point_x = debug_clstr.data[k].pt[m].x + debug_clstr.twist[k].linear.y * ds/pNode.v*pNode.depth;
                    // float point_y = debug_clstr.data[k].pt[m].y + (-debug_clstr.twist[k].linear.x) * ds/pNode.v*pNode.depth;
                    float point_difx = point_x - point_xr;
                    float point_dify = point_y - point_yr;
                    float angleTemp = atan2(point_dify, point_difx) + (pNode.target_angle-startNode.target_angle);
                    float disTemp = sqrt(pow(point_difx,2.0) + pow(point_dify,2.0));
                    //距離に応じてブロック範囲を変更する
                    int blockNum;
                    if(disTemp > dis_threshold ){
                        // double blockAng = atan2((robotRadius+marginRadius), disTemp);
                        // blockNum = (int)(blockAng/angle_div)+1;
                        continue;
                    }
                    else if(disTemp > robotRadius+marginRadius ){
                        double blockAng = atan2((robotRadius+marginRadius), disTemp);
                        blockNum = (int)(blockAng/angle_div)+1;
                    }
                    else if(disTemp > robotRadius){
                        double blockAng = M_PI_2;
                        blockNum = (int)(blockAng/angle_div)+1;
                    }
                    else{
                        double blockAng = M_PI;
                        blockNum = (int)(blockAng/angle_div)+1;
                    }
                    int n = transform_angle_RobotToNum(angleTemp);
                    for(int i = n-blockNum/2; i <= n+blockNum/2; i++){
                        if(i<0 || i>=(int)hst_bi.size()){
                            continue;
                        }
                        hst_bi[i] = false;
                    }
                }
            }
        }
};
#endif