#include<vfh.h>

class vfh_tdt : public vfh
{
    private:
        float timeRange;
        float timeDiv;
        int branch_num;
        int node_size;
        float angleThresholdMin;
        float angleThresholdMax;
        float dt;//１ステップの経過時間
        float time_range;//探索を行う最大時間
        struct cost_node{
            int num;//ノード番号
            int prev_node;//親ノード
            float t;//経過時間
            float dx;//始点ノードからの移動距離x
            float dy;//始点ノードからの移動距離y
            float angle;//始点ノードからの角度変化量angle
            float angle_vel;//角速度
            double cost;//総コスト
        };
        std::vector<cost_node> openNode;
        std::vector<cost_node> closedNode;
        std::vector<cost_node> conbineNode;
        cost_node startNode;
        cost_node goalNode;
    public:
        void create_start_node(){
            startNode.num = 0;
            startNode.prev_node = -1;
            startNode.dx = 0;
            startNode.dy = 0;
            startNode.angle = M_PI_2;
            startNode.cost = 0;
        }
        void create_goal_node(float goal_x, float goal_y){
            goalNode.num = -1;
            goalNode.prev_node = -1;
            goalNode.dx = goal_x;
            goalNode.dy = goal_y;
            goalNode.angle = M_PI_2;
            goalNode.cost = 0;
        }
        void add_start_node(){
            openNode.push_back(startNode);
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
        void add_node(cost_node& prev_node){
            //add対象のノードかをチェック
            if(!check_add_node(prev_node)){
                //このノードは探索終了
                //openNodeからノードを削除
                openNode.erase(openNode.begin());
            }
            //距離ヒストグラムを作成
            create_histgram_dis(prev_node.t);
            //バイナリヒストグラムを作成
            create_binary_histgram(float& robotRadius, float& marginRadius);
            std::vector<bool> hst_bi;
            get_histgram_bi(hst_bi);
            //code
            for(int k=0; k<hst_bi.size();k++){
                //
                float goal_angle = atan2(goalNode.y-prev_node.dy, goalNode.x-prev_node.dx);
                check_goalAng(goal_angle);
                //vfhコスト算出
                float angle = transform_numToAngle(k);//ロボット座標系
                float angle_from_c = transform_angleFromCenter(angle);
                float angle_vel = controler(angle_from_c);
                float angle_vel_prev = prev_node.angle_vel;
                double cost = getCost(angle, angle_from_c, angle_vel - angle_vel_prev);
                //状態遷移のための位置, 姿勢角度を算出
                cost_node node_temp;
                node_temp.num = node_size++;
                node_temp.prev_node = prev_node.num;
                node_temp.t = prev_node.t + dt;
                node_temp.x = prev_node.x + v*cos(prev_node.angle + angle)*dt;
                node_temp.y = prev_node.y + v*sin(prev_node.angle + angle)*dt;
                node_temp.angle = prev_node.angle + angle;
                node_temp.angle_vel = angle_vel;
                node_temp.cost = prev_node.cost + cost + huristic_function(node,goalNode);
                //オープンノードリストに追加
                openNode.push_back(node_temp);
            }
            //
            //親ノード(この関数の引数のノード)をクローズリストに移動
            closedNode.push_back(openNode[node.num]);
            //openNodeから移動したノードを削除
            openNode.erase(openNode.begin());
        }
        bool check_add_node(cost_node& add_node){
            //条件
            return ( add_node.t + dt <= time_range ? true : false);
        }
        //距離ヒストグラムの作成
        void create_histgram_dis(float past_time){
            //経過時間past_timeを考慮
            //クラスタ変数をどこで管理するかが未定
            //
            for(int k =0; k < clstr.data.size(); k++){//クラスタ数
                //各クラスタに含まれる点群を取得しヒストグラムを作成
                for(int m = 0; m < clstr.data[k].pt.size(); m++){
                    float point_x = clstr.data[k].pt[m].x + clstr.twist[k].linear.x * past_time;
                    float point_y = clstr.data[k].pt[m].y + clstr.twist[k].linear.y * past_time;
                    float angleTemp = atan2(point_y, point_x)*180/M_PI;
                    float disTemp = sqrt(pow(point_x,2.0) + pow(point_y,2.0));
                    add_histgram_dis(angleTemp, disTemp);
                }
            }
        }
        //--予測コスト
        double huristic_function(cost_node& node, cost_node& goal_node){
            //引数ノードからゴールノードまでの距離をヒューリスティック関数
            double dis = sqrt( pow(node.x - goal_node.x, 2.0) + pow(node.y - goal_node.y, 2.0);
            return (dis);
        }
        //クローズノードとオープンノードを統合し, ノード番号でソートする
        //親ノードを参照するときに, いちいち探索をするのではなく, cobine_node[pre_node]で
        //参照すれば良くなる
        void cobine_open_close_node(){
            //code
            //openNodeコピー
            conbineNode.resize(openNode.size());
            std::copy(openNode.begin(), openNode.end(), conbineNode.begin());
            //closedNode挿入
            conbineNode.reserve(openNode.size() + closedNode.size()); //キャパ確保
            std::copy(b.begin(), b.end(), std::back_inserter(a));
        }
        //最小コストノードから目標位置と角度を見つける
        //選択可能条件(旋回可能上限角度)を満たしてるノードまで参照していく
        //角度条件を満たすノードを出力
        //cobine_open_close_nodeを行った後に実行
        int search_node_n(cost_node& min_node){
            //conbineNodeからノードをさかのぼり, 
            //最終的な目標位置, 角度を取得
            int nodeNum = min_node.num;
            int preNodeNum = min_node.pre_node;
            int selectNodeNum;
            while(nodeNum != 0){
                //親ノードを取得
                // ROS_INFO("node[%d] --> node[%d]: (%f,%f,%f)-->(%f,%f,%f)",
                //     nodeNum,preNodeNum,
                //     conbineNode[nodeNum].dx, conbineNode[nodeNum].dy, conbineNode[nodeNum].angle,
                //     conbineNode[preNodeNum].dx, conbineNode[preNodeNum].dy, conbineNode[preNodeNum].angle
                // );
                node_cost nodeRef = conbineNode[nodeNum];
                nodeNum = conbineNode[preNodeNum].num;
                preNodeNum = conbineNode[preNodeNum].pre_node;
                //条件判断
                if(nodeRef.angle <= angleThresholdMax && nodeRef.angle >= angleThresholdMin){
                    selectNodeNum = nodeRef.num;
                    //探索終了
                    return selectNodeNum;
                }
            }
            //探索不能
            return -1;
        }

}
