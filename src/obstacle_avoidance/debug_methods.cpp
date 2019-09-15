#include<local_navigation/obstacleAvoidance.h>

//デバッグ方法選択メソッド
void obstacleAvoidance::debug(){
    switch(debugType){
        case 1: debugMethod1();break;
        default: ;
    }
}

//計測した速度データをポイントクラウドで可視化
// 1,2,3,,,秒後の点群データを表示し、障害物の速度ベクトルを表示
// rviz上での矢印、速度の文字表示を行いたいが、調べる時間が惜しいため
// 現在は未実装
void obstacleAvoidance::debugMethod1(){
    
    // デバッグ処理

}
