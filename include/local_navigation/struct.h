//処理データ
struct crossPoint{
    float x;//交差位置x
    float y;//交差位置y
    float dis;//交差位置とロボットの距離
    float t;//交差時の時間
    int index;//障害物番号
    bool safe;//安全フラグ
};