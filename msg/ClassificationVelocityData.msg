#classfied data with velocity
#
#ROSヘッダ
std_msgs/Header header
#マップサイズ
#実測サイズ
std_msgs/Float32 width#
std_msgs/Float32 height#
std_msgs/Float32 res#
#ピクセルサイズ
std_msgs/Int32 widthInt
std_msgs/Int32 heightInt
#マップの中心点（センサ座標）
geometry_msgs/Point cp
#クラスタデータ
std_msgs/Int32 size #クラスタ数
local_navigation/ClassificationElement[] data
geometry_msgs/Twist[] twist #測定速度データ
int32[] trackingNum #追跡回数
int32[] match #currentCluster->previousClusterの参照
#dt
float32 dt