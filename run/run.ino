//--------------------------------------
double p_gain=1.0; //Pゲイン
double i_gain=0.001; //Iゲイン（多すぎると暴走します）
double d_gain=0.5; //Dゲイン
double target_value=100; //目標の出力値（目標値）
double control_value=0.0; //制御量（モータなどへの入力量）

double sensor(void); //センサからの入力を取得する関数
double motor(double); //モータへ値を代入する関数

//割り込みタイマー処理(H8ならITUなど。一定周期でループする。)
void interrupt_service(void)
{
  double current_value; //現在の出力値
  static double last_value=0.0; //一つ前の出力値（要保存のためstatic）
  double error,d_error; //偏差、偏差の微小変化量
  static double i_error=0.0; //偏差の総和（要保存のためstatic）

  current_value=sensor(); //センサ等から値を読み取る

  error=target_value-currnet_value; //偏差の計算
  d_error=current_value-last_value; //偏差微小変化量の計算

  control_value=p_gain*error + i_gain*i_error + d_gain*d_error;
 //制御量の計算
  motor(control_value); //モータへ値を代入

  last_value=current_value; //一つ前の出力値を更新
  i_error+=error; //偏差の総和を更新
}
