#include "Tracer.h" 
#include <Motor.h> 
using namespace ev3api;
#include <math.h>

Tracer::Tracer():
  leftWheel(PORT_C), rightWheel(PORT_B), colorSensor(PORT_3) { // <2>
  }

void Tracer::init() {
  init_f("Tracer");
}
//書き換えた。
void Tracer::terminate() {
  msg_f("Stopped.", 1);
  leftWheel.stop();  
  rightWheel.stop();
}

float Tracer::calc_porp_value(){
  const float Kp = 0.83;
  const int bias = 0;
  int diff = colorSensor.getBrightness() - target;
  return (Kp * diff + bias);
}

float Tracer::derivative_control(){
  int diff = colorSensor.getBrightness() - target;
  return (kd * (diff - prev_diff));
  prev_diff = diff;
}

void Tracer::direction(){
  Motor motorA = Motor(PORT_B, true, LARGE_MOTOR);
  Motor motorB = Motor(PORT_C, true, LARGE_MOTOR);
  right_counts = motorA.getCount();
  left_counts = motorB.getCount();
}

//ホイールの回転数から距離を計算
float motor_count_to_dist(int32_t c)
{
  return ( c * (0.381f* 0.94f));
}

float get_direction_change(int32_t rm,int32_t lm)
{
  int32_t motor_difference = rm - lm;
  float distance = motor_count_to_dist(motor_difference);
  //タイヤの間の距離が半径
  //角度を返す（ラジアン)
  return distance / 60.8906092;//仮 タイヤの間の距離mm//45.78;
}


//座標を求める
void Tracer::get_coordinates(int32_t cl, int32_t cr)
{
  int32_t diff_cl = cl - bef_cl; //前～今のカウント
  int32_t diff_cr = cr - bef_cr;

  int32_t diff_cl0 = cl - cl0; //基準点～今のカウント
  int32_t diff_cr0 = cr - cr0;

  float new_angle = get_direction_change(diff_cr0, diff_cl0)+d0;//基準点からどれだけ進んだか
  float length = (motor_count_to_dist(diff_cl) + motor_count_to_dist(diff_cr)) / 2;//前回からどれだけ進んだか
  float ave_angle = (new_angle+now_angle)/2;
  x = x + length*cos(ave_angle);//
  y = y + length*sin(ave_angle);//

  //syslog(7, "基準点～今のカウント");
  //sprintf(s, "%ld %ld", diff_cr0, diff_cl0); //基準点～今のカウント）
  //syslog(7, s);
  //syslog(7, "今の角度,基準点から図った今の角度,基準点までの角度");
  //sprintf(s, "%lf %lf %lf", new_angle , get_direction_change(diff_cr0, diff_cl0) , d0);//今の角度,基準点から図った今の角度,基準点までの角度
  //syslog(7, s);
  //sprintf(s, "%f %f %f", ave_angle,cosf(ave_angle), sinf(ave_angle)); //今の角度,基準点から図った今の角度,基準点までの角度
  //syslog(7, s);

  if (halfway_point==1&& x>=410.5){ //第1エリア中間地点
    syslog(7, "第1エリア中間地点に入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl, cr); //今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl, bef_cr); //前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl, diff_cr);//前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0, cr0); //基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0, diff_cr0); //基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle, get_direction_change(diff_cr0, diff_cl0), d0);//今の角度、基準点から図った今の角度、基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf", length);//長さ
    syslog(7, s);
    sprintf(s, "%lf",ave_angle); //角度
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);//座標
    syslog(7, s); 
  }
  if (area==1 && x>=821){  //第2エリア
    syslog(7, "第2エリアに入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl, cr); //今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl, bef_cr); //前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl, diff_cr);//前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0, cr0); //基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0, diff_cr0); //基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle, get_direction_change(diff_cr0, diff_cl0), d0);//今の角度、基準点から図った今の角度、基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf", length);//長さ
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);//座標
    syslog(7, s);
  }
  if (area==2 && y>=187){//第3エリア
    syslog(7, "第3エリアに入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl, cr); //今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl, bef_cr); //前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl, diff_cr);//前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0, cr0); //基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0, diff_cr0); //基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle, get_direction_change(diff_cr0, diff_cl0), d0);//今の角度、基準点から図った今の角度、基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf", length);//長さ
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);//座標
    syslog(7, s);
  }
  if (halfway_point == 2 && y>= 376.755){ //第3エリア中間地点y376.755
    syslog(7, "第3エリア中間地点に入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl, cr); //今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl, bef_cr); //前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl, diff_cr);//前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0, cr0); //基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0, diff_cr0); //基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle, get_direction_change(diff_cr0, diff_cl0), d0);//今の角度、基準点から図った今の角度、基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf", length);//長さ
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);//座標
    syslog(7, s);

  }
  if (area==3 && y>=500){ //566 ){//第４エリア
    syslog(7, "第4に入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl, cr); //今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl, bef_cr); //前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl, diff_cr);//前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0, cr0); //基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0, diff_cr0); //基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle, get_direction_change(diff_cr0, diff_cl0), d0);//今の角度、基準点から図った今の角度、基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf", length);//長さ
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);//座標
    syslog(7, s);
  }
  if (area==4 && x<=800){//854){//第5エリア
    syslog(7, "第5エリアに入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl , cr);//今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl , bef_cr);//前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl , diff_cr);//前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0 , cr0);//基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0 , diff_cr0);//基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle , get_direction_change(diff_cr0, diff_cl0) , d0);//今の角度,基準点から図った今の角度,基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf",length);//長さ
    syslog(7, s);
    sprintf(s, "%lf %lf", x , y);//座標
    syslog(7, s);

  }
  //if (area == 5 ){ //854){//第5エリア
  //  sprintf(s, "%f %f %f", ave_angle,cos(ave_angle), sin(ave_angle)); //今の角度,基準点から図った今の角度,基準点までの角度
  //  syslog(7, s);
  //}
  if (halfway_point==3 && x<=427.425){ //第5エリア中間地点x427.425
    syslog(7, "第5エリア中間地点に入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl, cr); //今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl, bef_cr); //前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl, diff_cr);//前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0, cr0); //基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0, diff_cr0); //基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle, get_direction_change(diff_cr0, diff_cl0), d0);//今の角度、基準点から図った今の角度、基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf", length);//長さ
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);//座標
    syslog(7, s);
  }
  if (area==5 && x<=0){//x<=200
    syslog(7, "第６エリアに入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl, cr); //今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl, bef_cr); //前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl, diff_cr); //前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0, cr0); //基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0, diff_cr0); //基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle, get_direction_change(diff_cr0, diff_cl0), d0); //今の角度、基準点から図った今の角度、基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf", length); //長さ
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y); //座標
    syslog(7, s);
  }
  if (area == 6 && y >= 818.09)
  { //第７エリア
    syslog(7, "第7エリアに入りましたget_coordinates");
    sprintf(s, "%ld %ld", cl, cr); //今のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", bef_cl, bef_cr); //前のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl, diff_cr); //前～今のカウント）
    syslog(7, s);
    sprintf(s, "%ld %ld", cl0, cr0); //基準点のカウント
    syslog(7, s);
    sprintf(s, "%ld %ld", diff_cl0, diff_cr0); //基準点～今のカウント）
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", new_angle, get_direction_change(diff_cr0, diff_cl0), d0); //今の角度、基準点から図った今の角度、基準点までの角度
    syslog(7, s);
    sprintf(s, "%lf", length); //長さ
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y); //座標
    syslog(7, s);
  }

  now_angle = new_angle;
  bef_cl = cl;
  bef_cr = cr;
}


void Tracer::run() {
  direction();
  msg_f("running...", 1);
  get_coordinates(left_counts,right_counts);
  if (halfway_point==1 && x>=410.5){ //第１エリア中間地点x410.5
    halfway_point=2; 

    v_x = x - x0;
    v_y = y - y0;
    v_length = sqrt((v_x*v_x)+(v_y*v_y));

    syslog(7, "第１エリア中間地点");
    sprintf(s, "%ld %ld", left_counts, right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", v_x,x,x0);
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", v_y,y,y0);
    syslog(7, s);
    sprintf(s, "%lf", v_length);
    syslog(7, s);
    x = x0 + v_length;
    y = y0;
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);

    cl0 = left_counts;
    cr0 = right_counts;
  }
  if (area==1 && x>=821){//第2エリア
    area=2;

    syslog(7, "第2エリアに入りました");
    sprintf(s, "%ld %ld",left_counts, right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
  }
  if (area==2 && y>=187){//第3エリア
    area=3;
    x0 = x;
    y0 = y;
    cl0 = left_counts;
    cr0 = right_counts;
    d0 = 3.141592f / 2; //90
    //now_angle = 3.141592f / 2;

    syslog(7, "第3エリアに入りました");
    sprintf(s, "%ld %ld", left_counts, right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
  }
  if (halfway_point == 2 && y>= 376.755){ //第3エリア中間地点y376.755
    halfway_point=3;
    v_x = x - x0;
    v_y = y - y0;
    v_length = sqrt((v_x*v_x)+(v_y*v_y));

    syslog(7, "第3エリア中間地点");
    sprintf(s, "%ld %ld", left_counts, right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", v_x, x, x0);
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", v_y, y, y0);
    syslog(7, s);
    sprintf(s, "%lf", v_length);
    syslog(7, s);

    x = x0;
    y = y0+v_length;
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);

    d0= 3.141592f/2; //90
    //now_angle = 3.141592f / 2;
    cl0 = left_counts;
    cr0 = right_counts;
   }
  if (area==3 && y>=500){ //566 ){
    area=4;

    syslog(7, "第4エリアに入りました");
    sprintf(s, "%ld %ld",left_counts, right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
  }
  if (area==4 && x<=800){//854){
    area=5;
    x0 = x;
    y0 = y;
    cl0 = left_counts;
    cr0 = right_counts;
    d0 = 3.141592f;
    //now_angle = 3.141592f;
    syslog(7, "第5エリアに入りました");
    sprintf(s, "%ld %ld", left_counts, right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
  }
  if (halfway_point==3 && x<=427.425){ //第5エリア中間地点x427.425
    halfway_point=4;
    v_x = x - x0;
    v_y = y - y0;
    v_length = sqrt((v_x * v_x) + (v_y * v_y));

    syslog(7, "第5エリア中間地点");
    sprintf(s, "%ld %ld", left_counts, right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", v_x, x, x0);
    syslog(7, s);
    sprintf(s, "%lf %lf %lf", v_y, y, y0);
    syslog(7, s);
    sprintf(s, "%lf", v_length);
    syslog(7, s);

    x = x0 - v_length;
    y = y0;
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
    d0 = 3.141592f;
    //now_angle = 3.141592f;
    cl0 = left_counts;
    cr0 = right_counts;
   }
   //if (area==5 ){
    //sprintf(s, "%lf %lf", x_coordinates, y_coordinates);
    //syslog(7, s);
   //}
  if (area==5 && x<=0){//200
    area=6;

    syslog(7, "第6エリアに入りました");
    sprintf(s, "%ld %ld",left_counts,right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
    pwm=pwm/3;
  }
  if (area == 6 && y >= 818.09){ //第７エリア
    area = 7;

    syslog(7, "第7エリアに入りました");
    sprintf(s, "%ld %ld", left_counts, right_counts);
    syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
  }
  //syslog(7, "カウント");
  //sprintf(s, "%ld %ld", left_counts, right_counts);
  //syslog(7, s);
  //syslog(7, "座標");
  //sprintf(s, "%lf %lf", x, y);
  //syslog(7, s);
  msg_f("running...", 1);

  float turn = calc_porp_value();
  int pwm_l = pwm - turn;
  int pwm_r = pwm + turn;


  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);
}