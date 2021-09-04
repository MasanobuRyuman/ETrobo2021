#include "Tracer.h" 
#include <Motor.h>
#include <math.h>
#include "Clock.h"
using namespace ev3api;

Tracer::Tracer():
  leftWheel(PORT_C), rightWheel(PORT_B), colorSensor(PORT_3) { 
  for(int i=0; i<20; i++)//最初に呼び出しlight_logを初期化し中身を0にする。
    {
      light_log[i] = 0;
    }
    // <2>
  }

void Tracer::init() {
  init_f("Tracer");
}



void Tracer::terminate() {
  msg_f("Stopped.", 1);
  leftWheel.stop();  
  rightWheel.stop();
}

//p制御
float Tracer::calc_porp_value(){
  //syslog(7,"p制御");
  const int bias = 0;
  int diff;
  switch(area){
    case 1: 
      diff = colorSensor.getBrightness() - target;
      return (straight_road_kp * diff + bias);
      break;
    case 2:
      diff = colorSensor.getBrightness() - target;
      return (fast_curve_kp * diff + bias);
      break;
    case 3:
      diff = colorSensor.getBrightness() - target;
      return (straight_road_kp * diff + bias);
      break;
    case 4:
      diff = colorSensor.getBrightness() - target;
      return (second_curve_kp * diff + bias);
      break;
    case 5:
      syslog(7,"area5のp制御");
      diff = colorSensor.getBrightness() - target;
      return (area5_road_kp * diff + bias);
      break;
    case 6:
      diff = colorSensor.getBrightness() - target;
      return (third_curve_kp * diff + bias);
      break;
    case 7:
      diff = colorSensor.getBrightness() - target;
      return (straight_road_kp * diff + bias);
      break;
    case 8:
      if (line_status_blue == true){
        //syslog(7,"blueline");
        diff = colorSensor.getBrightness() - blue_target;
        return (kp * diff + bias);
      }else if (line_status_green == true){
        //syslog(7,"greenline");
        diff = colorSensor.getBrightness() - green_target;
        return (green_kp * diff + bias);
      }else if (last_caurce == true){
        //syslog(7,"最後の直線");
        diff = colorSensor.getBrightness() - last_target;
        return (green_kp * diff + bias);
      }else{
        //syslog(7,"通常のpid制御");
        diff = colorSensor.getBrightness() - target;
        return (kp * diff + bias);
      }
      break;
  }
  
}


//I制御の実装。
float Tracer::IntegralControl(){
  int LIGHT_LOG_SIZE = 20;//配列light_logの大きさを示す。
  int light_integra;//配列light_logの総和。これの平均値をi制御にて使用する。
  int diff;
  switch(area){
    case 1:
      diff = colorSensor.getBrightness() - target;
      break;
    case 2:
      diff = colorSensor.getBrightness() - target;
      break;
    case 3:
      diff = colorSensor.getBrightness() - target;
      break;
    case 4:
      diff = colorSensor.getBrightness() - target;
      break;
    case 5:
      diff = colorSensor.getBrightness() - target;
      break;
    case 6:
      diff = colorSensor.getBrightness() - target;
      break;
    case 7:
      diff = colorSensor.getBrightness() - target;
      break;
    case 8:
      if (line_status_blue == true){
        diff = colorSensor.getBrightness() - blue_target;
      }else if (line_status_green == true){
        diff = colorSensor.getBrightness() - green_target;
      }else if (last_caurce == true){
        diff = colorSensor.getBrightness() - last_target;
      }else{
        diff = colorSensor.getBrightness() - target;
      }
      break;
  }
  
  light_log[light_log_index] = diff;
	light_log_index = (light_log_index+1) % LIGHT_LOG_SIZE;
  light_integra = 0;
	for(int i=0;i<LIGHT_LOG_SIZE;i++){
		light_integra += light_log[i];
	}
  switch(area){
    case 1:
      return (straight_road_ki * (light_integra / LIGHT_LOG_SIZE));
      break;
    case 2:
      return (fast_curve_ki * (light_integra / LIGHT_LOG_SIZE));
      break;
    case 3:
      return (straight_road_ki * (light_integra / LIGHT_LOG_SIZE));
      break;
    case 4:
      return (second_curve_ki * (light_integra / LIGHT_LOG_SIZE));
      break;
    case 5:
      return (area5_road_ki * (light_integra / LIGHT_LOG_SIZE));
      break;
    case 6:
      return (third_curve_ki * (light_integra / LIGHT_LOG_SIZE));
      break;
    case 7:
      return (straight_road_ki * (light_integra / LIGHT_LOG_SIZE));
      break;
    case 8:
      if (line_status_blue == true){
        return (ki * (light_integra / LIGHT_LOG_SIZE));
      }else if (line_status_green == true){
        return (green_ki * (light_integra / LIGHT_LOG_SIZE));
      }else{
        return (ki * (light_integra / LIGHT_LOG_SIZE));
      }
      break;
  }
  
}

//D制御
float Tracer::derivative_control(){
  int diff;
  switch(area){
    case 1:
      diff = colorSensor.getBrightness() - target;
      return (straight_road_kd * (diff - prev_diff));
      break;
    case 2:
      diff = colorSensor.getBrightness() - target;
      return (fast_curve_kd * (diff - prev_diff));
      break;
    case 3:
      diff = colorSensor.getBrightness() - target;
      return (straight_road_kd * (diff - prev_diff));
      break;
    case 4:
      diff = colorSensor.getBrightness() - target;
      return (second_curve_kd * (diff - prev_diff));
      break;
    case 5:
      diff = colorSensor.getBrightness() - target;
      return (area5_road_kd * (diff - prev_diff));
      break;
    case 6:
      diff = colorSensor.getBrightness() - target;
      return (third_curve_kd * (diff - prev_diff));
      break;
    case 7:
      diff = colorSensor.getBrightness() - target;
      return (straight_road_kd * (diff - prev_diff));
      break;
    case 8:
      if (line_status_blue == true){
        diff = colorSensor.getBrightness() - blue_target;
        return (kd * (diff - prev_diff));
      }else if (line_status_green == true){
        diff = colorSensor.getBrightness() - green_target;
        return (green_kd * (diff - prev_diff));
      }else if (last_caurce == true){
        //syslog(7,"d制御が最後のコース");
        diff = colorSensor.getBrightness() - last_target;
        return (kd * (diff - prev_diff));
      }else{
        diff = colorSensor.getBrightness() - target;
        return (kd * (diff - prev_diff));
      }
      break;
  }
  
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



void Tracer::color_sensor()
{
  colorSensor.getRawColor(rgb);
}

//黄色の丸を超えたら首を振る
void Tracer::swing_neck()
{
  if (swing_time_start == true)
  {
    swing_time_start = false;
    clock.reset();
  }
  //syslog(7,"swing");
  /*
  char r[256];
  sprintf(r,"%d",clock.now());
  syslog(7,r);
  */
  if (swing_key == true)
  {
    //syslog(7,"kita");
    leftWheel.setPWM(30);
    rightWheel.setPWM(0);
    now_brightness = colorSensor.getBrightness();
    if (fast_comparison_brightness == true)
    {
      before_brightness = now_brightness - 1;
    }
    if (before_brightness < now_brightness)
    {
      getting_brighter = true;
    }
    else
    {
      getting_brighter = false;
    }

    if ((clock.now() >= 2000) & ((getting_brighter == true) & (now_brightness >= 17) & (rgb.r < 100) & (rgb.g < 100) & (rgb.b < 120)))
    {
      //syslog(7,"スィング中止");
      swing_key = false;
    }
  }
  else
  {
    //syslog(7,"スイングおわっった");
    swing_start = false;
    yellow_district_after = true;
  }
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

/*
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
  */

  now_angle = new_angle;
  bef_cl = cl;
  bef_cr = cr;
}


void Tracer::run() {
  direction();
  color_sensor();
  msg_f("running...", 1);
  //座標
  get_coordinates(left_counts,right_counts);
  if (halfway_point==1 && x>=410.5){ //第１エリア中間地点x410.5
    halfway_point=2; 

    v_x = x - x0;
    v_y = y - y0;
    v_length = sqrt((v_x*v_x)+(v_y*v_y));

    syslog(7, "第１エリア中間地点");
    
    //sprintf(s, "%ld %ld", left_counts, right_counts);
    //syslog(7, s);
    //sprintf(s, "%lf %lf %lf", v_x,x,x0);
    //syslog(7, s);
    //sprintf(s, "%lf %lf %lf", v_y,y,y0);
    //syslog(7, s);
    //sprintf(s, "%lf", v_length);
    //syslog(7, s);
    
    x = x0 + v_length;
    y = y0;
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);

    cl0 = left_counts;
    cr0 = right_counts;
  }
  if (area==1 && x>=815){//821){//第2エリア
    area=2;

    syslog(7, "第2エリアに入りました");
    //sprintf(s, "%ld %ld",left_counts, right_counts);
    //syslog(7, s);
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
    //sprintf(s, "%ld %ld", left_counts, right_counts);
    //syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
  }
  if (halfway_point == 2 && y>= 376.755){ //第3エリア中間地点y376.755
    halfway_point=3;
    v_x = x - x0;
    v_y = y - y0;
    v_length = sqrt((v_x*v_x)+(v_y*v_y));

    syslog(7, "第3エリア中間地点");
    //sprintf(s, "%ld %ld", left_counts, right_counts);
    //syslog(7, s);
    //sprintf(s, "%lf %lf %lf", v_x, x, x0);
    //syslog(7, s);
    //sprintf(s, "%lf %lf %lf", v_y, y, y0);
    //syslog(7, s);
    //sprintf(s, "%lf", v_length);
    //syslog(7, s);

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
    //sprintf(s, "%ld %ld",left_counts, right_counts);
    //syslog(7, s);
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
    //sprintf(s, "%ld %ld", left_counts, right_counts);
    //syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);
  }
  if (halfway_point==3 && x<=427.425){ //第5エリア中間地点x427.425
    halfway_point=4;
    v_x = x - x0;
    v_y = y - y0;
    v_length = sqrt((v_x * v_x) + (v_y * v_y));

    syslog(7, "第5エリア中間地点");
    //sprintf(s, "%ld %ld", left_counts, right_counts);
    //syslog(7, s);
    //sprintf(s, "%lf %lf %lf", v_x, x, x0);
    //syslog(7, s);
    //sprintf(s, "%lf %lf %lf", v_y, y, y0);
    //syslog(7, s);
    //sprintf(s, "%lf", v_length);
    //syslog(7, s);

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
    //sprintf(s, "%ld %ld",left_counts,right_counts);
    //syslog(7, s);
    sprintf(s, "%lf %lf", x, y);
    syslog(7, s);

  }
  if (area == 6 && y >= 805){//818.09){ //第７エリア
    area = 7;

    syslog(7, "第7エリアに入りました");
    //sprintf(s, "%ld %ld", left_counts, right_counts);
    //syslog(7, s);
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


  //真っ直ぐ走らせる
  /*
  float turn = calc_porp_value()+derivative_control() + IntegralControl();
  int pwm_l = pwm + turn;
  msg_f("running...", 1);
  float turn = calc_porp_value()+IntegralControl()+derivative_control();
  int pwm_l = pwm - turn;
  int pwm_r = pwm + turn;
  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);
  */

  //明るさ測定
  
  char b[256];
  sprintf(b,"%d",colorSensor.getBrightness());
  syslog(7,b);
  

  //rgbの測定
  /*
  char r[256];
  sprintf(r,"%d",rgb.r);
  syslog(7,"r");
  syslog(7,r);
  char g[256];
  sprintf(g,"%d",rgb.g);
  syslog(7,"g");
  syslog(7,g);
  char s[256];
  sprintf(s,"%d",rgb.b);
  syslog(7,"b");
  syslog(7,s);
  */
  msg_f("running...", 1);
  double b_r_difference = (double)rgb.b / (rgb.r + 1);
  double b_g_difference = (double)rgb.b / (rgb.g + 1);
  double r_b_difference = (double)rgb.r / (rgb.b + 1);
  double g_b_difference = (double)rgb.g / (rgb.b + 1);
  double g_r_difference = (double)rgb.g / (rgb.r + 1);
  double r_g_difference = (double)rgb.r / (rgb.g + 1);

  /*
  char r_dif[256];
  sprintf(r_dif,"%lf",b_r_difference);
  syslog(7,"r_dif");
  syslog(7,r_dif);
  char g_dif[256];
  sprintf(g_dif,"%lf",b_g_difference);
  syslog(7,"g_dif");
  syslog(7,g_dif);
  */

  /*
  char r_dif[256];
  sprintf(r_dif,"%lf",r_b_difference);
  syslog(7,"r_dif");
  syslog(7,r_dif);
  char g_dif[256];
  sprintf(g_dif,"%lf",g_b_difference);
  syslog(7,"g_dif");
  syslog(7,g_dif);
  */
  /*
  char brightness[256];
  sprintf(brightness,"%d",colorSensor.getBrightness());
  syslog(7,brightness);
  */

  //ここから
  //最後の直線（最後に先に色を判定されて動作が乱れるのを防ぐため一番最初にかく)
  if (run_fast == true){
    clock.reset();
    run_fast = false;
  }
  switch(area){
    case 1: 
      syslog(7,"area1");
      if (clock.now() <= 600000){
        turn = calc_porp_value() + IntegralControl() + derivative_control();
        pwm_l = fast_slow_pwm - turn;
        pwm_r = fast_slow_pwm + turn;
        leftWheel.setPWM(pwm_l);
        rightWheel.setPWM(pwm_r);
      }else{
        syslog(7,"ここから早くなる");
        turn = calc_porp_value() + IntegralControl() + derivative_control();
        pwm_l = straight_road_pwm - turn;
        pwm_r = straight_road_pwm + turn;
        leftWheel.setPWM(pwm_l);
        rightWheel.setPWM(pwm_r);
      }
      
      break;
    case 2:
      syslog(7,"area2");
      turn = calc_porp_value() + IntegralControl() + derivative_control();
      pwm_l = fast_curve_pwm - turn;
      pwm_r = fast_curve_pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
      break;
    case 3:
      turn = calc_porp_value() + IntegralControl() + derivative_control();
      pwm_l = straight_road_pwm - turn;
      pwm_r = straight_road_pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
      break;
    case 4:
      
      turn = calc_porp_value() + IntegralControl() + derivative_control();
      pwm_l = second_curve_pwm - turn;
      pwm_r = second_curve_pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
      break;
    case 5:
      syslog(7,"エリア5");
      turn = calc_porp_value() + IntegralControl() + derivative_control();
      pwm_l = area5_road_pwm - turn;
      pwm_r = area5_road_pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
    case 6:
      
      turn = calc_porp_value() + IntegralControl() + derivative_control();
      pwm_l = third_curve_pwm - turn;
      pwm_r = third_curve_pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
    case 7:
      
      turn = calc_porp_value() + IntegralControl() + derivative_control();
      pwm_l = straight_road_pwm - turn;
      pwm_r = straight_road_pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
    case 8:
      if (red_flag == true){
        syslog(7,"最後の直しん");
        leftWheel.setPWM(straight_pwm);
        rightWheel.setPWM(straight_pwm);
        if (clock.now() > 2800000){
          terminate();
        }
      //青色に入った
      } else if ((b_r_difference > 1.5) & (b_g_difference > 1.5)){
        syslog(7,"青色に入った");
        if (line_status_blue == false){
          blue_count += 1;
        }
        line_status_blue = true;
        float turn = calc_porp_value()+derivative_control()+IntegralControl();
        int pwm_l = difficulty_cource_pwm - turn;
        int pwm_r = difficulty_cource_pwm + turn;
        leftWheel.setPWM(pwm_l);
        rightWheel.setPWM(pwm_r);
      //黄色に入った
      } else if ((r_b_difference > 0.8) & (g_b_difference > 0.8)){
        syslog(7,"黄色に入った");
        if (line_status_yellow == false){
          //syslog(7,"きた");
          yellow_count += 1;//新しく黄色い線に入ったらカウントを増やす
        }
        line_status_yellow = true;
        leftWheel.setPWM(straight_pwm);
        rightWheel.setPWM(straight_pwm);
      //緑色に入った
      } else if ((g_r_difference > 1.0) & (g_b_difference > 1.0)){
        syslog(7,"緑に入った");
        line_status_green = true;
        //yellow_district_after = false;
        /*
        if (fast_green == true){
          clock.reset();
          fast_green = false;
        }
        */
        //最初は機体を傾ける
        /*if (clock.now() < 100000){
          leftWheel.setPWM(40);
          rightWheel.setPWM(0);
        }else{
          syslog(7,"傾きが終わった");
          float turn = calc_porp_value() + derivative_control() + IntegralControl();
          int pwm_l = green_pwm - turn;
          int pwm_r = green_pwm + turn;
          leftWheel.setPWM(pwm_l);
          rightWheel.setPWM(pwm_r);
        }
        */

        float turn = calc_porp_value() + derivative_control() + IntegralControl();
        int pwm_l = green_pwm - turn;
        int pwm_r = green_pwm + turn;
        leftWheel.setPWM(pwm_l);
        rightWheel.setPWM(pwm_r);

  
      //赤色に入った
      } else if ((r_g_difference > 1.2) & (r_b_difference > 1.2)){
        syslog(7,"赤色");
        red_flag = true;
        leftWheel.setPWM(straight_pwm);
        rightWheel.setPWM(straight_pwm);
        clock.reset();
      } else {
        syslog(7,"通常");
        if (line_status_green == true){
          last_caurce = true;
          syslog(7,"緑が終わった");
        }
        line_status_yellow = false;//黄色い線に入っていない
        line_status_blue = false;//青の線に入っていない
        line_status_green = false;//緑のせんに入っていない
        if ((yellow_count >= 2) & (swing_start == true)){
        syslog(7,"スィング開始");
        swing_neck();
      } else if (yellow_count == 1){
        leftWheel.setPWM(straight_pwm);
        rightWheel.setPWM(straight_pwm);
      } else if (yellow_district_after == true){
        if (fast_yellow_district_after == true){
          clock.reset();
          fast_yellow_district_after = false;
        }
        if (clock.now() > 1){ //黄色の丸を超えた後にゆっくり走り出す
          //syslog(7,"ゆっくり走り出す");
          float turn = calc_porp_value() + derivative_control() + IntegralControl();
          int pwm_l = yellow_district_after_pwm - turn;
          int pwm_r = yellow_district_after_pwm + turn;
          leftWheel.setPWM(pwm_l);
          rightWheel.setPWM(pwm_r);
        }else{
          float turn = calc_porp_value() + derivative_control() + IntegralControl();
          int pwm_l = difficulty_cource_pwm - turn;
          int pwm_r = difficulty_cource_pwm + turn;
          leftWheel.setPWM(pwm_l);
          rightWheel.setPWM(pwm_r);
        }
      
      } else {
        syslog(7,"通常モード");
        line_status_blue = false;
        line_status_green = false;
        line_status_yellow = false;
        float turn = calc_porp_value() + derivative_control() + IntegralControl();
        int pwm_l = difficulty_cource_pwm - turn;
        int pwm_r = difficulty_cource_pwm + turn;
        leftWheel.setPWM(pwm_l);
        rightWheel.setPWM(pwm_r);
      }
    }
  }

 /*
  float turn = calc_porp_value();
  int pwm_l = pwm - turn;
  int pwm_r = pwm + turn;
  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);
  */
}
