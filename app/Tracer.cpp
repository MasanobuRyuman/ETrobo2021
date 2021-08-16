#include "Tracer.h" 
#include <Motor.h> 
#include "Clock.h"
using namespace ev3api;  

Tracer::Tracer():
  leftWheel(PORT_C), rightWheel(PORT_B), colorSensor(PORT_3) { 
  for(int i=0; i<20; i++)
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
  const int bias = 0;
  int diff;
  if (line_status_blue == true){
    syslog(7,"blueline");
    diff = colorSensor.getBrightness() - blue_target;
    return (kp * diff + bias);
  }else if (line_status_green == true){
    syslog(7,"greenline");
    diff = colorSensor.getBrightness() - green_target;
    return (green_kp * diff + bias);
  }else{
    syslog(7,"通常のpid制御");
    diff = colorSensor.getBrightness() - target;
    return (kp * diff + bias);
  }
}

//I制御の簡単な実装。
float Tracer::IntegralControl(){
  int LIGHT_LOG_SIZE = 20;
  int light_log[20];
  int light_integra;
  int diff;

  light_log[light_log_index] = diff;
	light_log_index = (light_log_index+1) % LIGHT_LOG_SIZE;
  light_integra = 0;
	for(int i=0;i<LIGHT_LOG_SIZE;i++){
		light_integra += light_log[i];
    //char s[256];
    //sprintf(s, "%d", light_log[i]);
    //syslog(7, s);
	}
  if (line_status_blue == true){
    return (ki * (light_integra / LIGHT_LOG_SIZE));
  }else if (line_status_green = true){
    return (green_ki * (light_integra / LIGHT_LOG_SIZE));
  }else{
    return (ki * (light_integra / LIGHT_LOG_SIZE));
  }

}
//I制御の簡単な実装。
float Tracer::IntegralControl(){
  int LIGHT_LOG_SIZE = 20;
  //int light_log[20];
  int light_integra;
  if (line_status_blue == true){
    diff = colorSensor.getBrightness() - blue_target;
  }else if (line_status_green = true){
    diff = colorSensor.getBrightness() - green_target;
  }else{
    diff = colorSensor.getBrightness() - target;
  }
  light_log[light_log_index] = diff;
	light_log_index = (light_log_index+1) % LIGHT_LOG_SIZE;
  light_integra = 0;
	for(int i=0;i<LIGHT_LOG_SIZE;i++){
		light_integra += light_log[i];
    //char s[256];
    //sprintf(s, "%d", light_integra);
    //syslog(7, s);
	}
  if (line_status_blue == true){
    return (ki * (light_integra / LIGHT_LOG_SIZE));
  }else if (line_status_green = true){
    return (green_ki * (light_integra / LIGHT_LOG_SIZE));
  }else{
    return (ki * (light_integra / LIGHT_LOG_SIZE));
  }
}

//D制御
float Tracer::derivative_control(){
  int diff;
  if (line_status_blue == true){
    diff = colorSensor.getBrightness() - blue_target;
    return (kd * (diff - prev_diff));
  }else if (line_status_green = true){
    diff = colorSensor.getBrightness() - green_target;
    return (green_kd * (diff - prev_diff));
  }else{
    diff = colorSensor.getBrightness() - target;
    return (kd * (diff - prev_diff));
  }
  
  prev_diff = diff;
}

void Tracer::direction(){
  Motor motorA = Motor(PORT_B, true, LARGE_MOTOR);
  Motor motorB = Motor(PORT_C, true, LARGE_MOTOR);
  left_counts = motorA.getCount();
  right_counts = motorB.getCount();
}

//ホイールの回転数から距離を計算
float motor_count_to_dist(int c)
{
  return ( c * (0.381f * 0.94f));
}

float get_direction_change(int rm,int lm)
{
  int motor_difference = rm - lm;
  float distance = motor_count_to_dist(motor_difference);
  //タイヤの間の距離が半径
  //角度を返す（ラジアン)
  return distance / 100;//仮 タイヤの間の距離mm
}


void Tracer::color_sensor(){
  colorSensor.getRawColor(rgb);
}

void Tracer::swing_neck(){
  if (swing_time_start == true){
    swing_time_start = false;
    clock.reset();
  }
  syslog(7,"swing");
  /*
  char r[256];
  sprintf(r,"%d",clock.now());
  syslog(7,r);
  */
  if ((clock.now() <= 300000) & (swing_key == true)){
    syslog(7,"kita");
    leftWheel.setPWM(30);
    rightWheel.setPWM(0);
    now_brightness = colorSensor.getBrightness();
    if (fast_comparison_brightness == true){
      before_brightness = now_brightness - 1;
    }
    if (before_brightness < now_brightness){
      getting_brighter = true;
    }else {
      getting_brighter = false;
    }
    
    if ((getting_brighter == true) & (now_brightness >= 17)){
      syslog(7,"スィング中止");
      swing_key = false;
    }
    
  }else{
    syslog(7,"スイングおわっった");
    swing_start = false;
    yellow_district_after = true;
  }
}

void Tracer::run() {
  direction();
  color_sensor();
  
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
  /*
  char b[256];
  sprintf(b,"%d",colorSensor.getBrightness());
  syslog(7,b);
  */
  

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
  //青色に入った
  if ((b_r_difference > 1.5) & (b_g_difference > 1.5)){
    syslog(7,"青色に入った");
    line_status_blue = true;
    float turn = calc_porp_value()+derivative_control()+IntegralControl();
    int pwm_l = pwm - turn;
    int pwm_r = pwm + turn;
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
  //黄色に入った
  } else if ((r_b_difference > 0.8) & (g_b_difference > 0.8)){
    syslog(7,"黄色に入った");
    if (line_status_yellow == false){
      syslog(7,"きた");
      yellow_count += 1;//新しく黄色い線に入ったらカウントを増やす
    }
    line_status_yellow = true;
    
    float turn = calc_porp_value()+derivative_control() + IntegralControl();
    int pwm_l = pwm + turn;
    int pwm_r = pwm + turn;
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
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
    float turn = calc_porp_value() + derivative_control() + IntegralControl();
    int pwm_l = pwm + turn;
    int pwm_r = pwm + turn;
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
    clock.reset();
  } else if (red_flag == true){
    float turn = calc_porp_value() + derivative_control() + IntegralControl();
    int pwm_l = pwm + turn;
    int pwm_r = pwm + turn;
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
    if (clock.now() > 1400000){
      terminate();
    } 
  } else {
    line_status_yellow = false;//黄色い線に入っていない
    line_status_blue = false;//青の線に入っていない
    line_status_green = false;//緑のせんに入っていない
    if ((yellow_count >= 2) & (swing_start == true)){
      syslog(7,"スィング開始");
      swing_neck();
    } else if (yellow_count == 1){
      float turn = calc_porp_value() + derivative_control() + IntegralControl();
      int pwm_l = pwm + turn;
      int pwm_r = pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
    } else if (yellow_district_after == true){
      if (fast_yellow_district_after == true){
        clock.reset();
        fast_yellow_district_after = false;
      }
      if (clock.now() > 1500000){ //黄色の丸を超えた後にゆっくり走り出す
        //syslog(7,"ゆっくり走り出す");
        float turn = calc_porp_value() + derivative_control() + IntegralControl();
        int pwm_l = yellow_district_after_pwm - turn;
        int pwm_r = yellow_district_after_pwm + turn;
        leftWheel.setPWM(pwm_l);
        rightWheel.setPWM(pwm_r);
      }else{
        float turn = calc_porp_value() + derivative_control() + IntegralControl();
        int pwm_l = pwm - turn;
        int pwm_r = pwm + turn;
        leftWheel.setPWM(pwm_l);
        rightWheel.setPWM(pwm_r);
      }
      
    } else {
      syslog(7,"通常モード");
      line_status_blue = false;
      float turn = calc_porp_value() + derivative_control() + IntegralControl();
      int pwm_l = pwm - turn;
      int pwm_r = pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
    }
  }
}



