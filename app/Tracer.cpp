#include "Tracer.h" 
#include <Motor.h> 
#include "Clock.h"
using namespace ev3api;  

Tracer::Tracer():
  leftWheel(PORT_C), rightWheel(PORT_B), colorSensor(PORT_3) { // <2>
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
  if (line_status == true){
    diff = colorSensor.getBrightness() - blue_target;
  }else{
    diff = colorSensor.getBrightness() - target;
  }
  
  return (kp * diff + bias);
}

//I制御の簡単な実装。
float Tracer::IntegralControl(){
  int LIGHT_LOG_SIZE = 20;
  int light_log[20];
  int light_integra;
  int diff;

  if (line_status == true){
    diff = colorSensor.getBrightness() - blue_target;
  }else{
    diff = colorSensor.getBrightness() - target;
  }
  light_log[light_log_index] = diff;
	light_log_index = (light_log_index+1) % LIGHT_LOG_SIZE;
  light_integra = 0;
	for(int i=0;i<LIGHT_LOG_SIZE;i++){
		light_integra += light_log[i];
    //char s[256];
    //sprintf(s, "%d", light_log[i]);
    //syslog(7, s);
	}
  return (ki * (light_integra / LIGHT_LOG_SIZE));
}

//D制御
float Tracer::derivative_control(){
  int diff;
  if (line_status == true){
    diff = colorSensor.getBrightness() - blue_target;
  }else{
    diff = colorSensor.getBrightness() - target;
  }
  return (kd * (diff - prev_diff));
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
  if (clock.now() < 3000000){
    syslog(7,"kita");
    leftWheel.setPWM(0);
    rightWheel.setPWM(0);
  }else if(clock.now() >= 3000000){
    leftWheel.setPWM(pwm);
    rightWheel.setPWM(pwm);
  }
}

void Tracer::run() {
  direction();
  color_sensor();

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
  if ((b_r_difference > 1.5) & (b_g_difference > 1.5)){
    syslog(7,"青色に入った");
    line_status = true;
    float turn = calc_porp_value()+derivative_control()+IntegralControl();
    int pwm_l = pwm - turn;
    int pwm_r = pwm + turn;
    leftWheel.setPWM(pwm_l);
     rightWheel.setPWM(pwm_r);
  } else if ((r_b_difference > 1.5) & (g_b_difference > 1.5)){
    syslog(7,"黄色に入った");
    if (yellow_in_flag == false){
      syslog(7,"きた");
      yellow_count += 1;
    }
    yellow_in_flag = true;
    
    float turn = calc_porp_value()+derivative_control() + IntegralControl();
    int pwm_l = pwm + turn;
    int pwm_r = pwm + turn;
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
  }else{
    yellow_in_flag = false;
    if ((yellow_count == 2) & (swing_start == true)){
      swing_neck();
    }else{
      line_status = false;
      float turn = calc_porp_value() + derivative_control() + IntegralControl();
      int pwm_l = pwm - turn;
      int pwm_r = pwm + turn;
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
    }
  }
  
}



