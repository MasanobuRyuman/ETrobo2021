#include "Tracer.h" 
#include <Motor.h> 
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

float Tracer::calc_porp_value(){
  const float Kp = 0.83;
  const int bias = 0;
  int diff = colorSensor.getBrightness() - target;
  return (Kp * diff + bias);
}

//I制御の簡単な実装。
float IntegralControl(){
  int i;
  int LIGHT_LOG_SIZE = 20;
  int light_log[LIGHT_LOG_SIZE], light_log_index, light_integra;
  int i_val,Ki;

  light_integra = 0;
	for(i=0;i<LIGHT_LOG_SIZE;i++){
		light_integra += light_log[i];
	}
	i_val = Ki * light_integra / LIGHT_LOG_SIZE;	
}

float Tracer::derivative_control(){
  int diff = colorSensor.getBrightness() - target;
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

//I制御の簡単な実装。
float Tracer::IntegralControl(){
  int i;
  int LIGHT_LOG_SIZE = 20;
  int light_log[LIGHT_LOG_SIZE],light_log_index = 0,light_integra;
  const int target = 10;
  
  float diff = colorSensor.getBrightness() - target;
  light_log[light_log_index] = diff;
	light_log_index = (light_log_index+1) % LIGHT_LOG_SIZE;
  light_integra = 0;
	for(i=0;i<LIGHT_LOG_SIZE;i++){
		light_integra += light_log[i];
	}	
  return (ki * light_integra / LIGHT_LOG_SIZE);
}

void Tracer::run() {
  direction();
  msg_f("running...", 1);
  float turn = calc_porp_value();
  int pwm_l = pwm + turn;
  int pwm_r = pwm - turn;
  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);
}


