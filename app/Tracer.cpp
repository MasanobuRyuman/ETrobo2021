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

void Tracer::run() {
  direction();
  msg_f("running...", 1);
  float turn = calc_porp_value();
  int pwm_l = pwm + turn;
  int pwm_r = pwm - turn;
  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);
}


