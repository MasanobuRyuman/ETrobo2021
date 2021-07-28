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
  const int target = 10;
  const int bias = 0;

  int diff = colorSensor.getBrightness() - target;
  return (Kp * diff + bias);
}

void Tracer::direction(){
  Motor motorA = Motor(PORT_A, true, LARGE_MOTOR);
  Motor motorB = Motor(PORT_B, true, LARGE_MOTOR);
  left_counts = motorA.getCount();
  right_counts = motorB.getCount();
  body_direction = 10 / (left_counts - right_counts) * 3.14;
}

float get_direction_change(int rm,int lm)
{
  int motor_difference = rm - lm;
  /*
  float distance = motor_count_to_dist(motor_difference);
  //タイヤの間の距離が半径
  return 1 / distance;
  */
}

//ホイールの回転数から距離を計算
float motor_count_to_dist(int c)
{
  return ( c * 0.187f);//仮
}

void Tracer::run() {
  syslog(7,right_counts);
  msg_f("running...", 1);
  float turn = calc_porp_value();
  int pwm_l = pwm + turn;
  int pwm_r = pwm - turn;
  if (tracerStatus == 0){
    leftWheel.setPWM(pwm);
    rightWheel.setPWM(pwm);

  }
  if (tracerStatus == 0){
    float turn = calc_porp_value();
    int pwm_l = straightMaxPwm - turn;
    int pwm_r = straightMaxPwm + turn;
    //角度に応じて調整をしている。
    if (body_direction >= 10){
      if (right_counts >= 10){
        terminate();
      }
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r+20);
    } else if (body_direction <= 10){
      leftWheel.setPWM(pwm_l+20);
      rightWheel.setPWM(pwm_r);
    }else{
      leftWheel.setPWM(pwm_l);
      rightWheel.setPWM(pwm_r);
    }
  }else if (tracerStatus == 1){
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
  }else if (tracerStatus == 2){
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
  }else if (tracerStatus == 3){
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
  }
 
  
}


