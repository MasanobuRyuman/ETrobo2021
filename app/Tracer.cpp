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
  int32_t left_counts = motorA.getCount();
  int32_t right_counts = motorB.getCount();
  body_direction = 2 / (left_counts - right_counts) * 3.14;
}


void Tracer::run() {
  msg_f("running...", 1);
  float turn = calc_porp_value();
  int pwm_l = pwm + turn;
  int pwm_r = pwm - turn;
  if (tracerStatus == 0){
    float turn = calc_porp_value();
    int pwm_l = straightMaxPwm - turn;
    int pwm_r = straightMaxPwm + turn;
    leftWheel.setPWM(pwm_l);
    rightWheel.setPWM(pwm_r);
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


