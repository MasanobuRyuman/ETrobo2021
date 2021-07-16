#include "Tracer.h" 

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

void Tracer::run() {
  msg_f("running...", 1);
  float turn = calc_porp_value();
  int pwm_l = pwm + turn;
  int pwm_r = pwm - turn;
  if (tracerStatus == 0){
    leftWheel.setPWM(10);
    rightWheel.setPWM(10);
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


