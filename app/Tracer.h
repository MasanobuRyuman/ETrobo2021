#include "Motor.h"       
#include "ColorSensor.h" 
#include "util.h"        

using namespace ev3api;  

class Tracer {  
public:
  Tracer();
  void run();       
  void init();
  void terminate();
  void leftTracer();
  void rightTracer();

private:
  Motor leftWheel;
  Motor rightWheel;
  ColorSensor colorSensor; 
  const int8_t mThreshold = 28;  
  //最高速度
  const int8_t maxPwm = (Motor::PWM_MAX) / 1;
  //直線の最高速度
  const int8_t straightMaxPwm = (Motor::PWM_MAX) / 1;
  const int8_t pwm = (Motor::PWM_MAX) / 2;
  int tracerStatus = 0;
  float body_direction;
  float calc_porp_value();
  float derivative_control();
  void direction();
  int32_t left_counts;
  int32_t right_counts;

  float x_coordinates=0; //x座標
  float y_coordinates=0; //y座標
  int32_t before_left_counts=0;
  int32_t before_right_counts=0;
  float now_angle;
  void get_coordinates(int32_t now_left_counts, int32_t now_right_counts);

  const int target = 10;
  static float prev_diff = 0.0f;
  const float kd = 10.0;
};
