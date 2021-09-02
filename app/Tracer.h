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
  const int8_t maxPwm = (Motor::PWM_MAX) / 3;
  //直線の最高速度
  const int8_t straightMaxPwm = (Motor::PWM_MAX) / 3;
  int8_t pwm = (Motor::PWM_MAX) / 3;
  int tracerStatus = 0;
  float body_direction;
  float calc_porp_value();
  float derivative_control();
  void direction();
  int32_t left_counts;
  int32_t right_counts;

  float x=-17; //x座標
  float y=0; //y座標

  int32_t bef_cl = 0;//前回のカウント
  int32_t bef_cr=0;
  float now_angle;//今の向き
  void get_coordinates(int32_t cl, int32_t cr);

  const int target = 10;
  int prev_diff = 0;
  const float kd = 10.0f;
  char s[256];

  int area=1;
  int halfway_point=1;
  float v_x;
  float v_y;
  float v_length;

  float d0=0;//基準点角度
  float x0 = -17; //x座標
  float y0 = 0;   //y座標
  int32_t cl0=0; //基準点カウント左
  int32_t cr0=0; //基準点カウント右

};
