#include "Motor.h"       
#include "ColorSensor.h" 
#include "util.h"  
#include "Clock.h"

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
  Clock clock; 
  const int8_t mThreshold = 28;  
  //最高速度
  const int8_t maxPwm = (Motor::PWM_MAX) / 1;
  //直線の最高速度
  const int8_t straightMaxPwm = (Motor::PWM_MAX) / 1;
  const int8_t pwm = (Motor::PWM_MAX) / 2;
  int tracerStatus = 0;
  float body_direction;
  float calc_porp_value();
  float IntegralControl();
  float derivative_control();
  void direction();
  void swing_neck();
  int32_t left_counts;
  int32_t right_counts;
  const int target = 19;
  //青い線を通る時の基準値
  const int blue_target = 26;
  int prev_diff = 0;
  const float kp = 0.83;
  //pwmの値がmax / 1　の時
  const float maxkp = 1.0;
  const float kd = 0;
  const float ki = 0;
  int light_log_index = 0;
  void color_sensor();
  rgb_raw_t rgb;
  bool line_status=false;
  int yellow_count = 0;
  int map_status=0;
  bool swing = true;
  bool yellow_in_flag=false;
  bool swing_start = true;
  bool swing_time_start = true;
};
