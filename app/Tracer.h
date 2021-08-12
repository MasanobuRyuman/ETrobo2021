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
  const int8_t green_pwm = (Motor::PWM_MAX) / 6;
  const int8_t yellow_district_after_pwm = (Motor::PWM_MAX) / 5;
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
  const int blue_target = 28;
  const int green_target = 26;
  int prev_diff = 0;
  const float kp = 0.83;
  const float green_kp = 1.0f;
  //pwmの値がmax / 1　の時
  const float maxkp = 1.0;
  const float kd = 0;
  const float green_kd = 2.0f;
  const float ki = 0;
  const float green_ki = 0;
  int light_log_index = 0;
  void color_sensor();
  rgb_raw_t rgb;
  bool line_status_blue = false;
  bool line_status_green = false;
  int yellow_count = 0;
  int map_status=0;
  bool swing = true;
  bool yellow_in_flag=false;
  bool swing_start = true;
  bool swing_time_start = true;
  bool swing_key = true;
  int before_brightness;
  bool fast_comparison_brightness = true;
  bool getting_brighter = false;
  int now_brightness = 0;
  bool fast_green = true;
  bool yellow_district_after = false;
  bool fast_yellow_district_after = true;
};
