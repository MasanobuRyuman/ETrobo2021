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
  const int8_t maxPwm = (Motor::PWM_MAX) / 3;
  //直線の最高速度

  const int8_t straightMaxPwm = (Motor::PWM_MAX) / 1;
  const int8_t pwm = (Motor::PWM_MAX) / 2;
  const int8_t green_pwm = (Motor::PWM_MAX) / 6;
  const int8_t yellow_district_after_pwm = (Motor::PWM_MAX) / 6;
  const int8_t straight_pwm = (Motor::PWM_MAX) / 2;
  const int8_t difficulty_cource_pwm = (Motor::PWM_MAX) / 2;
  const int8_t slow_pwm = (Motor::PWM_MAX) / 6;

  int tracerStatus = 0;
  float body_direction;
  float calc_porp_value();
  float IntegralControl();
  float derivative_control();
  void direction();
  void swing_neck();
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


  const int target = 19;
  //青い線を通る時の基準値
  const int blue_target = 28;
  const int green_target = 26;
  const int last_target = 19;
  int prev_diff = 0;
  const float kp = 0.83;
  const float green_kp = 1.0f;
  //pwmの値がmax / 1　の時
  const float maxkp = 1.0;
  const float kd = 1.0;

  const float green_kd = 2.0f;
  const float ki = 0.0;//i制御の際の定数。
  const float green_ki = 0;
  int light_log_index = 0;

  void color_sensor();
  rgb_raw_t rgb;
  bool line_status_blue = false;
  bool line_status_yellow=false;
  bool line_status_green = false;
  int yellow_count = 0;
  int map_status=0;
  bool swing = true;
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
  bool red_flag = false;
  bool last_caurce = false;
  int light_log[20];
  int light_log_was_made = 0;

  int blue_count = 0;
  bool blue_after_fast = false;
  bool blue_slow_run_end = false;
  int eria = 1;

};
