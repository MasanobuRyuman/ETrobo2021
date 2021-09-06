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


  int tracerStatus = 0;
  float body_direction;
  float calc_porp_value();
  float IntegralControl();
  float derivative_control();
  void direction();
  void swing_neck();
  int32_t left_counts;
  int32_t right_counts;

  float x=-35; //x座標
  float y=0; //y座標

  int32_t bef_cl = 0;//前回のカウント
  int32_t bef_cr=0;
  float now_angle;//今の向き
  void get_coordinates(int32_t cl, int32_t cr);

  char s[256];

  int area=1;
  int halfway_point=1;
  float v_x;
  float v_y;
  float v_length;

  float d0=0;//基準点角度
  float x0 = -35; //x座標
  float y0 = 0;   //y座標
  int32_t cl0=0; //基準点カウント左
  int32_t cr0=0; //基準点カウント右


  const int target = 22;
  //青い線を通る時の基準値
  const int blue_target = 30;
  const int green_target = 29;
  const int last_target = 22;
  int prev_diff = 0;

  
  //速度
  const int8_t pwm = (Motor::PWM_MAX) / 1.2;
  const int8_t straight_road_pwm = (Motor::PWM_MAX) / 1.0;
  const int8_t area5_road_pwm = (Motor::PWM_MAX) / 1.5;
  const int8_t fast_curve_pwm = (Motor::PWM_MAX) / 2;
  const int8_t second_curve_pwm = (Motor::PWM_MAX) / 2;
  const int8_t third_curve_pwm = (Motor::PWM_MAX) / 6;
  const int8_t green_pwm = (Motor::PWM_MAX) / 6;
  const int8_t yellow_district_after_pwm = (Motor::PWM_MAX) / 6;
  const int8_t straight_pwm = (Motor::PWM_MAX) / 2;
  const int8_t difficulty_cource_pwm = (Motor::PWM_MAX) / 1.0;
  const int8_t slow_pwm = (Motor::PWM_MAX) / 6;
  const int8_t fast_slow_pwm = (Motor::PWM_MAX) / 5;
  //P制御の係数
  const float kp = 0.83;
  const float straight_road_kp = 0.83;
  const float area5_road_kp = 0.83;
  const float fast_curve_kp = 0.83;
  const float second_curve_kp = 0.83;
  const float third_curve_kp = 0.83;
  const float green_kp = 1.0f;
  //D制御の係数
  const float kd = 2.0;
  const float straight_road_kd = 2.2;
  const float area5_road_kd = 2.2;
  const float fast_curve_kd = 1.2;
  const float second_curve_kd = 1.2;
  const float third_curve_kd = 2.0;
  const float green_kd = 2.0f;
  //I制御の係数
  const float ki = 0.5;//i制御の際の定数。
  const float straight_road_ki = 0;
  const float area5_road_ki = 0;
  const float fast_curve_ki = 0.5;
  const float second_curve_ki = 0.5;
  const float third_curve_ki = 0.5;
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
  bool run_fast = true;


  int blue_count = 0;
  bool blue_after_fast = false;
  bool blue_slow_run_end = false;
  int eria = 1;

  float turn;
  float pwm_l;
  float pwm_r;

  float derivative;
};
