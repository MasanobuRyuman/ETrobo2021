#include "app.h" 
#include "Tracer.h" 
#include "Clock.h"  

#include <TouchSensor.h>
#include <Sensor.h>

using namespace ev3api;

Tracer tracer;  
Clock clock;    
//TouchSensor touchSensor;




void tracer_task(intptr_t exinf) { 

  ev3_sensor_config (EV3_PORT_1 ,TOUCH_SENSOR );
  while(1){
    if(ev3_touch_sensor_is_pressed(EV3_PORT_1)) break;
  }
  tracer.run(); 
  ext_tsk();
}

void main_task(intptr_t unused) { 
  const uint32_t duration = 100; 

  tracer.init(); 
  sta_cyc(TRACER_CYC); 
  
  while (!ev3_button_is_pressed(LEFT_BUTTON)) { 
      clock.sleep(duration);   
  }

  stp_cyc(TRACER_CYC); 
  tracer.terminate(); 
  ext_tsk(); 
}

