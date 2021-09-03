#include "app.h" 
#include "Tracer.h" 
#include "Clock.h"  
#include "TouchSensor.h"
using namespace ev3api;

Tracer tracer;  
Clock clock;    

void tracer_task(intptr_t exinf) { 
  /*
  if (TouchSensor	(EV3_PORT_1).isPressed()){
    syslog(7,"kita");
  }
  */
  
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

