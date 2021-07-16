#include "app.h" 
#include "Tracer.h" 
#include "Clock.h"  
using namespace ev3api;

Tracer tracer;  
Clock clock;    

void tracer_task(intptr_t exinf) { 
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

