#include "planner.h"
#include "stepper.h"
#include "Marlin.h"


void setup() {

  plan_init();
  st_init();

}

void loop() {
  //plan_buffer_line(0, 0, 0, 0, 0, 0, 10, 0);
}
