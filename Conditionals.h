/**
 * Conditionals.h
 * Defines that depend on configuration but are not editable.
 */
#include "Arduino.h"
 
#ifndef CONDITIONALS_H

#ifndef M_PI
  #define M_PI 3.1415926536
#endif


#define CONDITIONALS_H

#include "pins.h"
#include "Arduino.h"

/**
* ENDSTOPPULLUPS
*/
/**
* Axis lengths
*/
#define A_MAX_LENGTH (A_MAX_POS - A_MIN_POS)
#define B_MAX_LENGTH (B_MAX_POS - B_MIN_POS)
#define C_MAX_LENGTH (C_MAX_POS - C_MIN_POS)
#define D_MAX_LENGTH (D_MAX_POS - D_MIN_POS)
#define F_MAX_LENGTH (F_MAX_POS - F_MIN_POS)


/**
* MAX_STEP_FREQUENCY differs for TOSHIBA
*/
#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)


/**
* ARRAY_BY_EXTRUDERS based on EXTRUDERS
*/
#define ARRAY_BY_EXTRUDERS(v1, v2, v3, v4) { v1 }
#define ARRAY_BY_EXTRUDERS1(v1) ARRAY_BY_EXTRUDERS(v1, v1, v1, v1)

/**
* Shorthand for pin tests, used wherever needed
*/


#define HAS_STEPPER_RESET (PIN_EXISTS(STEPPER_RESET))
#define HAS_A_ENABLE (PIN_EXISTS(A_ENABLE))
#define HAS_B_ENABLE (PIN_EXISTS(B_ENABLE))
#define HAS_C_ENABLE (PIN_EXISTS(C_ENABLE))
#define HAS_D_ENABLE (PIN_EXISTS(D_ENABLE))
#define HAS_F_ENABLE (PIN_EXISTS(F_ENABLE))
#define HAS_E0_ENABLE (PIN_EXISTS(E0_ENABLE))

#define HAS_A_DIR (PIN_EXISTS(A_DIR))
#define HAS_B_DIR (PIN_EXISTS(B_DIR))
#define HAS_C_DIR (PIN_EXISTS(C_DIR))
#define HAS_D_DIR (PIN_EXISTS(D_DIR))
#define HAS_F_DIR (PIN_EXISTS(F_DIR))
#define HAS_E0_DIR (PIN_EXISTS(E0_DIR))

#define HAS_A_STEP (PIN_EXISTS(A_STEP))
#define HAS_B_STEP (PIN_EXISTS(B_STEP))
#define HAS_C_STEP (PIN_EXISTS(C_STEP))
#define HAS_D_STEP (PIN_EXISTS(D_STEP))
#define HAS_F_STEP (PIN_EXISTS(F_STEP))
#define HAS_E0_STEP (PIN_EXISTS(E0_STEP))

#define TEMP_SENSOR_0 1
#define THERMISTORHEATER_0 TEMP_SENSOR_0
#define HEATER_0_USES_THERMISTOR
#define HAS_HEATER_0 (PIN_EXISTS(HEATER_0))
#define WRITE_HEATER_0(v) WRITE(HEATER_0_PIN, v)



  

#endif //CONDITIONALS_H
