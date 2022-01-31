// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline
/**
 * Compiler warning on unused varable.
 */
#define UNUSED(x) (void) (x)

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"


#include "Arduino.h"

typedef unsigned long millis_t;

// Arduino < 1.0.0 does not define this, so we need to do it ourselves
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) ((p) + 0xA0)
#endif

//#include "MarlinSerial.h"

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define _AXIS(AXIS) AXIS ##_AXIS

/*
// Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char* str) {
  char ch;
  while ((ch = pgm_read_byte(str))) {
    MYSERIAL.write(ch);
    str++;
  }
}
*/
// پینهای میکرو سوئیچ
#define A_MIN 16
#define B_MIN 23
#define C_MIN 27



void get_command();

void idle(); // the standard idle routine calls manage_inactivity(false)

void manage_inactivity(bool ignore_stepper_queue = false);


#define enable_a() ;
#define disable_a() ;

#define enable_b() ;
#define disable_b() ;

#define enable_c() ;
#define disable_c() ;

#define enable_d() ;
#define disable_d() ;

#define enable_f() ;
#define disable_f() ;

#define enable_e0()  /* nothing */
#define disable_e0() /* nothing */

#define UNEAR_ZERO(x) ((x) < 0.000001)
/**
 * The axis order in all axis related arrays is X, Y, Z, E
 */
#define NUM_AXIS 6

/**
 * Axis indices as enumerated constants
 *
 * A_AXIS and B_AXIS are used by COREXY printers
 * X_HEAD and Y_HEAD is used for systems that don't have a 1:1 relationship between X_AXIS and X Head movement, like CoreXY bots.
 */
enum AxisEnum {A_AXIS = 0, B_AXIS = 1, C_AXIS = 2, D_AXIS = 3, F_AXIS = 4, E_AXIS = 5};

//enum EndstopEnum {X_MIN = 0, Y_MIN = 1, Z_MIN = 2, Z_MIN_PROBE = 3, X_MAX = 4, Y_MAX = 5, Z_MAX = 6, Z2_MIN = 7, Z2_MAX = 8};

void enable_all_steppers();
void disable_all_steppers();

void FlushSerialRequestResend();
void ok_to_send();

void reset_bed_level();
void prepare_move();
void kill(const char*);
void Stop();

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  void filrunout();
#endif

/**
 * Debug flags - not yet widely applied
 */
enum DebugFlags {
  DEBUG_ECHO          = BIT(0),
  DEBUG_INFO          = BIT(1),
  DEBUG_ERRORS        = BIT(2),
  DEBUG_DRYRUN        = BIT(3),
  DEBUG_COMMUNICATION = BIT(4),
  DEBUG_LEVELING      = BIT(5)
};
extern uint8_t marlin_debug_flags;

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }

bool enqueuecommand(const char* cmd); //put a single ASCII command at the end of the current buffer or return false when it is full
void enqueuecommands_P(const char* cmd); //put one or many ASCII commands at the end of the current buffer, read from flash

void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

extern millis_t previous_cmd_ms;
inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }

#if ENABLED(FAST_PWM_FAN)
  void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

extern bool axis_relative_modes[];
extern int feedrate_multiplier;
extern bool volumetric_enabled;
extern int extruder_multiplier[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float filament_size[EXTRUDERS]; // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
extern float volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS];
extern float home_offset[3]; // axis[n].home_offset
extern float min_pos[3]; // axis[n].min_pos
extern float max_pos[3]; // axis[n].max_pos
extern bool axis_known_position[3]; // axis[n].is_known

extern int fanSpeed;



extern millis_t print_job_start_ms;
extern millis_t print_job_stop_ms;

// Handling multiple extruders pins
extern uint8_t active_extruder;


extern void calculate_volumetric_multipliers();

#endif //MARLIN_H
