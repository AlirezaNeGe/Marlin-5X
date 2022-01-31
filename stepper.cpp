/**
 * stepper.cpp - stepper motor driver: executes motion plans using stepper motors
 * Marlin Firmware
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "speed_lookuptable.h"
#include "stepper_indirection.h"
#include "fastio.h"
#include "pins.h"
#include "macros.h"


//===========================================================================
//============================= public variables ============================
//===========================================================================
block_t* current_block;  // A pointer to the block currently being traced


//===========================================================================
//============================= private variables ===========================
//===========================================================================
//static makes it impossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits = 0;        // The next stepping-bits to be output
static unsigned int cleaning_buffer_counter;


// Counter variables for the Bresenham line tracer
static long counter_a, counter_b, counter_c, counter_d, counter_f, counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block

static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deceleration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;

volatile long endstops_trigsteps[3] = { 0 };
volatile long endstops_stepsTotal, endstops_stepsDone;
static volatile char endstop_hit_bits = 0; // use X_MIN, Y_MIN, Z_MIN and Z_MIN_PROBE as BIT value

static uint16_t old_endstop_bits = 0; // use X_MIN, X_MAX... Z_MAX, Z_MIN_PROBE, Z2_MIN, Z2_MAX

static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0 };
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1, 1, 1 };


//===========================================================================
//================================ functions ================================
//===========================================================================


#define A_APPLY_DIR(v,Q) A_DIR_WRITE(v)
#define A_APPLY_STEP(v,Q) A_STEP_WRITE(v)

#define B_APPLY_DIR(v,Q) B_DIR_WRITE(v)
#define B_APPLY_STEP(v,Q) B_STEP_WRITE(v)

#define C_APPLY_DIR(v,Q) C_DIR_WRITE(v)
#define C_APPLY_STEP(v,Q) C_STEP_WRITE(v)

#define D_APPLY_DIR(v,Q) D_DIR_WRITE(v)
#define D_APPLY_STEP(v,Q) D_STEP_WRITE(v)

#define F_APPLY_DIR(v,Q) F_DIR_WRITE(v)
#define F_APPLY_STEP(v,Q) F_STEP_WRITE(v)

#define E_APPLY_STEP(v,Q) E_STEP_WRITE(v)

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %A1, %A2 \n\t" \
                 "add %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r0 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (charIn1), \
                 "d" (intIn2) \
                 : \
                 "r26" \
               )

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
//
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "mov r27, r1 \n\t" \
                 "mul %B1, %C2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %C1, %C2 \n\t" \
                 "add %B0, r0 \n\t" \
                 "mul %C1, %B2 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %A1, %C2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %B2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %C1, %A2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %A2 \n\t" \
                 "add r27, r1 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r27 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %D2, %A1 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %D2, %B1 \n\t" \
                 "add %B0, r0 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (longIn1), \
                 "d" (longIn2) \
                 : \
                 "r26" , "r27" \
               )

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= BIT(OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~BIT(OCIE1A)


//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
  if (step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

  if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2) & 0x3fff;
    step_loops = 4;
  }
  else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1) & 0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  if (step_rate < (F_CPU / 500000)) step_rate = (F_CPU / 500000);
  step_rate -= (F_CPU / 500000); // Correct for minimal speed
  if (step_rate >= (8 * 256)) { // higher step rate
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate >> 8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address + 2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate) >> 1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address + 2) * (unsigned char)(step_rate & 0x0007)) >> 3);
  }
  //if (timer < 100) { timer = 100; MYSERIAL.print(MSG_STEPPER_TOO_HIGH); MYSERIAL.println(step_rate); }//(20kHz this should never happen)
  return timer;
}

/**
 * Set the stepper direction of each axis
 *
 *   X_AXIS=A_AXIS and Y_AXIS=B_AXIS for COREXY
 *   X_AXIS=A_AXIS and Z_AXIS=C_AXIS for COREXZ
 */
void set_stepper_direction() {

  if (TEST(out_bits, A_AXIS)) { // A_AXIS
    A_APPLY_DIR(INVERT_A_DIR, 0);
    count_direction[A_AXIS] = -1;
  }
  else {
    A_APPLY_DIR(!INVERT_A_DIR, 0);
    count_direction[A_AXIS] = 1;
  }

  if (TEST(out_bits, B_AXIS)) { // B_AXIS
    B_APPLY_DIR(INVERT_B_DIR, 0);
    count_direction[B_AXIS] = -1;
  }
  else {
    B_APPLY_DIR(!INVERT_B_DIR, 0);
    count_direction[B_AXIS] = 1;
  }

  if (TEST(out_bits, C_AXIS)) { // C_AXIS
    C_APPLY_DIR(INVERT_C_DIR, 0);
    count_direction[C_AXIS] = -1;
  }
  else {
    C_APPLY_DIR(!INVERT_C_DIR, 0);
    count_direction[C_AXIS] = 1;
  }
  
  if (TEST(out_bits, D_AXIS)) { // C_AXIS
    D_APPLY_DIR(INVERT_D_DIR, 0);
    count_direction[D_AXIS] = -1;
  }
  else {
    D_APPLY_DIR(!INVERT_D_DIR, 0);
    count_direction[D_AXIS] = 1;
  }
  
  if (TEST(out_bits, F_AXIS)) { // C_AXIS
    F_APPLY_DIR(INVERT_F_DIR, 0);
    count_direction[F_AXIS] = -1;
  }
  else {
    F_APPLY_DIR(!INVERT_F_DIR, 0);
    count_direction[F_AXIS] = 1;
  }

  if (TEST(out_bits, E_AXIS)) {
	 REV_E_DIR();
	 count_direction[E_AXIS] = -1;
  }
  else {
	 NORM_E_DIR();
	 count_direction[E_AXIS] = 1;
  }
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {

  if (current_block->direction_bits != out_bits) {
    out_bits = current_block->direction_bits;
    set_stepper_direction();
  }

  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;

}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
ISR(TIMER1_COMPA_vect) {

  if (cleaning_buffer_counter) {
    current_block = NULL;
    plan_discard_current_block();
    cleaning_buffer_counter--;
    OCR1A = 200;
    return;
  }

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block) {
      current_block->busy = true;
      trapezoid_generator_reset();
      counter_a = -(current_block->step_event_count >> 1);
      counter_b = counter_c = counter_d = counter_f = counter_e = counter_a;
      step_events_completed = 0;
    }
    else {
      OCR1A = 2000; // 1kHz.
    }
  }

  if (current_block != NULL) {

    // Update endstops state, if enabled
    //if (check_endstops) update_endstops();

    // Take multiple steps per interrupt (For high speed moves)
    for (int8_t i = 0; i < step_loops; i++) {
      
      //#define _AXIS(AXIS) AXIS ##_AXIS
      #define _COUNTER(axis) counter_## axis
      #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
      #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

      #define STEP_ADD(axis, AXIS) \
        _COUNTER(axis) += current_block->steps[_AXIS(AXIS)]; \
        if (_COUNTER(axis) > 0) { _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); }

      STEP_ADD(a,A);
      STEP_ADD(b,B);
      STEP_ADD(c,C);
      STEP_ADD(d,D);
      STEP_ADD(f,F);
      STEP_ADD(e,E);

      #define STEP_IF_COUNTER(axis, AXIS) \
        if (_COUNTER(axis) > 0) { \
          _COUNTER(axis) -= current_block->step_event_count; \
          count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
          _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
        }

      STEP_IF_COUNTER(a, A);
      STEP_IF_COUNTER(b, B);
      STEP_IF_COUNTER(c, C);
      STEP_IF_COUNTER(d, D);
      STEP_IF_COUNTER(f, F);
      STEP_IF_COUNTER(e, E);

      step_events_completed++;
      if (step_events_completed >= current_block->step_event_count) break;
    }
    // Calculate new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long)current_block->accelerate_until) {

      MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;

      // upper limit
      if (acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;

    }
    else if (step_events_completed > (unsigned long)current_block->decelerate_after) {
      MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);

      if (step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if (step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;
    }
    else {
      OCR1A = OCR1A_nominal;
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    OCR1A = (OCR1A < (TCNT1 + 16)) ? (TCNT1 + 16) : OCR1A;

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }
  }
}


void st_init() {

  // Initialize Dir Pins
    A_DIR_INIT;
    B_DIR_INIT;
    C_DIR_INIT;
    D_DIR_INIT;
    F_DIR_INIT;
    E0_DIR_INIT;

  
  //Initialize Enable Pins - steppers default to disabled.
    A_ENABLE_INIT;
    if (!A_ENABLE_ON) A_ENABLE_WRITE(HIGH);
    
    B_ENABLE_INIT;
    if (!B_ENABLE_ON) B_ENABLE_WRITE(HIGH);
    
    C_ENABLE_INIT;
    if (!C_ENABLE_ON) C_ENABLE_WRITE(HIGH);

    D_ENABLE_INIT;
    if (!D_ENABLE_ON) D_ENABLE_WRITE(HIGH);

    F_ENABLE_INIT;
    if (!F_ENABLE_ON) F_ENABLE_WRITE(HIGH);

    E0_ENABLE_INIT;
    if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);
	

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(axis) disable_## axis()

  #define AXIS_INIT(axis, AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(axis)

  #define E_AXIS_INIT(NUM) AXIS_INIT(e## NUM, E## NUM, E)

  // Initialize Step Pins
    AXIS_INIT(a, A, A);
    AXIS_INIT(b, B, B);
    AXIS_INIT(c, C, C);
    AXIS_INIT(d, D, D);
    AXIS_INIT(f, F, F);
    E_AXIS_INIT(0);


  // waveform generation = 0100 = CTC
  TCCR1B &= ~BIT(WGM13);
  TCCR1B |=  BIT(WGM12);
  TCCR1A &= ~BIT(WGM11);
  TCCR1A &= ~BIT(WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  //enable_endstops(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_stepper_direction(); // Init directions to out_bits = 0
}


/**
 * Block until all buffered steps are executed
 */
void st_synchronize() { while (blocks_queued()) {}; }

void st_set_position(const long& a, const long& b, const long& c, const long& d, const long& f, const long& e) {
  CRITICAL_SECTION_START;
  count_position[A_AXIS] = a;
  count_position[B_AXIS] = b;
  count_position[C_AXIS] = c;
  count_position[D_AXIS] = d;
  count_position[F_AXIS] = f;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(double e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis) {
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

float st_get_position_mm(AxisEnum axis) { return st_get_position(axis) / axis_steps_per_unit[axis]; }

void finishAndDisableSteppers() {
  st_synchronize();
  disable_all_steppers();
}

void quickStop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (blocks_queued()) plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}
