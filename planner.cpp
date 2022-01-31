/**
 * planner.cpp - Buffer movement commands and manage the acceleration profile plan
 * Part of Grbl
 *
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
 *
 *
 * The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis.
 *
 *
 * Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 *
 * s == speed, a == acceleration, t == time, d == distance
 *
 * Basic definitions:
 *   Speed[s_, a_, t_] := s + (a*t)
 *   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 *
 * Distance to reach a specific speed with a constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 *   d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 *
 * Speed after a given distance of travel with constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 *   m -> Sqrt[2 a d + s^2]
 *
 * DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 *
 * When to start braking (di) to reach a specified destination speed (s2) after accelerating
 * from initial speed s1 without ever stopping at a plateau:
 *   Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 *   di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 *
 * IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 *
 */

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include <stdlib.h>

//===========================================================================
//============================= public variables ============================
//===========================================================================

millis_t minsegmenttime;
float max_feedrate[NUM_AXIS] = {300, 300, 300, 300, 300, 25}; // Max speeds in mm per minute
float axis_steps_per_unit[NUM_AXIS] = {80, 80, 80, 80, 40, 96};
unsigned long max_acceleration_units_per_sq_second[NUM_AXIS] = {3000, 3000, 3000, 3000, 3000, 10000}; // Use M201 to override by software
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
float retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
float travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
float max_xy_jerk = 20;          // The largest speed change requiring no acceleration
float max_z_jerk = 20;
float max_e_jerk = 5;
float max_jerk[NUM_AXIS] = {20, 20, 20, 20, 20, 5};
//float previous_nominal_speed = 0;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

//===========================================================================
//============ semi-private variables, used in inline functions =============
//===========================================================================

block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now

//===========================================================================
//============================ private variables ============================
//===========================================================================

// The current position of the tool in absolute steps
long position[NUM_AXIS];               // Rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[NUM_AXIS]; // Speed of previous path line segment
static float previous_nominal_speed;   // Nominal speed of previous path line segment

uint8_t g_uc_extruder_last_move[EXTRUDERS] = { 0 };


//===========================================================================
//================================ functions ================================
//===========================================================================

// Get the next / previous index of the next block in the ring buffer
// NOTE: Using & here (not %) because BLOCK_BUFFER_SIZE is always a power of 2
FORCE_INLINE int8_t next_block_index(int8_t block_index) { return BLOCK_MOD(block_index + 1); }
FORCE_INLINE int8_t prev_block_index(int8_t block_index) { return BLOCK_MOD(block_index - 1); }

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration) {
  if (acceleration == 0) return 0; // acceleration was 0, set acceleration distance to 0
  return (target_rate * target_rate - initial_rate * initial_rate) / (acceleration * 2);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) {
  if (acceleration == 0) return 0; // acceleration was 0, set intersection distance to 0
  return (acceleration * 2 * distance - initial_rate * initial_rate + final_rate * final_rate) / (acceleration * 4);
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t* block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate * entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate * exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  NOLESS(initial_rate, 120);
  NOLESS(final_rate, 120);

  long acceleration = block->acceleration_st;
  int32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));

  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps, 0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }

  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if (!block->busy) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;

  }
  CRITICAL_SECTION_END;
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t* previous, block_t* current, block_t* next) {
  if (!current) return;
  UNUSED(previous);

  if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if (!current->nominal_length_flag && current->max_entry_speed > next->entry_speed) {
        current->entry_speed = min(current->max_entry_speed,
                                   max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
      }
      else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;

  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
    unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END

  if (BLOCK_MOD(block_buffer_head - tail + BLOCK_BUFFER_SIZE) > 3) { // moves queued
    block_index = BLOCK_MOD(block_buffer_head - 3);
    block_t* block[3] = { NULL, NULL, NULL };
    while (block_index != tail) {
      block_index = prev_block_index(block_index);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t* previous, block_t* current, block_t* next) {
  if (!previous) return;
  UNUSED(next);

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min(current->entry_speed,
                               max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters));
      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t* block[3] = { NULL, NULL, NULL };

  while (block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0], block[1], block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor for each junction. Must be called by planner_recalculate() after
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t* current;
  block_t* next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        float nom = current->nominal_speed;
        calculate_trapezoid_for_block(current, current->entry_speed / nom, next->entry_speed / nom);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index(block_index);
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next) {
    float nom = next->nominal_speed;
    calculate_trapezoid_for_block(next, next->entry_speed / nom, MINIMUM_PLANNER_SPEED / nom);
    next->recalculate_flag = false;
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor)
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if
//     a. The speed increase within one block would require faster acceleration than the one, true
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate() {
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = 0.0;
  previous_nominal_speed = 0.0;
}


void check_axes_activity() {
  unsigned char axis_active[NUM_AXIS] = { 0 },
                tail_fan_speed = fanSpeed;

  block_t* block;

  if (blocks_queued()) {
    uint8_t block_index = block_buffer_tail;
    tail_fan_speed = block_buffer[block_index].fan_speed;
    while (block_index != block_buffer_head) {
      block = &block_buffer[block_index];
      for (int i = 0; i < NUM_AXIS; i++) if (block->steps[i]) axis_active[i]++;
      block_index = next_block_index(block_index);
    }
  }
  /*
  if (DISABLE_A && !axis_active[A_AXIS]) disable_a();
  if (DISABLE_B && !axis_active[B_AXIS]) disable_b();
  if (DISABLE_C && !axis_active[C_AXIS]) disable_c();
  if (DISABLE_D && !axis_active[D_AXIS]) disable_d();
  if (DISABLE_F && !axis_active[F_AXIS]) disable_f();
  if (DISABLE_E && !axis_active[E_AXIS]) {
    disable_e0();
  }
  */
}


float junction_deviation = 0.1;
// Add a new linear movement to the buffer. steps[X_AXIS], _y and _z is the absolute position in
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.

void plan_buffer_line(double a, double b, double c, double d, double f, double e, double feed_rate, uint8_t extruder)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head) {};

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  long target[NUM_AXIS];
  target[A_AXIS] = lround(a * axis_steps_per_unit[A_AXIS]);
  target[B_AXIS] = lround(b * axis_steps_per_unit[B_AXIS]);
  target[C_AXIS] = lround(c * axis_steps_per_unit[C_AXIS]);
  target[D_AXIS] = lround(d * axis_steps_per_unit[D_AXIS]);
  target[F_AXIS] = lround(f * axis_steps_per_unit[F_AXIS]);
  target[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);

  float da = target[A_AXIS] - position[A_AXIS],
        db = target[B_AXIS] - position[B_AXIS],
        dc = target[C_AXIS] - position[C_AXIS],
        dd = target[D_AXIS] - position[D_AXIS],
        df = target[F_AXIS] - position[F_AXIS];
        

  // DRYRUN ignores all temperature constraints and assures that the extruder is instantly satisfied
  //if (marlin_debug_flags & DEBUG_DRYRUN)
    //position[E_AXIS] = target[E_AXIS];

  float de = target[E_AXIS] - position[E_AXIS];

  // Prepare to set up new block
  block_t* block = &block_buffer[block_buffer_head];

  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

  // Number of steps for each axis
    block->steps[A_AXIS] = labs(da);
    block->steps[B_AXIS] = labs(db);
    block->steps[C_AXIS] = labs(dc);
    block->steps[D_AXIS] = labs(dd);
    block->steps[F_AXIS] = labs(df);
    


  block->steps[E_AXIS] = labs(de);
  //block->steps[E_AXIS] *= volumetric_multiplier[extruder];
  //block->steps[E_AXIS] *= extruder_multiplier[extruder];
  //block->steps[E_AXIS] /= 100;
  block->step_event_count = max(block->steps[A_AXIS], max(block->steps[B_AXIS], max(block->steps[C_AXIS], max(block->steps[D_AXIS], max(block->steps[F_AXIS], block->steps[E_AXIS])))));

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments) return;

  //block->fan_speed = fanSpeed;

  // Compute direction bits for this block
  uint8_t dbit = 0;
  if (da < 0) dbit |= BIT(A_AXIS);
  if (db < 0) dbit |= BIT(B_AXIS);
  if (dc < 0) dbit |= BIT(C_AXIS);
  if (dd < 0) dbit |= BIT(D_AXIS);
  if (df < 0) dbit |= BIT(F_AXIS);
  if (de < 0) dbit |= BIT(E_AXIS);
  block->direction_bits = dbit;

  block->active_extruder = extruder;

  //enable active axes
    if (block->steps[A_AXIS]) enable_a();
    if (block->steps[B_AXIS]) enable_b();
    if (block->steps[C_AXIS]) enable_c();
    if (block->steps[D_AXIS]) enable_d();
    if (block->steps[F_AXIS]) enable_f();


  // Enable extruder(s)
  if (block->steps[E_AXIS]) {
        if (g_uc_extruder_last_move[0] > 0) g_uc_extruder_last_move[0]--;

          enable_e0();
          g_uc_extruder_last_move[0] = BLOCK_BUFFER_SIZE * 2;      
    }

  if (block->steps[E_AXIS])
    NOLESS(feed_rate, minimumfeedrate);
  else
    NOLESS(feed_rate, mintravelfeedrate);

  /**
   * This part of the code calculates the total length of the movement.
   * For cartesian bots, the X_AXIS is the real X movement and same for Y_AXIS.
   * But for corexy bots, that is not true. The "X_AXIS" and "Y_AXIS" motors (that should be named to A_AXIS
   * and B_AXIS) cannot be used for X and Y length, because A=X+Y and B=X-Y.
   * So we need to create other 2 "AXIS", named X_HEAD and Y_HEAD, meaning the real displacement of the Head.
   * Having the real displacement of the head, we can calculate the total movement length and apply the desired speed.
   */
    float delta_mm[6];
    delta_mm[A_AXIS] = da / axis_steps_per_unit[A_AXIS];
    delta_mm[B_AXIS] = db / axis_steps_per_unit[B_AXIS];
    delta_mm[C_AXIS] = dc / axis_steps_per_unit[C_AXIS];
    delta_mm[D_AXIS] = dd / axis_steps_per_unit[D_AXIS];
    delta_mm[F_AXIS] = df / axis_steps_per_unit[F_AXIS];
  	delta_mm[E_AXIS] = (de / axis_steps_per_unit[E_AXIS]); /* * volumetric_multiplier[extruder] * extruder_multiplier[extruder] / 100.0; */

  if (block->steps[A_AXIS] <= dropsegments && block->steps[B_AXIS] <= dropsegments && block->steps[C_AXIS] <= dropsegments && 
  	  block->steps[D_AXIS] <= dropsegments && block->steps[F_AXIS] <= dropsegments) {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  else {
    block->millimeters = sqrt(
    square(delta_mm[A_AXIS]) + square(delta_mm[B_AXIS]) + square(delta_mm[C_AXIS]) + square(delta_mm[D_AXIS]) + square(delta_mm[F_AXIS])
    );
  }
  float inverse_millimeters = 1.0 / block->millimeters;  // Inverse millimeters to remove multiple divides

  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued = movesplanned();

  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for (int i = 0; i < NUM_AXIS; i++) {
    current_speed[i] = delta_mm[i] * inverse_second;
    float cs = fabs(current_speed[i]), mf = max_feedrate[i];
    if (cs > mf) speed_factor = min(speed_factor, mf / cs);
  }


  // Correct the speed
  if (speed_factor < 1.0) {
    for (unsigned char i = 0; i < NUM_AXIS; i++) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  float steps_per_mm = block->step_event_count / block->millimeters;
  long bsa = block->steps[A_AXIS], bsb = block->steps[B_AXIS], bsc = block->steps[C_AXIS], bsd = block->steps[D_AXIS], bsf = block->steps[F_AXIS], bse = block->steps[E_AXIS];
  if (bsa == 0 && bsb == 0 && bsc == 0 && bsd == 0 && bsf == 0) {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else if (bse == 0) {
    block->acceleration_st = ceil(travel_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else {
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  // Limit acceleration per axis
  unsigned long acc_st = block->acceleration_st,
                asteps = axis_steps_per_sqr_second[A_AXIS],
                bsteps = axis_steps_per_sqr_second[B_AXIS],
                csteps = axis_steps_per_sqr_second[C_AXIS],
                dsteps = axis_steps_per_sqr_second[D_AXIS],
                fsteps = axis_steps_per_sqr_second[F_AXIS],
                esteps = axis_steps_per_sqr_second[E_AXIS];
  if ((float)acc_st * bsa / block->step_event_count > asteps) acc_st = asteps;
  if ((float)acc_st * bsb / block->step_event_count > bsteps) acc_st = bsteps;
  if ((float)acc_st * bsc / block->step_event_count > csteps) acc_st = csteps;
  if ((float)acc_st * bsd / block->step_event_count > dsteps) acc_st = dsteps;
  if ((float)acc_st * bsf / block->step_event_count > fsteps) acc_st = fsteps;
  if ((float)acc_st * bse / block->step_event_count > esteps) acc_st = esteps;

  block->acceleration_st = acc_st;
  block->acceleration = acc_st / steps_per_mm;
  block->acceleration_rate = (long)(acc_st * 16777216.0 / (F_CPU / 8.0));

  /*
  // Start with a safe speed
  float vmax_junction = max_xy_jerk / 2;
  float vmax_junction_factor = 1.0;
  float mz2 = max_z_jerk / 2, me2 = max_e_jerk / 2;
  float csz = current_speed[C_AXIS], cse = current_speed[E_AXIS];
  if (fabs(csz) > mz2) vmax_junction = min(vmax_junction, mz2);
  if (fabs(cse) > me2) vmax_junction = min(vmax_junction, me2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float dx = current_speed[A_AXIS] - previous_speed[A_AXIS],
          dy = current_speed[B_AXIS] - previous_speed[B_AXIS],
          dz = fabs(csz - previous_speed[C_AXIS]),
          de = fabs(cse - previous_speed[E_AXIS]),
          jerk = sqrt(da * da + db * db);

    //    if ((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) vmax_junction_factor = max_xy_jerk / jerk;
    if (dz > max_z_jerk) vmax_junction_factor = min(vmax_junction_factor, max_z_jerk / dc);
    if (de > max_e_jerk) vmax_junction_factor = min(vmax_junction_factor, max_e_jerk / de);

    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);
  */
  
  // Initial limit on the segment entry velocity
  float vmax_junction;
  
  static float previous_safe_speed;

  float safe_speed = block->nominal_speed;
  uint8_t limited = 0;
  for (int i=0; i < NUM_AXIS; i++) {
    const float jerk = fabs(current_speed[i]), maxj = max_jerk[i];
    if (jerk > maxj) {
      if (limited) {
        const float mjerk = maxj * block->nominal_speed;
        if (jerk * safe_speed > mjerk) safe_speed = mjerk / jerk;
      }
      else {
        ++limited;
        safe_speed = maxj;
      }
    }
  }
  
    if (moves_queued > 1 && previous_nominal_speed > 0.0001) {
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    bool prev_speed_larger = previous_nominal_speed > block->nominal_speed;
    float smaller_speed_factor = prev_speed_larger ? (block->nominal_speed / previous_nominal_speed) : (previous_nominal_speed / block->nominal_speed);
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = prev_speed_larger ? block->nominal_speed : previous_nominal_speed;
    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1.f;
    limited = 0;
    // Now limit the jerk in all axes.
    for (int axis=0; axis < NUM_AXIS; axis++) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit = previous_speed[axis], v_entry = current_speed[axis];
      if (prev_speed_larger) v_exit *= smaller_speed_factor;
      if (limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }

      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry)
          ? //                                  coasting             axis reversal
            ( (v_entry > 0.f || v_exit < 0.f) ? (v_exit - v_entry) : max(v_exit, -v_entry) )
          : // v_exit <= v_entry                coasting             axis reversal
            ( (v_entry < 0.f || v_exit > 0.f) ? (v_entry - v_exit) : max(-v_exit, v_entry) );

      if (jerk > max_jerk[axis]) {
        v_factor *= max_jerk[axis] / jerk;
        ++limited;
      }
    }
    if (limited) vmax_junction *= v_factor;
    // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
    // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
    const float vmax_junction_threshold = vmax_junction * 0.99f;
    if (previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
      // Not coasting. The machine will stop and start the movements anyway,
      // better to start the segment from start.
      //SBI(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
      vmax_junction = safe_speed;
    }
  }
  else {
    //SBI(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
    vmax_junction = safe_speed;
  }

  // Max entry speed of this block equals the max exit speed of the previous block.
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  const float v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);
  
  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  block->nominal_length_flag = (block->nominal_speed <= v_allowable);
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = current_speed[i];
  previous_nominal_speed = block->nominal_speed;


  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update position
  for (int i = 0; i < NUM_AXIS; i++) position[i] = target[i];

  planner_recalculate();

  st_wake_up();

} // plan_buffer_line()


void plan_set_position(const float& a, const float& b, const float& c, const float& d, const float& f, const float& e)
{
	float na = position[A_AXIS] = lround(a * axis_steps_per_unit[A_AXIS]),
		  nb = position[B_AXIS] = lround(b * axis_steps_per_unit[B_AXIS]),
		  nc = position[C_AXIS] = lround(c * axis_steps_per_unit[C_AXIS]),
		  nd = position[D_AXIS] = lround(d * axis_steps_per_unit[D_AXIS]),
		  nf = position[F_AXIS] = lround(f * axis_steps_per_unit[F_AXIS]),
		  ne = position[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
	st_set_position(na, nb, nc, nd, nf, ne);
	previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.

	for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = 0.0;
}

void plan_set_e_position(double e) {
  position[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
  st_set_e_position(position[E_AXIS]);
}

// Calculate the steps/s^2 acceleration rates, based on the mm/s^s
void reset_acceleration_rates() {
  for (int i = 0; i < NUM_AXIS; i++)
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
}
