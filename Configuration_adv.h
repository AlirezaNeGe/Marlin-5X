#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

#include "Conditionals.h"

// @section temperature


//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// @section homing

#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

// @section extras

// @section machine

#define AXIS_RELATIVE_MODES {false, false, false, false, false, false}

// @section machine

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_A_STEP_PIN false
#define INVERT_B_STEP_PIN false
#define INVERT_C_STEP_PIN false
#define INVERT_D_STEP_PIN false
#define INVERT_F_STEP_PIN false
#define INVERT_E_STEP_PIN false

// Default stepper release if idle. Set to 0 to deactivate.
#define DEFAULT_STEPPER_DEACTIVE_TIME 60

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0


// @section extras

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000



// Frequency limit
// See nophead's blog for more info
// Not working O
//#define XY_FREQUENCY_LIMIT  15

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
#define MICROSTEP_MODES {16,16,16,16,16,16} // [1,2,4,8,16]

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
#define DIGIPOT_MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)

// uncomment to enable an I2C based DIGIPOT like on the Azteeg X3 Pro
//#define DIGIPOT_I2C
// Number of channels available for I2C digipot, For Azteeg X3 Pro we have 8
#define DIGIPOT_I2C_NUM_CHANNELS 8
// actual motor currents in Amps, need as many here as DIGIPOT_I2C_NUM_CHANNELS
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value

//#define CHDK 4        //Pin for triggering CHDK to take a picture see how to use it here http://captain-slow.dk/2014/03/09/3d-printing-timelapses/
#define CHDK_DELAY 50 //How long in ms the pin should stay HIGH before going LOW again

// @section extruder

// extruder advance constant (s2/mm3)
//
// advance (steps) = STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K * cubic mm per second ^ 2
//
// Hooke's law says:    force = k * distance
// Bernoulli's principle says:  v ^ 2 / 2 + g . h + pressure / density = constant
// so: v ^ 2 is proportional to number of steps we advance the extruder

// @section extras

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

const unsigned int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement

// @section temperature

// Control heater 0 and heater 1 in parallel.
//#define HEATERS_PARALLEL

//===========================================================================
//================================= Buffers =================================
//===========================================================================

// @section hidden

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ring-buffering.

#define BLOCK_BUFFER_SIZE 16 // maximize block buffer


// @section more

//The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

// Bad Serial-connections can miss a received command by sending an 'ok'
// Therefore some clients abort after 30 seconds in a timeout.
// Some other clients start sending commands while receiving a 'wait'.
// This "wait" is only sent when the buffer is empty. 1 second is a good value here.
//#define NO_TIMEOUTS 1000 // Milliseconds

// Some clients will have this feature soon. This could make the NO_TIMEOUTS unnecessary.
//#define ADVANCED_OK

// @section fwretract

// Firmware based and LCD controlled retract
// M207 and M208 can be used to define parameters for the retraction.
// The retraction can be called by the slicer using G10 and G11
// until then, intended retractions can be detected by moves that only extrude and the direction.
// the moves are than replaced by the firmware controlled ones.


/******************************************************************************\
 * enable this section if you have TMC26X motor drivers.
 * you need to import the TMC26XStepper library into the arduino IDE for this
 ******************************************************************************/

// @section tmc

/******************************************************************************\
 * enable this section if you have L6470  motor drivers.
 * you need to import the L6470 library into the arduino IDE for this
 ******************************************************************************/
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0
#define BED_CHECK_INTERVAL 5000

#include "Conditionals.h"
//#include "SanityCheck.h"

#endif //CONFIGURATION_ADV_H
