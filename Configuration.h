#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//#include "boards.h"
#include "macros.h"

// @section machine


#define EXTRUDERS 1

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
//#define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
//#define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

//// The following define selects which power supply you have. Please choose the one that matches your setup
// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)
// :{1:'ATX',2:'X-Box 360'}


//===========================================================================
//============================= Mechanical Settings =========================
//===========================================================================

// @section machine

// Uncomment this option to enable CoreXY kinematics
//#define COREXY

// Uncomment this option to enable CoreXZ kinematics
//#define COREXZ

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================



// Enable this option for Toshiba steppers
//#define CONFIG_STEPPERS_TOSHIBA

// @section homing


// If you want to enable the Z probe pin, but disable its use, uncomment the line below.
// This only affects a Z probe endstop if you have separate Z min endstop as well and have
// activated Z_MIN_PROBE_ENDSTOP below. If you are using the Z Min endstop on your Z probe,
// this has no effect.
//#define DISABLE_Z_MIN_PROBE_ENDSTOP

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
// :{0:'Low',1:'High'}
#define A_ENABLE_ON 1
#define B_ENABLE_ON 1
#define C_ENABLE_ON 1
#define D_ENABLE_ON 1
#define F_ENABLE_ON 1
#define E_ENABLE_ON 1 // For all extruders

// Disables axis when it's not being used.
// WARNING: When motors turn off there is a chance of losing position accuracy!
#define DISABLE_A false
#define DISABLE_B false
#define DISABLE_C false
#define DISABLE_D false
#define DISABLE_F false

// @section extruder

#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER true //disable only inactive extruders and keep active extruder enabled

// @section machine

// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.
#define INVERT_A_DIR false // DELTA does not invert
#define INVERT_B_DIR false
#define INVERT_C_DIR false
#define INVERT_D_DIR false
#define INVERT_F_DIR false

// @section extruder

// For direct drive extruder v9 set to true, for geared extruder set to false.
#define INVERT_E0_DIR false


// @section machine

// Travel limits after homing (units are in mm)
#define X_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Y_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Z_MIN_POS 0
#define X_MAX_POS DELTA_PRINTABLE_RADIUS
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS


// @section movement

/**
 * MOVEMENT SETTINGS
 */

// default settings
// delta speeds must be the same on xyz
#define DEFAULT_AXIS_STEPS_PER_UNIT   {53, 53, 53, 53, 53, 760*1.1}  // default steps per unit for Kossel (GT2, 20 tooth)
#define DEFAULT_MAX_FEEDRATE          {500, 500, 500, 500, 500, 25}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000, 9000, 9000, 9000, 9000, 10000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000    // E acceleration in mm/s^2 for retracts
#define DEFAULT_TRAVEL_ACCELERATION   3000    // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 20.0    // (mm/sec) Must be same as XY for delta
#define DEFAULT_EJERK                 5.0     // (mm/sec)


//===========================================================================
//============================= PID Settings ================================
//===========================================================================
// PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning

// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
//#define PID_DEBUG // Sends debug data to the serial port.
//#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
//#define SLOW_PWM_HEATERS // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
//#define PID_PARAMS_PER_EXTRUDER // Uses separate PID parameters for each extruder (useful for mismatched extruders)
                                // Set/get with gcode: M301 E[extruder number, 0-2]
#define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                              // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
#define PID_INTEGRAL_DRIVE_MAX PID_MAX  //limit for the integral term
#define K1 0.95 //smoothing factor within the PID

// If you are using a pre-configured hotend then you can use one of the value sets by uncommenting it
// Ultimaker
#define  DEFAULT_Kp 22.2
#define  DEFAULT_Ki 1.08
#define  DEFAULT_Kd 114
#define MAX_BED_POWER 255
// MakerGear
//#define  DEFAULT_Kp 7.0
//#define  DEFAULT_Ki 0.1
//#define  DEFAULT_Kd 12

// Mendel Parts V9 on 12V
//#define  DEFAULT_Kp 63.0
//#define  DEFAULT_Ki 2.25
//#define  DEFAULT_Kd 440




#include "Configuration_adv.h"
//#include "thermistortables.h"

#endif //CONFIGURATION_H
