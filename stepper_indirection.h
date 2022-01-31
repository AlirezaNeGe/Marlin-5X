/*
  stepper_indirection.h - stepper motor driver indirection macros
  to allow some stepper functions to be done via SPI/I2c instead of direct pin manipulation
  Part of Marlin

  Copyright (c) 2015 Dominik Wenger

  Marlin is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Marlin is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Marlin.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPER_INDIRECTION_H
#define STEPPER_INDIRECTION_H

#include "macros.h"
#include "fastio.h"
#include "pins.h"

// A motor
#define A_STEP_INIT SET_OUTPUT(A_STEP_PIN)
#define A_STEP_WRITE(STATE) WRITE(A_STEP_PIN,STATE)
#define A_STEP_READ READ(A_STEP_PIN)

#define A_DIR_INIT SET_OUTPUT(A_DIR_PIN)
#define A_DIR_WRITE(STATE) WRITE(A_DIR_PIN,STATE)
#define A_DIR_READ READ(A_DIR_PIN)

#define A_ENABLE_INIT SET_OUTPUT(A_ENABLE_PIN)
#define A_ENABLE_WRITE(STATE) WRITE(A_ENABLE_PIN,STATE)
#define A_ENABLE_READ READ(A_ENABLE_PIN)


// B motor
#define B_STEP_INIT SET_OUTPUT(B_STEP_PIN)
#define B_STEP_WRITE(STATE) WRITE(B_STEP_PIN,STATE)
#define B_STEP_READ READ(B_STEP_PIN)

#define B_DIR_INIT SET_OUTPUT(B_DIR_PIN)
#define B_DIR_WRITE(STATE) WRITE(B_DIR_PIN,STATE)
#define B_DIR_READ READ(B_DIR_PIN)

#define B_ENABLE_INIT SET_OUTPUT(B_ENABLE_PIN)
#define B_ENABLE_WRITE(STATE) WRITE(B_ENABLE_PIN,STATE)
#define B_ENABLE_READ READ(B_ENABLE_PIN)

// C motor
#define C_STEP_INIT SET_OUTPUT(C_STEP_PIN)
#define C_STEP_WRITE(STATE) WRITE(C_STEP_PIN,STATE)
#define C_STEP_READ READ(C_STEP_PIN)

#define C_DIR_INIT SET_OUTPUT(C_DIR_PIN)
#define C_DIR_WRITE(STATE) WRITE(C_DIR_PIN,STATE)
#define C_DIR_READ READ(C_DIR_PIN)

#define C_ENABLE_INIT SET_OUTPUT(C_ENABLE_PIN)
#define C_ENABLE_WRITE(STATE) WRITE(C_ENABLE_PIN,STATE)
#define C_ENABLE_READ READ(C_ENABLE_PIN)

// D motor
#define D_STEP_INIT SET_OUTPUT(D_STEP_PIN)
#define D_STEP_WRITE(STATE) WRITE(D_STEP_PIN,STATE)
#define D_STEP_READ READ(D_STEP_PIN)

#define D_DIR_INIT SET_OUTPUT(D_DIR_PIN)
#define D_DIR_WRITE(STATE) WRITE(D_DIR_PIN,STATE)
#define D_DIR_READ READ(D_DIR_PIN)

#define D_ENABLE_INIT SET_OUTPUT(D_ENABLE_PIN)
#define D_ENABLE_WRITE(STATE) WRITE(D_ENABLE_PIN,STATE)
#define D_ENABLE_READ READ(D_ENABLE_PIN)

// F motor
#define F_STEP_INIT SET_OUTPUT(F_STEP_PIN)
#define F_STEP_WRITE(STATE) WRITE(F_STEP_PIN,STATE)
#define F_STEP_READ READ(F_STEP_PIN)

#define F_DIR_INIT SET_OUTPUT(F_DIR_PIN)
#define F_DIR_WRITE(STATE) WRITE(F_DIR_PIN,STATE)
#define F_DIR_READ READ(F_DIR_PIN)

#define F_ENABLE_INIT SET_OUTPUT(F_ENABLE_PIN)
#define F_ENABLE_WRITE(STATE) WRITE(F_ENABLE_PIN,STATE)
#define F_ENABLE_READ READ(F_ENABLE_PIN)

// E0 motor
#define E0_STEP_INIT SET_OUTPUT(E0_STEP_PIN)
#define E0_STEP_WRITE(STATE) WRITE(E0_STEP_PIN,STATE)
#define E0_STEP_READ READ(E0_STEP_PIN)

#define E0_DIR_INIT SET_OUTPUT(E0_DIR_PIN)
#define E0_DIR_WRITE(STATE) WRITE(E0_DIR_PIN,STATE)
#define E0_DIR_READ READ(E0_DIR_PIN)

#define E0_ENABLE_INIT SET_OUTPUT(E0_ENABLE_PIN)
#define E0_ENABLE_WRITE(STATE) WRITE(E0_ENABLE_PIN,STATE)
#define E0_ENABLE_READ READ(E0_ENABLE_PIN)


#endif // STEPPER_INDIRECTION_H
