/**
 * pins.h
 */

#ifndef PINS_H
#define PINS_H

/*
#define A_STEP_PIN         54
#define A_DIR_PIN          55
#define A_ENABLE_PIN       38
*/
//پینهای موتور1
#define A_STEP_PIN         32
#define A_DIR_PIN          47
#define A_ENABLE_PIN       45

/*
#define B_STEP_PIN         60
#define B_DIR_PIN          61
#define B_ENABLE_PIN       56
*/
//پینهای موتور2
#define B_STEP_PIN         35
#define B_DIR_PIN          33
#define B_ENABLE_PIN       37

//پینهای موتور3
#define C_STEP_PIN         46
#define C_DIR_PIN          48
#define C_ENABLE_PIN       39
//پینهای  مربوط به موتور دورانی حول محور ایکس
#define D_STEP_PIN         54
#define D_DIR_PIN          55
#define D_ENABLE_PIN       38
// پینهای دورانی حول محور زد
#define F_STEP_PIN         60
#define F_DIR_PIN          61
#define F_ENABLE_PIN       56
// پینهای دورانی  اکسترودر
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

// پینهای فن هیتر و سنسور
#define FAN_PIN          10
#define HEATER_0_PIN     8
#define TEMP_0_PIN       A13

#endif //__PINS_H
