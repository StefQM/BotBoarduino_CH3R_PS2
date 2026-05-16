//==============================================================================
// GLOBALS - The main global definitions for the CPhenix program
//==============================================================================
#ifndef _HEX_GLOBALS_H_
#define _HEX_GLOBALS_H_

#include <stdarg.h>
#ifndef ARDUINO_ARCH_STM32
#include <SoftwareSerial.h>
#endif
#include "Hex_Cfg.h"
#include "ControlState.h"
#include "InputController.h"
#include "ServoDriver.h"

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP   1

#define c1DEC       10
#define c2DEC       100
#define c4DEC       10000
#define c6DEC       1000000

#define cRR         0
#define cRM         1
#define cRF         2
#define cLR         3
#define cLM         4
#define cLF         5

#define NUM_GAITS    5
#define BalanceDivFactor 6
#define cTravelDeadZone 4

// Forward declarations/externs for functions defined in .ino
extern void GaitSelect(void);
extern short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, uint8_t CtrlDivider);
extern void MSound(uint8_t cNotes, ...);
extern bool CheckVoltage(void);

#include "Leg.h"
#include "Hexapod.h"

//-----------------------------------------------------------------------------
// Define global class objects
//-----------------------------------------------------------------------------
extern ServoDriver      g_ServoDriver;      // Global servo driver
extern InputController  g_InputController;  // Global input controller 
extern INCONTROLSTATE   g_InControlState;   // Shared robot control state
extern Leg              g_Legs[6];          // The six legs
extern Hexapod          g_Hexapod;          // The robot-wide coordinator

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
extern bool          g_fDebugOutput;

#endif // _HEX_GLOBALS_H_
