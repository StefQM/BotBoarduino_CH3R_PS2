#ifndef _CONTROL_STATE_H_
#define _CONTROL_STATE_H_

#if ARDUINO>99
#include <Arduino.h>
#else
#include <Wprogram.h>
#endif

//==============================================================================
// COORD3D: Simple 3D coordinate structure
//==============================================================================
typedef struct _Coord3D {
    long      x;
    long      y;
    long      z;
} COORD3D;

//==============================================================================
// INCONTROLSTATE: This is the main structure of data that the InputController 
//      manipulates and is used by the main Phoenix Code to determine robot behavior.
//==============================================================================
typedef struct _InControlState {
    boolean fHexOn;             // Switch to turn on Phoenix
    boolean fPrev_HexOn;        // Previous loop state 
    
    COORD3D BodyPos;            // Body position offsets
    COORD3D BodyRot1;           // Body rotation (X-Pitch, Y-Rotation, Z-Roll)

    byte    GaitType;           // Current gait selection
    short   LegLiftHeight;      // Height legs lift during gait
    COORD3D TravelLength;       // X-Z movement, Y is rotation

    byte    SelectedLeg;        // Currently selected leg for Single Leg Mode
    COORD3D SLLeg;              // Single leg position offset
    boolean fSLHold;            // Single leg control mode hold flag

    boolean BalanceMode;        // Global balance mode flag

    byte    InputTimeDelay;     // Delay for "sneaking" effect based on input speed
    word    SpeedControl;       // Adjustable overall delay
    byte    ForceGaitStepCnt;   // Force a step even when not moving
} INCONTROLSTATE;

#endif // _CONTROL_STATE_H_
