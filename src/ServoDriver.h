//==============================================================================
// ServoDriver.h - Abstract API for robot servo control
//==============================================================================
#ifndef _Servo_Driver_h_
#define _Servo_Driver_h_

#include "Hex_Cfg.h"
#if ARDUINO>99
#include <Arduino.h>
#else
#include <Wprogram.h>
#endif

class ServoDriver {
public:
    // Initialize the servo hardware
    void Init(void);

    // --- GP Player Interface (Optional) ---
#ifdef OPT_GPPLAYER    
    bool FIsGPEnabled(void);
    bool FIsGPSeqDefined(uint8_t iSeq);
    bool FIsGPSeqActive(void);
    void    GPStartSeq(uint8_t iSeq);
    void    GPPlayer(void);
#endif

    // --- Servo Update Interface ---
    // Prepare the driver for a new set of servo updates
    void BeginServoUpdate(void);

    // Send angle information for a specific leg
#ifdef c4DOF
    void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1);
#else
    void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
#endif    

    // Commit the update and move servos over wMoveTime ms
    void CommitServoDriver(word wMoveTime);

    // Relax all servos
    void FreeServos(void);

    // --- Diagnostic/Maintenance Interface ---
#ifdef OPT_FIND_SERVO_OFFSETS
    void FindServoOffsets(void);
#endif    
#ifdef OPT_SSC_FORWARDER
    void SSCForwarder(void);
#endif
};   

#endif //_Servo_Driver_h_
