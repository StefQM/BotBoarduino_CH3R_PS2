#ifndef HEXAPOD_H
#define HEXAPOD_H

#if ARDUINO > 99
#include <Arduino.h>
#else
#include "Arduino.h"
#endif

#include "Leg.h"
#include "InputController.h"

// --- Math Tables (Defined in .ino) ---
extern const byte GetACos[] PROGMEM;
extern const word GetSin[] PROGMEM;

class Hexapod {
public:
    // --- Timing ---
    unsigned long lTimerStart;
    unsigned long lTimerEnd;
    byte CycleTime;
    word ServoMoveTime;
    word PrevServoMoveTime;

    // --- Balance State ---
    long TotalTransX, TotalTransY, TotalTransZ;
    long TotalXBal1, TotalYBal1, TotalZBal1;

    // --- Gait State ---
    short NomGaitSpeed;
    short TLDivFactor;
    short NrLiftedPos;
    byte LiftDivFactor;
    byte HalfLiftHeigth;
    boolean TravelRequest;
    byte StepsInGait;
    byte GaitStep;
    byte GaitLegNr[6];
    boolean LastLeg;

    // --- IK/FK State ---
    boolean IKSolution;
    boolean IKSolutionWarning;
    boolean IKSolutionError;
    short BodyRotOffsetX, BodyRotOffsetY, BodyRotOffsetZ;

    // --- Math Buffers (Moved from globals) ---
    short sin4;
    short cos4;
    short AngleRad4;
    short Atan4;
    short XYhyp2;

    // --- Status ---
    boolean fWalking;
    boolean fContinueWalking;
    boolean fLowVoltageShutdown;
    word Voltage;
    byte Eyes, LedA, LedB, LedC;

    // --- Single Leg Control ---
    byte PrevSelectedLeg;
    boolean AllDown;

    Hexapod();
    void init();

    // --- Logic ---
    void GaitSelect();
    void GaitSeq();
    void Gait(byte legIndex);
    void BalanceBody();
    void BalCalcOneLeg(short PosX, short PosZ, short PosY, byte BalLegNr);
    void CheckAngles();
    bool CheckVoltage();
    
    void GetSinCos(short AngleDeg1);
    long GetArcCos(short cos4);
    short GetATan2(short AtanX, short AtanY);
    unsigned long isqrt32(unsigned long n);
};

extern Hexapod g_Hexapod;

#endif // HEXAPOD_H
