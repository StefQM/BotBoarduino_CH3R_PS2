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
extern const uint8_t GetACos[] PROGMEM;
extern const word GetSin[] PROGMEM;

class Hexapod {
public:
    // --- Timing ---
    unsigned long lTimerStart;
    unsigned long lTimerEnd;
    uint8_t CycleTime;
    word ServoMoveTime;
    word PrevServoMoveTime;

    // --- Profiling State ---
    unsigned long lLogicTimeStart;
    unsigned long lLogicTimeMax;
    unsigned long lSerialTimeMax;
    unsigned long lProfileWindowStart;
    bool fShowProfile;

    // --- Balance State ---
    long TotalTransX, TotalTransY, TotalTransZ;
    long TotalXBal1, TotalYBal1, TotalZBal1;

    // --- Gait State ---
    short NomGaitSpeed;
    short TLDivFactor;
    short NrLiftedPos;
    uint8_t LiftDivFactor;
    uint8_t HalfLiftHeigth;
    bool TravelRequest;
    uint8_t StepsInGait;
    uint8_t GaitStep;
    uint8_t GaitLegNr[6];
    bool LastLeg;

    // --- IK/FK State ---
    bool IKSolution;
    bool IKSolutionWarning;
    bool IKSolutionError;
    short BodyRotOffsetX, BodyRotOffsetY, BodyRotOffsetZ;

    // --- Math Buffers (Moved from globals) ---
    short sin4;
    short cos4;
    short AngleRad4;
    short Atan4;
    short XYhyp2;

    // --- Status ---
    bool fWalking;
    bool fContinueWalking;
    bool fLowVoltageShutdown;
    word Voltage;
    uint8_t Eyes, LedA, LedB, LedC;

    // --- Single Leg Control ---
    uint8_t PrevSelectedLeg;
    bool AllDown;

    Hexapod();
    void init();

    // --- Logic ---
    void GaitSelect();
    void GaitSeq();
    void Gait(uint8_t legIndex);
    void BalanceBody();
    void BalCalcOneLeg(short PosX, short PosZ, short PosY, uint8_t BalLegNr);
    void CheckAngles();
    bool CheckVoltage();
    
    void GetSinCos(short AngleDeg1);
    long GetArcCos(short cos4);
    short GetATan2(short AtanX, short AtanY);
    unsigned long isqrt32(unsigned long n);
};

extern Hexapod g_Hexapod;

#endif // HEXAPOD_H
