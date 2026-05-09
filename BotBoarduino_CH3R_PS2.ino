//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino 
//=============================================================================
// Header Files
//=============================================================================

#define DEFINE_HEX_GLOBALS
#if ARDUINO>99
#include <Arduino.h>
#else
#endif
#include <PS2X_lib.h>
#include <pins_arduino.h>
#include <SoftwareSerial.h>
#include "Hex_globals.h"
#define BalanceDivFactor 6    

//--------------------------------------------------------------------
//[TABLES]
extern const byte GetACos[] PROGMEM = {
                    255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225,
                    224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193,
                    192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,
                    157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117,
                    115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70,
                    70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
                    59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
                    46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
                    28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
                    16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0 };

extern const word GetSin[] PROGMEM = {0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564,  
                 1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007,
                 3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383,
                 4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664,
                 5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819,
                 6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826,
                 7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660,
                 8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304,
                 9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743,
                 9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969,
                 9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000 };

extern const short cFemurHornOffset1[] PROGMEM = {
#ifdef cRRFemurHornOffset1
  cRRFemurHornOffset1,  cRMFemurHornOffset1,  cRFFemurHornOffset1,  cLRFemurHornOffset1,  cLMFemurHornOffset1,  cLFFemurHornOffset1
#else
  0, 0, 0, 0, 0, 0
#endif
};

#ifdef c4DOF
extern const short cTarsHornOffset1[] PROGMEM = {
#ifdef cRRTarsHornOffset1
  cRRTarsHornOffset1,  cRMTarsHornOffset1,  cRFTarsHornOffset1,  cLRTarsHornOffset1,  cLMTarsHornOffset1,  cLFTarsHornOffset1      
#else
  0, 0, 0, 0, 0, 0
#endif
};
#endif

extern const short cCoxaMin1[] PROGMEM = {cRRCoxaMin1,  cRMCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLFCoxaMin1};      
extern const short cCoxaMax1[] PROGMEM = {cRRCoxaMax1,  cRMCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLFCoxaMax1};      
extern const short cFemurMin1[] PROGMEM ={cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1};     
extern const short cFemurMax1[] PROGMEM ={cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1};     
extern const short cTibiaMin1[] PROGMEM ={cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1};     
extern const short cTibiaMax1[] PROGMEM = {cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1};    

#ifdef c4DOF
extern const short cTarsMin1[] PROGMEM = {cRRTarsMin1, cRMTarsMin1, cRFTarsMin1, cLRTarsMin1, cLMTarsMin1, cLFTarsMin1};
extern const short cTarsMax1[] PROGMEM = {cRRTarsMax1, cRMTarsMax1, cRFTarsMax1, cLRTarsMax1, cLMTarsMax1, cLFTarsMax1};
#endif

extern const byte cCoxaLength[] PROGMEM = {cRRCoxaLength,  cRMCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLMCoxaLength,  cLFCoxaLength};
extern const byte cFemurLength[] PROGMEM = {cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength};
extern const byte cTibiaLength[] PROGMEM = {cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength};
#ifdef c4DOF
extern const byte cTarsLength[] PROGMEM = {cRRTarsLength, cRMTarsLength, cRFTarsLength, cLRTarsLength, cLMTarsLength, cLFTarsLength};
#endif

extern const short cOffsetX[] PROGMEM = {cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX};
extern const short cOffsetZ[] PROGMEM = {cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ};
extern const short cCoxaAngle1[] PROGMEM = {cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1};
extern const short cInitPosX[] PROGMEM = {cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
extern const short cInitPosY[] PROGMEM = {cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};
extern const short cInitPosZ[] PROGMEM = {cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};

//--------------------------------------------------------------------
//[GLOABAL]
INCONTROLSTATE   g_InControlState;
Leg              g_Legs[6];
ServoDriver      g_ServoDriver;
boolean          g_fShowDebugPrompt;
boolean          g_fDebugOutput = true;
byte             LegIndex;

// Prototypes for methods in Hexapod.cpp
void GaitSelect() { g_Hexapod.GaitSelect(); }
void GaitSeq() { g_Hexapod.GaitSeq(); }
void Gait(byte leg) { g_Hexapod.Gait(leg); }
void BalanceBody() { g_Hexapod.BalanceBody(); }
void CheckAngles() { g_Hexapod.CheckAngles(); }
bool CheckVoltage() { return g_Hexapod.CheckVoltage(); }
void BalCalcOneLeg(short x, short z, short y, byte leg) { g_Hexapod.BalCalcOneLeg(x, z, y, leg); }

void setup(){
    g_fShowDebugPrompt = true;
    g_fDebugOutput = false;
#ifdef DBGSerial    
    DBGSerial.begin(57600);
#endif
    g_ServoDriver.Init();
    if (g_InputController.FIsDiagnosticModeRequested()) g_ServoDriver.SSCForwarder();
    delay(10);
    
    for (LegIndex= 0; LegIndex <= 5; LegIndex++ ) {
        g_Legs[LegIndex].init(LegIndex);
    }
    
    g_InControlState.SelectedLeg = 255;
    g_Hexapod.PrevSelectedLeg = 255;
    g_InControlState.BodyPos.x = g_InControlState.BodyPos.y = g_InControlState.BodyPos.z = 0;
    g_InControlState.BodyRot1.x = g_InControlState.BodyRot1.y = g_InControlState.BodyRot1.z = 0;
    g_Hexapod.BodyRotOffsetX = g_Hexapod.BodyRotOffsetY = g_Hexapod.BodyRotOffsetZ = 0;
    
    g_InControlState.GaitType = 1;
    g_InControlState.BalanceMode = 0;
    g_InControlState.LegLiftHeight = 50;
    g_InControlState.ForceGaitStepCnt = 0;
    g_Hexapod.GaitStep = 1;
    GaitSelect();
    
    g_InputController.Init();
    g_Hexapod.ServoMoveTime = 150;
    g_InControlState.fHexOn = 0;
    g_Hexapod.fLowVoltageShutdown = false;
}

void loop(void)
{
    g_Hexapod.lTimerStart = millis(); 
    CheckVoltage();
    if (!g_Hexapod.fLowVoltageShutdown) g_InputController.ControlInput();
    WriteOutputs();

#ifdef OPT_GPPLAYER
    g_ServoDriver.GPPlayer();
#endif

    SingleLegControl();
    GaitSeq();
             
    g_Hexapod.TotalTransX = g_Hexapod.TotalTransZ = g_Hexapod.TotalTransY = 0;
    g_Hexapod.TotalXBal1 = g_Hexapod.TotalYBal1 = g_Hexapod.TotalZBal1 = 0;

    if (g_InControlState.BalanceMode) {
        for (LegIndex = 0; LegIndex <= 2; LegIndex++) {
            BalCalcOneLeg (-g_Legs[LegIndex].posX+g_Legs[LegIndex].gaitPosX, 
                        g_Legs[LegIndex].posZ+g_Legs[LegIndex].gaitPosZ, 
                        (g_Legs[LegIndex].posY-(short)pgm_read_word(&cInitPosY[LegIndex]))+g_Legs[LegIndex].gaitPosY, LegIndex);
        }
        for (LegIndex = 3; LegIndex <= 5; LegIndex++) {
            BalCalcOneLeg(g_Legs[LegIndex].posX+g_Legs[LegIndex].gaitPosX, 
                        g_Legs[LegIndex].posZ+g_Legs[LegIndex].gaitPosZ, 
                        (g_Legs[LegIndex].posY-(short)pgm_read_word(&cInitPosY[LegIndex]))+g_Legs[LegIndex].gaitPosY, LegIndex);
        }
        BalanceBody();
    }
          
     g_Hexapod.IKSolution = g_Hexapod.IKSolutionWarning = g_Hexapod.IKSolutionError = 0;
            
     for (LegIndex = 0; LegIndex <=2; LegIndex++) {    
        g_Legs[LegIndex].calculateBodyFK(-g_Legs[LegIndex].posX+g_InControlState.BodyPos.x+g_Legs[LegIndex].gaitPosX - g_Hexapod.TotalTransX,
                g_Legs[LegIndex].posY+g_InControlState.BodyPos.y+g_Legs[LegIndex].gaitPosY - g_Hexapod.TotalTransY,
                g_Legs[LegIndex].posZ+g_InControlState.BodyPos.z+g_Legs[LegIndex].gaitPosZ - g_Hexapod.TotalTransZ,
                g_Legs[LegIndex].gaitRotY, g_Hexapod.BodyRotOffsetX, g_Hexapod.BodyRotOffsetY, g_Hexapod.BodyRotOffsetZ, g_Hexapod.TotalXBal1, g_Hexapod.TotalYBal1, g_Hexapod.TotalZBal1);

        g_Legs[LegIndex].calculateLegIK(g_Legs[LegIndex].posX-g_InControlState.BodyPos.x+g_Legs[LegIndex].bodyFKPosX-(g_Legs[LegIndex].gaitPosX - g_Hexapod.TotalTransX), 
                g_Legs[LegIndex].posY+g_InControlState.BodyPos.y-g_Legs[LegIndex].bodyFKPosY+g_Legs[LegIndex].gaitPosY - g_Hexapod.TotalTransY,
                g_Legs[LegIndex].posZ+g_InControlState.BodyPos.z-g_Legs[LegIndex].bodyFKPosZ+g_Legs[LegIndex].gaitPosZ - g_Hexapod.TotalTransZ);
     }

     for (LegIndex = 3; LegIndex <=5; LegIndex++) {
        g_Legs[LegIndex].calculateBodyFK(g_Legs[LegIndex].posX-g_InControlState.BodyPos.x+g_Legs[LegIndex].gaitPosX - g_Hexapod.TotalTransX,
                g_Legs[LegIndex].posY+g_InControlState.BodyPos.y+g_Legs[LegIndex].gaitPosY - g_Hexapod.TotalTransY,
                g_Legs[LegIndex].posZ+g_InControlState.BodyPos.z+g_Legs[LegIndex].gaitPosZ - g_Hexapod.TotalTransZ,
                g_Legs[LegIndex].gaitRotY, g_Hexapod.BodyRotOffsetX, g_Hexapod.BodyRotOffsetY, g_Hexapod.BodyRotOffsetZ, g_Hexapod.TotalXBal1, g_Hexapod.TotalYBal1, g_Hexapod.TotalZBal1);
        g_Legs[LegIndex].calculateLegIK(g_Legs[LegIndex].posX+g_InControlState.BodyPos.x-g_Legs[LegIndex].bodyFKPosX+g_Legs[LegIndex].gaitPosX - g_Hexapod.TotalTransX,
                g_Legs[LegIndex].posY+g_InControlState.BodyPos.y-g_Legs[LegIndex].bodyFKPosY+g_Legs[LegIndex].gaitPosY - g_Hexapod.TotalTransY,
                g_Legs[LegIndex].posZ+g_InControlState.BodyPos.z-g_Legs[LegIndex].bodyFKPosZ+g_Legs[LegIndex].gaitPosZ - g_Hexapod.TotalTransZ);
     }
    
    CheckAngles();
    // g_Hexapod.LedC = g_Hexapod.IKSolutionWarning; g_Hexapod.LedA = g_Hexapod.IKSolutionError;
            
    if (g_InControlState.fHexOn) {
        if (!g_InControlState.fPrev_HexOn) {
            MSound(3, 60, 2000, 80, 2250, 100, 2500);
            g_Hexapod.Eyes = 1;
        }
        
        if ((abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.y*2)>cTravelDeadZone)) {         
            g_Hexapod.ServoMoveTime = g_Hexapod.NomGaitSpeed + (g_InControlState.InputTimeDelay*2) + g_InControlState.SpeedControl;
            if (g_InControlState.BalanceMode) g_Hexapod.ServoMoveTime += 100;
        } else g_Hexapod.ServoMoveTime = 200 + g_InControlState.SpeedControl;
        
        StartUpdateServos();
        g_Hexapod.fContinueWalking = false;
            
        for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
            if ( (abs(g_Legs[LegIndex].gaitPosX) > 2) || (abs(g_Legs[LegIndex].gaitPosY) > 2) || (abs(g_Legs[LegIndex].gaitPosZ) > 2) || (abs(g_Legs[LegIndex].gaitRotY) > 2) ) {
                g_Hexapod.fContinueWalking = true;
                break;
            }
        }

        if (g_Hexapod.fWalking || g_Hexapod.fContinueWalking) {
            g_Hexapod.fWalking = g_Hexapod.fContinueWalking;
            g_Hexapod.lTimerEnd = millis();
            if (g_Hexapod.lTimerEnd > g_Hexapod.lTimerStart) g_Hexapod.CycleTime = g_Hexapod.lTimerEnd-g_Hexapod.lTimerStart;
            else g_Hexapod.CycleTime = 0xffffffffL - g_Hexapod.lTimerEnd + g_Hexapod.lTimerStart + 1;
            delay(min(max ((g_Hexapod.PrevServoMoveTime - g_Hexapod.CycleTime), 1), g_Hexapod.NomGaitSpeed)); 
        }
        delay(20); 
        
    } else {
        if (g_InControlState.fPrev_HexOn || (g_Hexapod.AllDown == 0)) {
            g_Hexapod.ServoMoveTime = 600;
            StartUpdateServos();
            g_ServoDriver.CommitServoDriver(g_Hexapod.ServoMoveTime);
            MSound(3, 100, 2500, 80, 2250, 60, 2000);
            delay(600);
        } else {
            g_ServoDriver.FreeServos();
            g_Hexapod.Eyes = 0;
        }

#ifdef OPT_TERMINAL_MONITOR  
        TerminalMonitor();
#endif
        delay(20);
    }

    g_ServoDriver.CommitServoDriver(g_Hexapod.ServoMoveTime);
    g_Hexapod.PrevServoMoveTime = g_Hexapod.ServoMoveTime;
    g_InControlState.fPrev_HexOn = g_InControlState.fHexOn;
}

void StartUpdateServos() {        
    g_ServoDriver.BeginServoUpdate();
    for (byte i = 0; i <= 5; i++) {
#ifdef c4DOF
        g_ServoDriver.OutputServoInfoForLeg(i, g_Legs[i].coxaAngle, g_Legs[i].femurAngle, g_Legs[i].tibiaAngle, g_Legs[i].tarsAngle);
#else
        g_ServoDriver.OutputServoInfoForLeg(i, g_Legs[i].coxaAngle, g_Legs[i].femurAngle, g_Legs[i].tibiaAngle);
#endif      
    }
}

void WriteOutputs() {
#ifdef cEyesPin
    digitalWrite(cEyesPin, g_Hexapod.Eyes);
#endif        
}

void SingleLegControl() {
    g_Hexapod.AllDown = true;
    for(byte i=0; i<6; i++) if(g_Legs[i].posY != (short)pgm_read_word(&cInitPosY[i])) g_Hexapod.AllDown = false;

    if (g_InControlState.SelectedLeg <= 5) {
        if (g_InControlState.SelectedLeg != g_Hexapod.PrevSelectedLeg) {
            if (g_Hexapod.AllDown) {
                g_Legs[g_InControlState.SelectedLeg].posY = (short)pgm_read_word(&cInitPosY[g_InControlState.SelectedLeg])-20;
                g_Hexapod.PrevSelectedLeg = g_InControlState.SelectedLeg;
            } else {
                g_Legs[g_Hexapod.PrevSelectedLeg].posX = (short)pgm_read_word(&cInitPosX[g_Hexapod.PrevSelectedLeg]);
                g_Legs[g_Hexapod.PrevSelectedLeg].posY = (short)pgm_read_word(&cInitPosY[g_Hexapod.PrevSelectedLeg]);
                g_Legs[g_Hexapod.PrevSelectedLeg].posZ = (short)pgm_read_word(&cInitPosZ[g_Hexapod.PrevSelectedLeg]);
            }
        } else if (!g_InControlState.fSLHold) {
            g_Legs[g_InControlState.SelectedLeg].posY += g_InControlState.SLLeg.y;
            g_Legs[g_InControlState.SelectedLeg].posX = (short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.x;
            g_Legs[g_InControlState.SelectedLeg].posZ = (short)pgm_read_word(&cInitPosZ[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.z;
        }
    } else {
        if (!g_Hexapod.AllDown) {
            for(byte i=0; i<=5; i++) {
                g_Legs[i].posX = (short)pgm_read_word(&cInitPosX[i]);
                g_Legs[i].posY = (short)pgm_read_word(&cInitPosY[i]);
                g_Legs[i].posZ = (short)pgm_read_word(&cInitPosZ[i]);
            }
        }
        g_Hexapod.PrevSelectedLeg = 255;
    }
}

void SoundNoTimer(uint8_t _pin, unsigned long duration,  unsigned int frequency) {
#ifdef __AVR__
    volatile uint8_t *pin_port; uint8_t pin_mask;
#else
    volatile uint32_t *pin_port; uint16_t pin_mask;
#endif
    pinMode(_pin, OUTPUT);
    pin_port = portOutputRegister(digitalPinToPort(_pin));
    pin_mask = digitalPinToBitMask(_pin);
    long toggle_count = 2 * frequency * duration / 1000;
    long lusDelayPerHalfCycle = 1000000L/(frequency * 2);
    while (toggle_count--) { *pin_port ^= pin_mask; delayMicroseconds(lusDelayPerHalfCycle); }
    *pin_port &= ~(pin_mask);
}

void MSound(byte cNotes, ...) {
    va_list ap; va_start(ap, cNotes);
    while (cNotes > 0) {
        unsigned int duration = va_arg(ap, unsigned int);
        unsigned int frequency = va_arg(ap, unsigned int);
        SoundNoTimer(SOUND_PIN, duration, frequency);
        cNotes--;
    }
    va_end(ap);
}

#ifdef OPT_TERMINAL_MONITOR
boolean TerminalMonitor(void) {
    byte szCmdLine[5]; int ich; int ch;
    if (g_fShowDebugPrompt) {
        DBGSerial.println("Arduino Phoenix Monitor");
        DBGSerial.println("D - Toggle debug on or off");
        g_fShowDebugPrompt = false;
    }
    if (ich = DBGSerial.available()) {
        ich = 0;
        for (ich=0; ich < sizeof(szCmdLine); ich++) {
            ch = DBGSerial.read(); if ((ch == -1) || ((ch >= 10) && (ch <= 15))) break;
            szCmdLine[ich] = ch;
        }
        szCmdLine[ich] = '\0';
        if (ich == 0) g_fShowDebugPrompt = true;
        else if ((ich == 1) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
            g_fDebugOutput = !g_fDebugOutput;
        }
        return true;
    }
    return false;
}
#endif

short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider) {
    if (g_Hexapod.fWalking) {
        if (CtrlMoveOut < (CtrlMoveInp - 4)) return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
        else if (CtrlMoveOut > (CtrlMoveInp + 4)) return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
    }
    return CtrlMoveInp;
}

uint8_t g_iLegInitIndex = 0x00;
void AdjustLegPositionsToBodyHeight(void) {
#ifdef CNT_HEX_INITS
    if (g_InControlState.BodyPos.y > (short)pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS-1]))
      g_InControlState.BodyPos.y =  (short)pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS-1]);
    uint8_t i; word XZLength1;
    for(i = 0; i < CNT_HEX_INITS; i++) {
      if (g_InControlState.BodyPos.y <= (short)pgm_read_byte(&g_abHexMaxBodyY[i])) {
        XZLength1 = pgm_read_byte(&g_abHexIntXZ[i]); break;
      }
    }
    if (i != g_iLegInitIndex) { 
       g_iLegInitIndex = i;
       for (uint8_t li = 0; li <= 5; li++) {
           g_Hexapod.GetSinCos((short)pgm_read_word(&cCoxaAngle1[li]));
           g_Legs[li].posX = ((long)((long)g_Hexapod.cos4 * XZLength1))/c4DEC;
           g_Legs[li].posZ = -((long)((long)g_Hexapod.sin4 * XZLength1))/c4DEC;
       }
       g_InControlState.ForceGaitStepCnt = g_Hexapod.StepsInGait;
    }
#endif
}
