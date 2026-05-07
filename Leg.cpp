#include "Leg.h"
#include "Hex_Globals.h"

// External declarations for functions in .ino
extern void GetSinCos(short AngleDeg1);
extern long GetArcCos(short cos4);
extern short GetATan2(short AtanX, short AtanY);
extern short sin4;
extern short cos4;
extern short AngleRad4;
extern short XYhyp2;
extern short Atan4;
extern boolean IKSolution;
extern boolean IKSolutionWarning;
extern boolean IKSolutionError;

// Macro equivalents for LegIK
#ifdef c4DOF
#define CTARSHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTarsHornOffset1[LEGI]))
#else
#define CTARSHORNOFFSET1(LEGI) 0
#endif
#define CFEMURHORNOFFSET1(LEGI) ((short)pgm_read_word(&cFemurHornOffset1[LEGI]))


void Leg::init(byte legIndex) {
    index = legIndex;
    posX = (short)pgm_read_word(&cInitPosX[index]);
    posY = (short)pgm_read_word(&cInitPosY[index]);
    posZ = (short)pgm_read_word(&cInitPosZ[index]);
    gaitPosX = gaitPosY = gaitPosZ = gaitRotY = 0;
    coxaAngle = femurAngle = tibiaAngle = 0;
#ifdef c4DOF
    tarsAngle = 0;
#endif
}

void Leg::calculateBodyFK(short tx, short ty, short tz, short rotationY, 
                        short bodyRotOffsetX, short bodyRotOffsetY, short bodyRotOffsetZ,
                        short totalXBal, short totalYBal, short totalZBal) {
    short            SinA4, CosA4, SinB4, CosB4, SinG4, CosG4;
    short            CPR_X, CPR_Y, CPR_Z;

    //Calculating totals from center of the body to the feet
    CPR_X = (short)pgm_read_word(&cOffsetX[index]) + tx + bodyRotOffsetX;
    CPR_Y = ty + bodyRotOffsetY;
    CPR_Z = (short)pgm_read_word(&cOffsetZ[index]) + tz + bodyRotOffsetZ;

    GetSinCos (g_InControlState.BodyRot1.x + totalXBal);
    SinG4 = sin4; CosG4 = cos4;

    GetSinCos (g_InControlState.BodyRot1.z + totalZBal);
    SinB4 = sin4; CosB4 = cos4;

    GetSinCos (g_InControlState.BodyRot1.y + (rotationY * c1DEC) + totalYBal) ;
    SinA4 = sin4; CosA4 = cos4;

    bodyFKPosX = ((long)CPR_X*c2DEC - ((long)CPR_X*c2DEC*CosA4/c4DEC*CosB4/c4DEC - (long)CPR_Z*c2DEC*CosB4/c4DEC*SinA4/c4DEC 
                + (long)CPR_Y*c2DEC*SinB4/c4DEC ))/c2DEC;
    bodyFKPosZ = ((long)CPR_Z*c2DEC - ( (long)CPR_X*c2DEC*CosG4/c4DEC*SinA4/c4DEC + (long)CPR_X*c2DEC*CosA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC 
                + (long)CPR_Z*c2DEC*CosA4/c4DEC*CosG4/c4DEC - (long)CPR_Z*c2DEC*SinA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC 
                - (long)CPR_Y*c2DEC*CosB4/c4DEC*SinG4/c4DEC ))/c2DEC;
    bodyFKPosY = ((long)CPR_Y  *c2DEC - ( (long)CPR_X*c2DEC*SinA4/c4DEC*SinG4/c4DEC - (long)CPR_X*c2DEC*CosA4/c4DEC*CosG4/c4DEC*SinB4/c4DEC 
                + (long)CPR_Z*c2DEC*CosA4/c4DEC*SinG4/c4DEC + (long)CPR_Z*c2DEC*CosG4/c4DEC*SinA4/c4DEC*SinB4/c4DEC 
                + (long)CPR_Y*c2DEC*CosB4/c4DEC*CosG4/c4DEC ))/c2DEC;
}

void Leg::calculateLegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ) {
    unsigned long   IKSW2, IKA14, IKA24;
    short           IKFeetPosXZ;
#ifdef c4DOF
    long            TarsOffsetXZ, TarsOffsetY, TarsToGroundAngle1, TGA_A_H4, TGA_B_H3;
#else
    long TarsOffsetXZ = 0, TarsOffsetY = 0;
#endif
    long            Temp1, Temp2, T3;

    GetATan2 (IKFeetPosX, IKFeetPosZ);
    coxaAngle = (((long)Atan4*180) / 3141) + (short)pgm_read_word(&cCoxaAngle1[index]);
    IKFeetPosXZ = XYhyp2/c2DEC;

#ifdef c4DOF
    if ((byte)pgm_read_byte(&cTarsLength[index])) {
        TarsToGroundAngle1 = -cTarsConst + cTarsMulti*IKFeetPosY + ((long)(IKFeetPosXZ*cTarsFactorA))/c1DEC - ((long)(IKFeetPosXZ*IKFeetPosY)/(cTarsFactorB));
        if (IKFeetPosY < 0) TarsToGroundAngle1 -= ((long)(IKFeetPosY*cTarsFactorC)/c1DEC);
        TGA_B_H3 = (TarsToGroundAngle1 > 400) ? 200 + (TarsToGroundAngle1/2) : TarsToGroundAngle1;
        TGA_A_H4 = (TarsToGroundAngle1 > 300) ? 240 + (TarsToGroundAngle1/5) : TarsToGroundAngle1;
        if (IKFeetPosY > 0) TarsToGroundAngle1 = TGA_A_H4;
        else if (IKFeetPosY > -10) TarsToGroundAngle1 = (TGA_A_H4 -(((long)IKFeetPosY*(TGA_B_H3-TGA_A_H4))/c1DEC));
        else TarsToGroundAngle1 = TGA_B_H3;
        GetSinCos(TarsToGroundAngle1);
        TarsOffsetXZ = ((long)sin4*(byte)pgm_read_byte(&cTarsLength[index]))/c4DEC;
        TarsOffsetY = ((long)cos4*(byte)pgm_read_byte(&cTarsLength[index]))/c4DEC;
    } else { TarsOffsetXZ = 0; TarsOffsetY = 0; }
#endif

    IKA14 = GetATan2 (IKFeetPosY-TarsOffsetY, IKFeetPosXZ-(byte)pgm_read_byte(&cCoxaLength[index])-TarsOffsetXZ);
    IKSW2 = XYhyp2;
    Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[index])*(byte)pgm_read_byte(&cFemurLength[index])) - ((long)(byte)pgm_read_byte(&cTibiaLength[index])*(byte)pgm_read_byte(&cTibiaLength[index])))*c4DEC + ((long)IKSW2*IKSW2));
    Temp2 = (long)(2*(byte)pgm_read_byte(&cFemurLength[index]))*c2DEC * (unsigned long)IKSW2;
    T3 = Temp1 / (Temp2/c4DEC);
    IKA24 = GetArcCos (T3 );
    femurAngle = -(long)(IKA14 + IKA24) * 180 / 3141 + 900 + CFEMURHORNOFFSET1(index);
    Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[index])*(byte)pgm_read_byte(&cFemurLength[index])) + ((long)(byte)pgm_read_byte(&cTibiaLength[index])*(byte)pgm_read_byte(&cTibiaLength[index])))*c4DEC - ((long)IKSW2*IKSW2));
    Temp2 = (2*(byte)pgm_read_byte(&cFemurLength[index])*(byte)pgm_read_byte(&cTibiaLength[index]));
    GetArcCos (Temp1 / Temp2);
    tibiaAngle = -(900-(long)AngleRad4*180/3141);
#ifdef c4DOF
    if ((byte)pgm_read_byte(&cTarsLength[index])) tarsAngle = (TarsToGroundAngle1 + femurAngle - tibiaAngle) + CTARSHORNOFFSET1(index);
#endif
    if(IKSW2 < ((byte)pgm_read_byte(&cFemurLength[index])+(byte)pgm_read_byte(&cTibiaLength[index])-30)*c2DEC) IKSolution = 1;
    else if(IKSW2 < ((byte)pgm_read_byte(&cFemurLength[index])+(byte)pgm_read_byte(&cTibiaLength[index]))*c2DEC) IKSolutionWarning = 1;
    else IKSolutionError = 1;
}
