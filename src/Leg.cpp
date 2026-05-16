#include "Leg.h"
#include "Hex_Globals.h"

// Macro equivalents for LegIK
#ifdef c4DOF
#define CTARSHORNOFFSET1(LEGI) ((short)cTarsHornOffset1[LEGI])
#else
#define CTARSHORNOFFSET1(LEGI) 0
#endif
#define CFEMURHORNOFFSET1(LEGI) ((short)cFemurHornOffset1[LEGI])


void Leg::init(uint8_t legIndex) {
    index = legIndex;
    posX = (short)cInitPosX[index];
    posY = (short)cInitPosY[index];
    posZ = (short)cInitPosZ[index];
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
    CPR_X = (short)cOffsetX[index] + tx + bodyRotOffsetX;
    CPR_Y = ty + bodyRotOffsetY;
    CPR_Z = (short)cOffsetZ[index] + tz + bodyRotOffsetZ;

    g_Hexapod.GetSinCos (g_InControlState.BodyRot1.x + totalXBal);
    SinG4 = g_Hexapod.sin4; CosG4 = g_Hexapod.cos4;

    g_Hexapod.GetSinCos (g_InControlState.BodyRot1.z + totalZBal);
    SinB4 = g_Hexapod.sin4; CosB4 = g_Hexapod.cos4;

    g_Hexapod.GetSinCos (g_InControlState.BodyRot1.y + (rotationY * c1DEC) + totalYBal) ;
    SinA4 = g_Hexapod.sin4; CosA4 = g_Hexapod.cos4;

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

    g_Hexapod.GetATan2 (IKFeetPosX, IKFeetPosZ);
    coxaAngle = (((long)g_Hexapod.Atan4*180) / 3141) + (short)cCoxaAngle1[index];
    IKFeetPosXZ = g_Hexapod.XYhyp2/c2DEC;

#ifdef c4DOF
    if ((uint8_t)cTarsLength[index]) {
        TarsToGroundAngle1 = -cTarsConst + cTarsMulti*IKFeetPosY + ((long)(IKFeetPosXZ*cTarsFactorA))/c1DEC - ((long)(IKFeetPosXZ*IKFeetPosY)/(cTarsFactorB));
        if (IKFeetPosY < 0) TarsToGroundAngle1 -= ((long)(IKFeetPosY*cTarsFactorC)/c1DEC);
        TGA_B_H3 = (TarsToGroundAngle1 > 400) ? 200 + (TarsToGroundAngle1/2) : TarsToGroundAngle1;
        TGA_A_H4 = (TarsToGroundAngle1 > 300) ? 240 + (TarsToGroundAngle1/5) : TarsToGroundAngle1;
        if (IKFeetPosY > 0) TarsToGroundAngle1 = TGA_A_H4;
        else if (IKFeetPosY > -10) TarsToGroundAngle1 = (TGA_A_H4 -(((long)IKFeetPosY*(TGA_B_H3-TGA_A_H4))/c1DEC));
        else TarsToGroundAngle1 = TGA_B_H3;
        g_Hexapod.GetSinCos(TarsToGroundAngle1);
        TarsOffsetXZ = ((long)g_Hexapod.sin4*(uint8_t)cTarsLength[index])/c4DEC;
        TarsOffsetY = ((long)g_Hexapod.cos4*(uint8_t)cTarsLength[index])/c4DEC;
    } else { TarsOffsetXZ = 0; TarsOffsetY = 0; }
#endif

    IKA14 = g_Hexapod.GetATan2 (IKFeetPosY-TarsOffsetY, IKFeetPosXZ-(uint8_t)cCoxaLength[index]-TarsOffsetXZ);
    IKSW2 = g_Hexapod.XYhyp2;
    Temp1 = ((((long)(uint8_t)cFemurLength[index]*(uint8_t)cFemurLength[index]) - ((long)(uint8_t)cTibiaLength[index]*(uint8_t)cTibiaLength[index]))*c4DEC + ((long)IKSW2*IKSW2));
    Temp2 = (long)(2*(uint8_t)cFemurLength[index])*c2DEC * (unsigned long)IKSW2;
    T3 = Temp1 / (Temp2/c4DEC);
    IKA24 = g_Hexapod.GetArcCos (T3 );
    femurAngle = -(long)(IKA14 + IKA24) * 180 / 3141 + 900 + CFEMURHORNOFFSET1(index);
    Temp1 = ((((long)(uint8_t)cFemurLength[index]*(uint8_t)cFemurLength[index]) + ((long)(uint8_t)cTibiaLength[index]*(uint8_t)cTibiaLength[index]))*c4DEC - ((long)IKSW2*IKSW2));
    Temp2 = (2*(uint8_t)cFemurLength[index]*(uint8_t)cTibiaLength[index]);
    g_Hexapod.GetArcCos (Temp1 / Temp2);
    tibiaAngle = -(900-(long)g_Hexapod.AngleRad4*180/3141);
#ifdef c4DOF
    if ((uint8_t)cTarsLength[index]) tarsAngle = (TarsToGroundAngle1 + femurAngle - tibiaAngle) + CTARSHORNOFFSET1(index);
#endif
    if(IKSW2 < ((uint8_t)cFemurLength[index]+(uint8_t)cTibiaLength[index]-30)*c2DEC) g_Hexapod.IKSolution = 1;
    else if(IKSW2 < ((uint8_t)cFemurLength[index]+(uint8_t)cTibiaLength[index])*c2DEC) g_Hexapod.IKSolutionWarning = 1;
    else g_Hexapod.IKSolutionError = 1;
}
