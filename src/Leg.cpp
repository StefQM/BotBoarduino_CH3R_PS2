#include "Leg.h"
#include "Hex_Globals.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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
    float SinA, CosA, SinB, CosB, SinG, CosG;
    float CPR_X, CPR_Y, CPR_Z;

    //Calculating totals from center of the body to the feet
    CPR_X = (float)((short)cOffsetX[index] + tx + bodyRotOffsetX);
    CPR_Y = (float)(ty + bodyRotOffsetY);
    CPR_Z = (float)((short)cOffsetZ[index] + tz + bodyRotOffsetZ);

    float angleG = (float)(g_InControlState.BodyRot1.x + totalXBal) * M_PI / 1800.0f;
    SinG = sinf(angleG); CosG = cosf(angleG);

    float angleB = (float)(g_InControlState.BodyRot1.z + totalZBal) * M_PI / 1800.0f;
    SinB = sinf(angleB); CosB = cosf(angleB);

    float angleA = (float)(g_InControlState.BodyRot1.y + (rotationY * 10) + totalYBal) * M_PI / 1800.0f;
    SinA = sinf(angleA); CosA = cosf(angleA);

    bodyFKPosX = (short)(CPR_X - (CPR_X * CosA * CosB - CPR_Z * CosB * SinA + CPR_Y * SinB));
    bodyFKPosZ = (short)(CPR_Z - (CPR_X * CosG * SinA + CPR_X * CosA * SinB * SinG + CPR_Z * CosA * CosG - CPR_Z * SinA * SinB * SinG - CPR_Y * CosB * SinG));
    bodyFKPosY = (short)(CPR_Y - (CPR_X * SinA * SinG - CPR_X * CosA * CosG * SinB + CPR_Z * CosA * SinG + CPR_Z * CosG * SinA * SinB + CPR_Y * CosB * CosG));
}

void Leg::calculateLegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ) {
    float IKSW, IKA1, IKA2;
    float IKFeetPosXZ;
    float TarsOffsetXZ = 0, TarsOffsetY = 0;
    short TarsToGroundAngle1 = 0;

    // 1. Coxa Angle (Horizontal)
    IKFeetPosXZ = sqrtf((float)IKFeetPosX * IKFeetPosX + (float)IKFeetPosZ * IKFeetPosZ);
    float Atan4 = acosf((float)IKFeetPosX / IKFeetPosXZ);
    if (IKFeetPosZ < 0) Atan4 = -Atan4;
    coxaAngle = (short)(Atan4 * 1800.0f / M_PI) + (short)cCoxaAngle1[index];

#ifdef c4DOF
    if ((uint8_t)cTarsLength[index]) {
        TarsToGroundAngle1 = -cTarsConst + cTarsMulti*IKFeetPosY + ((long)(IKFeetPosXZ*cTarsFactorA))/10 - ((long)(IKFeetPosXZ*IKFeetPosY)/(cTarsFactorB));
        if (IKFeetPosY < 0) TarsToGroundAngle1 -= ((long)(IKFeetPosY*cTarsFactorC)/10);
        short TGA_B_H3 = (TarsToGroundAngle1 > 400) ? 200 + (TarsToGroundAngle1/2) : TarsToGroundAngle1;
        short TGA_A_H4 = (TarsToGroundAngle1 > 300) ? 240 + (TarsToGroundAngle1/5) : TarsToGroundAngle1;
        if (IKFeetPosY > 0) TarsToGroundAngle1 = TGA_A_H4;
        else if (IKFeetPosY > -10) TarsToGroundAngle1 = (TGA_A_H4 -(((long)IKFeetPosY*(TGA_B_H3-TGA_A_H4))/10));
        else TarsToGroundAngle1 = TGA_B_H3;

        float tarsAngleRad = (float)TarsToGroundAngle1 * M_PI / 1800.0f;
        TarsOffsetXZ = sinf(tarsAngleRad) * (uint8_t)cTarsLength[index];
        TarsOffsetY = cosf(tarsAngleRad) * (uint8_t)cTarsLength[index];
    }
#endif

    // 2. Femur and Tibia (Vertical)
    float vert = (float)IKFeetPosY - TarsOffsetY;
    float horz = IKFeetPosXZ - (uint8_t)cCoxaLength[index] - TarsOffsetXZ;
    IKSW = sqrtf(vert*vert + horz*horz);
    
    // IKA1 is angle to the foot from the vertical
    IKA1 = acosf(vert / IKSW);
    if (horz < 0) IKA1 = -IKA1;

    float femurLen = (float)(uint8_t)cFemurLength[index];
    float tibiaLen = (float)(uint8_t)cTibiaLength[index];

    // IKA2 is interior angle at the femur
    float cosIKA2 = (femurLen*femurLen - tibiaLen*tibiaLen + IKSW*IKSW) / (2.0f * femurLen * IKSW);
    cosIKA2 = fminf(fmaxf(cosIKA2, -1.0f), 1.0f);
    IKA2 = acosf(cosIKA2);
    
    femurAngle = (short)(-(IKA1 + IKA2) * 1800.0f / M_PI) + 900 + CFEMURHORNOFFSET1(index);

    // Tibia interior angle
    float cosTibia = (femurLen*femurLen + tibiaLen*tibiaLen - IKSW*IKSW) / (2.0f * femurLen * tibiaLen);
    cosTibia = fminf(fmaxf(cosTibia, -1.0f), 1.0f);
    tibiaAngle = -(900 - (short)(acosf(cosTibia) * 1800.0f / M_PI));

#ifdef c4DOF
    if ((uint8_t)cTarsLength[index]) tarsAngle = (TarsToGroundAngle1 + femurAngle - tibiaAngle) + CTARSHORNOFFSET1(index);
#endif

    if(IKSW < (femurLen + tibiaLen - 30.0f)) g_Hexapod.IKSolution = 1;
    else if(IKSW < (femurLen + tibiaLen)) g_Hexapod.IKSolutionWarning = 1;
    else g_Hexapod.IKSolutionError = 1;
}

