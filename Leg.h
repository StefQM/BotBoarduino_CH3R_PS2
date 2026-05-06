#ifndef LEG_H
#define LEG_H

#if ARDUINO > 99
#include <Arduino.h>
#else
#include "Arduino.h"
#endif

#include "Hex_Cfg.h"

class Leg {
public:
    // State
    short posX, posY, posZ;
    short coxaAngle, femurAngle, tibiaAngle;
#ifdef c4DOF
    short tarsAngle;
#endif

    short gaitPosX, gaitPosY, gaitPosZ, gaitRotY;
    long bodyFKPosX, bodyFKPosY, bodyFKPosZ;

    byte index;

    Leg() {}
    void init(byte legIndex);
    
    void calculateBodyFK(short tx, short ty, short tz, short rotationY, 
                        short bodyRotOffsetX, short bodyRotOffsetY, short bodyRotOffsetZ,
                        short totalXBal, short totalYBal, short totalZBal);
    void calculateLegIK(short tx, short ty, short tz);
};

#endif
