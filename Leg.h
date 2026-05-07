#ifndef LEG_H
#define LEG_H

#if ARDUINO > 99
#include <Arduino.h>
#else
#include "Arduino.h"
#endif

#include "Hex_Cfg.h"

// --- PROGMEM Tables (Defined in .ino, used in .cpp) ---
// These declarations are required for Arduino/AVR cross-file linking.
extern const short cOffsetX[] PROGMEM;
extern const short cOffsetZ[] PROGMEM;
extern const short cCoxaAngle1[] PROGMEM;
extern const byte cCoxaLength[] PROGMEM;
extern const byte cFemurLength[] PROGMEM;
extern const byte cTibiaLength[] PROGMEM;
extern const short cInitPosX[] PROGMEM;
extern const short cInitPosY[] PROGMEM;
extern const short cInitPosZ[] PROGMEM;
extern const short cCoxaMin1[] PROGMEM;
extern const short cCoxaMax1[] PROGMEM;
extern const short cFemurMin1[] PROGMEM;
extern const short cFemurMax1[] PROGMEM;
extern const short cTibiaMin1[] PROGMEM;
extern const short cTibiaMax1[] PROGMEM;
extern const short cFemurHornOffset1[] PROGMEM;

#ifdef c4DOF
extern const byte cTarsLength[] PROGMEM;
extern const short cTarsMin1[] PROGMEM;
extern const short cTarsMax1[] PROGMEM;
extern const short cTarsHornOffset1[] PROGMEM;
#endif

class Leg {
public:
    // --- State Variables ---
    short posX, posY, posZ;
    short coxaAngle, femurAngle, tibiaAngle;
#ifdef c4DOF
    short tarsAngle;
#endif

    short gaitPosX, gaitPosY, gaitPosZ, gaitRotY;
    long bodyFKPosX, bodyFKPosY, bodyFKPosZ;

    byte index;

    // --- Methods ---
    Leg() {}
    void init(byte legIndex);
    
    void calculateBodyFK(short tx, short ty, short tz, short rotationY, 
                        short bodyRotOffsetX, short bodyRotOffsetY, short bodyRotOffsetZ,
                        short totalXBal, short totalYBal, short totalZBal);
    void calculateLegIK(short tx, short ty, short tz);
};

#endif
