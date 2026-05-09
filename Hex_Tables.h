#ifndef HEX_TABLES_H
#define HEX_TABLES_H

#include <Arduino.h>

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

#endif
