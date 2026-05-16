//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use the SSC-32 to control
// the servos.
//====================================================================
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include "Hex_Globals.h"
#include "ServoDriver.h"
#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif

#ifdef USE_SSC32

//Servo Pin numbers - May be SSC-32 or actual pins on main controller, depending on configuration.
const uint8_t cCoxaPin[] = {cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin};
const uint8_t cFemurPin[] = {cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin};
const uint8_t cTibiaPin[] = {cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin};
#ifdef c4DOF
const uint8_t cTarsPin[] = {cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin};
#endif



// Add support for running on non-mega Arduino boards as well.
#ifndef ARDUINO_ARCH_STM32
#if defined(__AVR__)
#if !defined(UBRR1H)
#if cSSC_IN == 0
#define SSCSerial Serial
#else
SoftwareSerial SSCSerial(cSSC_IN, cSSC_OUT);
#endif    
#endif
#endif
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================

// definition of some helper functions
extern int SSCRead (uint8_t* pb, int cb, uint16_t wTimeout, uint16_t wEOL);

#ifdef OPT_GPPLAYER
static bool _fGPEnabled;     // IS GP defined for this servo driver?
static bool _fGPActive;      // Is a sequence currently active
static uint8_t _iSeq;           // current sequence we are running
#endif


//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
    SSCSerial.begin(cSSC_BAUD);

    // Lets do the check for GP Enabled here...
#ifdef OPT_GPPLAYER
    char abVer[40];        // give a nice large buffer.
    uint8_t cbRead;

    _fGPEnabled = false;  // starts off assuming that it is not enabled...
    _fGPActive = false;

#ifdef __AVR__
#if not defined(UBRR1H)
#if cSSC_IN != 0
    SSCSerial.listen();
#endif    
#endif    
#endif
    SSCSerial.print("ver\r");
    cbRead = SSCRead((uint8_t*)abVer, sizeof(abVer), 10000, (uint16_t)13);

#ifdef DBGSerial
    DBGSerial.write("Check GP Enable: ");
    if (cbRead > 0) {
        uint8_t iT;
        for (iT = 0; iT < cbRead; iT++)
            DBGSerial.print(abVer[iT], HEX);
        DBGSerial.write((uint8_t*)abVer, cbRead);
    }
    DBGSerial.print("\n\r");
#endif        
    if ((cbRead > 3) && (abVer[cbRead-3]=='G') && (abVer[cbRead-2]=='P') && (abVer[cbRead-1]==13))
        _fGPEnabled = true;  // starts off assuming that it is not enabled...
    else
        MSound(2, 40, 2500, 40, 2500);
#endif
}

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
#ifdef OPT_GPPLAYER

bool ServoDriver::FIsGPEnabled(void) {
    return _fGPEnabled;
}

bool ServoDriver::FIsGPSeqActive(void) {
    return _fGPActive;
}

//--------------------------------------------------------------------
//[FIsGPSeqDefined]
//--------------------------------------------------------------------
bool ServoDriver::FIsGPSeqDefined(uint8_t iSeq)
{
    word wGPSeqPtr;

    // See if we can see if this sequence is defined
    SSCSerial.print("EER -");
    SSCSerial.print(iSeq*2, DEC);
    SSCSerial.println(";2");
    if ((SSCRead((uint8_t*)&wGPSeqPtr, sizeof(wGPSeqPtr), 1000, (uint16_t)0xffff) == sizeof(wGPSeqPtr)) && (wGPSeqPtr != 0)  && (wGPSeqPtr != 0xffff)) {
        return true;
    }
    return false;  // nope return error
}


//--------------------------------------------------------------------
// Setup to start sequence number...
//--------------------------------------------------------------------
void ServoDriver::GPStartSeq(uint8_t iSeq)
{
    _fGPActive = true;
    _iSeq = iSeq;
}

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
void ServoDriver::GPPlayer(void)
{
    uint8_t abStat[4];
    uint8_t cbRead;

    // BUGBUG:: Should integrate in the newer stuff to all us to control the speed of a sequence.
    //Start sequence
    if (_fGPActive) {
        g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

        SSCSerial.print("PL0SQ");
        SSCSerial.print(_iSeq, DEC);
        SSCSerial.println("ONCE"); //Start sequence
        delay(20);
        SSCSerial.flush();        // get rid of anything that was previously queued up...

        //Wait for GPPlayer to complete sequence    
        do {
            SSCSerial.print("QPL0\r");
            cbRead = SSCRead((uint8_t*)abStat, sizeof(abStat), 10000, (uint16_t)0xffff);  //    [GPStatSeq, GPStatFromStep, GPStatToStep, GPStatTime]
            delay(20);
        }
        while ((cbRead == sizeof(abStat)) && ((abStat[0]!=255) || (abStat[1]!=0) || (abStat[2]!=0) || (abStat[3]!=0)));

        g_InputController.AllowControllerInterrupts(true);    // Ok to process hserial again...

        _fGPActive=false;
    }  
}
#endif // OPT_GPPLAYER

//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void ServoDriver::BeginServoUpdate(void)    // Start the update 
{
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#define cPwmDiv       991  //old 1059;
#define cPFConst      592  //old 650 ; 900*(1000/cPwmDiv)+cPFConst must always be 1500
                           // A PWM/deg factor of 10,09 give cPwmDiv = 991 and cPFConst = 592
                           // For a modified 5645 (to 180 deg travel): cPwmDiv = 1500 and cPFConst = 900.
#ifdef c4DOF
void ServoDriver::OutputServoInfoForLeg(uint8_t LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1)
#else
void ServoDriver::OutputServoInfoForLeg(uint8_t LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
#endif    
{        
    word    wCoxaSSCV;     // Coxa value in SSC units
    word    wFemurSSCV;    //
    word    wTibiaSSCV;    //
#ifdef c4DOF
    word    wTarsSSCV;     //
#endif

    //Update Right Legs
    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    if (LegIndex < 3) {
        wCoxaSSCV = ((long)(-sCoxaAngle1 +900))*1000/cPwmDiv+cPFConst;
        wFemurSSCV = ((long)(-sFemurAngle1+900))*1000/cPwmDiv+cPFConst;
        wTibiaSSCV = ((long)(-sTibiaAngle1+900))*1000/cPwmDiv+cPFConst;
#ifdef c4DOF
        wTarsSSCV = ((long)(-sTarsAngle1+900))*1000/cPwmDiv+cPFConst;
#endif
    } else {
        wCoxaSSCV = ((long)(sCoxaAngle1 +900))*1000/cPwmDiv+cPFConst;
        wFemurSSCV = ((long)((long)(sFemurAngle1+900))*1000/cPwmDiv+cPFConst);
        wTibiaSSCV = ((long)(sTibiaAngle1+900))*1000/cPwmDiv+cPFConst;
#ifdef c4DOF
        wTarsSSCV = ((long)(sTarsAngle1+900))*1000/cPwmDiv+cPFConst;
#endif
    }

#ifdef cSSC_BINARYMODE
    SSCSerial.write(cCoxaPin[LegIndex]  + 0x80);
    SSCSerial.write(wCoxaSSCV >> 8);
    SSCSerial.write(wCoxaSSCV & 0xff);
    SSCSerial.write(cFemurPin[LegIndex] + 0x80);
    SSCSerial.write(wFemurSSCV >> 8);
    SSCSerial.write(wFemurSSCV & 0xff);
    SSCSerial.write(cTibiaPin[LegIndex] + 0x80);
    SSCSerial.write(wTibiaSSCV >> 8);
    SSCSerial.write(wTibiaSSCV & 0xff);
#ifdef c4DOF
    if ((uint8_t)cTarsLength[LegIndex]) {    // We allow mix of 3 and 4 DOF legs...
        SSCSerial.write(cTarsPin[LegIndex] + 0x80);
        SSCSerial.write(wTarsSSCV >> 8);
        SSCSerial.write(wTarsSSCV & 0xff);
    }
#endif
#else
    SSCSerial.print("#");
    SSCSerial.print(cCoxaPin[LegIndex], DEC);
    SSCSerial.print("P");
    SSCSerial.print(wCoxaSSCV, DEC);
    SSCSerial.print("#");
    SSCSerial.print(cFemurPin[LegIndex], DEC);
    SSCSerial.print("P");
    SSCSerial.print(wFemurSSCV, DEC);
    SSCSerial.print("#");
    SSCSerial.print(cTibiaPin[LegIndex], DEC);
    SSCSerial.print("P");
    SSCSerial.print(wTibiaSSCV, DEC);
#ifdef c4DOF
    if ((uint8_t)cTarsLength[LegIndex]) {
        SSCSerial.print("#");
        SSCSerial.print(cTarsPin[LegIndex], DEC);
        SSCSerial.print("P");
        SSCSerial.print(wTarsSSCV, DEC);
    }
#endif
#endif        
    g_InputController.AllowControllerInterrupts(true);    // Ok for hserial again...
}


//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly 
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime)
{
#ifdef cSSC_BINARYMODE
    uint8_t    abOut[3];
#endif

    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

#ifdef cSSC_BINARYMODE
    abOut[0] = 0xA1;
    abOut[1] = wMoveTime >> 8;
    abOut[2] = wMoveTime & 0xff;
    SSCSerial.write(abOut, 3);
#else
    //Send <CR>
    SSCSerial.print("T");
    SSCSerial.println(wMoveTime, DEC);
#endif

    g_InputController.AllowControllerInterrupts(true);    

}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    for (uint8_t LegIndex = 0; LegIndex < 32; LegIndex++) {
        SSCSerial.print("#");
        SSCSerial.print(LegIndex, DEC);
        SSCSerial.print("P0");
    }
    SSCSerial.print("T200\r");
    g_InputController.AllowControllerInterrupts(true);    
}


//==============================================================================
// SSC Forwarder - used to allow things like Lynxterm to talk to the SSC-32 
// through the Arduino...  Will see if it is fast enough...
//==============================================================================
#ifdef OPT_SSC_FORWARDER
void  ServoDriver::SSCForwarder(void) 
{
    MSound(1, 1000, 2000);  //sound SOUND_PIN, [50\4000]
    delay(2000);
	
    int sChar;
    int sPrevChar = 0;
    DBGSerial.println("SSC Forwarder mode - Enter $<cr> to exit");
    
    while(digitalRead(PS2_CMD)) {
        if ((sChar = DBGSerial.read()) != -1) {
            SSCSerial.write(sChar & 0xff);
            if (((sChar == '\n') || (sChar == '\r')) && (sPrevChar == '$'))
                break;    // exit out of the loop
            sPrevChar = sChar;
        }

        if ((sChar = SSCSerial.read()) != -1) {
            DBGSerial.write(sChar & 0xff);
        }
    }
    DBGSerial.println("Exited SSC Forwarder mode");
}
#endif // OPT_SSC_FORWARDER




//==============================================================================
// Quick and dirty helper function to read so many uint8_ts in from the SSC with a timeout and an end of character marker...
//==============================================================================
int SSCRead (uint8_t* pb, int cb, uint16_t wTimeout, uint16_t wEOL)
{
    int ich;
    uint8_t* pbIn = pb;
    unsigned long ulTimeLastChar = micros();
    while (cb) {
        while (!SSCSerial.available()) {
            // check for timeout
            if ((word)(micros()-ulTimeLastChar) > wTimeout) {
                return (int)(pb-pbIn);
            }    
        }
        ich = SSCSerial.read();
        *pb++ = (uint8_t)ich;
        cb--;

        if ((word)ich == wEOL)
            break;    // we matched so get out of here.
        ulTimeLastChar = micros();    // update to say we received something
    }

    return (int)(pb-pbIn);
}

//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos... 
// 		Will use the new servo function to set the actual pwm rate and see
//		how well that works...
//==============================================================================
#ifdef OPT_FIND_SERVO_OFFSETS

void ServoDriver::FindServoOffsets()
{
    // not clean but...
    uint8_t abSSCServoNum[NUMSERVOSPERLEG*6];           // array of servos...
    signed char asOffsets[NUMSERVOSPERLEG*6];        // we have 18 servos to find/set offsets for...
    signed char asOffsetsRead[NUMSERVOSPERLEG*6];    // array for our read in servos...

    static const char *apszLegs[] = {"RR","RM","RF", "LR", "LM", "LF"};  // Leg Order
    static const char *apszLJoints[] = {" Coxa", " Femur", " Tibia", " tArs"}; // which joint on the leg...

    uint8_t szTemp[5];
    uint8_t cbRead;

    int data;
    short sSN ; 			// which servo number
    bool fNew = true;	// is this a new servo to work with?
    bool fExit = false;	// when to exit

    if (CheckVoltage()) {
        // Voltage is low... 
        Serial.println("Low Voltage: fix or hit $ to abort");
        while (CheckVoltage()) {
            if (Serial.read() == '$')  return;
        }
    }

    // Fill in array of SSC-32 servo numbers    
    for (sSN=0; sSN < 6; sSN++) {   // Make sure all of our servos initialize to 0 offset from saved.
        abSSCServoNum[sSN*NUMSERVOSPERLEG + 0] = cCoxaPin[sSN];
        abSSCServoNum[sSN*NUMSERVOSPERLEG + 1] = cFemurPin[sSN];
        abSSCServoNum[sSN*NUMSERVOSPERLEG + 2] = cTibiaPin[sSN];
#ifdef c4DOF
        abSSCServoNum[sSN*NUMSERVOSPERLEG + 3] = cTarsPin[sSN];
#endif
    }
    // now lets loop through and get information and set servos to 1500
    for (sSN=0; sSN < 6*NUMSERVOSPERLEG; sSN++ ) {
        asOffsets[sSN] = 0;       
        asOffsetsRead[sSN] = 0; 

        SSCSerial.print("R");
        SSCSerial.println(32+abSSCServoNum[sSN], DEC);
        // now read in the current value...  Maybe should use atoi...
        cbRead = SSCRead((uint8_t*)szTemp, sizeof(szTemp), 10000, (uint16_t)13);
        if (cbRead > 0)
            asOffsetsRead[sSN] = atoi((const char *)szTemp);

        SSCSerial.print("#");
        SSCSerial.print(abSSCServoNum[sSN], DEC);
        SSCSerial.println("P1500");
    }

    // OK lets move all of the servos to their zero point.
    Serial.println("Find Servo Zeros.\n$-Exit, +- changes, *-change servo");
    Serial.println("    0-5 Chooses a leg, C-Coxa, F-Femur, T-Tibia");

    sSN = true;
    while(!fExit) {
        if (fNew) {
            Serial.print("Servo: ");
            Serial.print(apszLegs[sSN/NUMSERVOSPERLEG]);
            Serial.print(apszLJoints[sSN%NUMSERVOSPERLEG]);
            Serial.print("(");
            Serial.print(asOffsetsRead[sSN]+asOffsets[sSN], DEC);
            Serial.println(")");

            // Now lets wiggle the servo
            SSCSerial.print("#");
            SSCSerial.print(abSSCServoNum[sSN], DEC);
            SSCSerial.print("P");
            SSCSerial.print(1500+asOffsets[sSN]+250, DEC);
            SSCSerial.println("T250");
            delay(250);

            SSCSerial.print("#");
            SSCSerial.print(abSSCServoNum[sSN], DEC);
            SSCSerial.print("P");
            SSCSerial.print(1500+asOffsets[sSN]-250, DEC);
            SSCSerial.println("T500");
            delay(500);

            SSCSerial.print("#");
            SSCSerial.print(abSSCServoNum[sSN], DEC);
            SSCSerial.print("P");
            SSCSerial.print(1500+asOffsets[sSN], DEC);
            SSCSerial.println("T250");
            delay(250);

            fNew = false;
        }

        //get user entered data
        data = Serial.read();
        //if data received
        if (data !=-1) 	{
            if (data == '$')
                fExit = true;	// not sure how the keypad will map so give NL, CR, LF... all implies exit

            else if ((data == '+') || (data == '-')) {
                if (data == '+')
                    asOffsets[sSN] += 5;		// increment by 5us
                else
                    asOffsets[sSN] -= 5;		// increment by 5us

                Serial.print("    ");
                Serial.println(asOffsetsRead[sSN]+asOffsets[sSN], DEC);

                SSCSerial.print("#");
                SSCSerial.print(abSSCServoNum[sSN], DEC);
                SSCSerial.print("P");
                SSCSerial.print(1500+asOffsets[sSN], DEC);
                SSCSerial.println("T100");
            } else if ((data >= '0') && (data <= '5')) {
                // direct enter of which servo to change
                fNew = true;
                sSN = (sSN % NUMSERVOSPERLEG) + (data - '0')*NUMSERVOSPERLEG;
            } else if ((data == 'c') && (data == 'C')) {
                fNew = true;
                sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 0;
            } else if ((data == 'c') && (data == 'C')) {
                fNew = true;
                sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 1;
            } else if ((data == 'c') && (data == 'C')) {
                // direct enter of which servo to change
                fNew = true;
                sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 2;
            } else if (data == '*') {
                // direct enter of which servo to change
                fNew = true;
                sSN++;
                if (sSN == 6*NUMSERVOSPERLEG) 
                    sSN = 0;	
            }
        }
    }
    Serial.print("Find Servo exit ");
    for (sSN=0; sSN < 6*NUMSERVOSPERLEG; sSN++){
        Serial.print("Servo: ");
        Serial.print(apszLegs[sSN/NUMSERVOSPERLEG]);
        Serial.print(apszLJoints[sSN%NUMSERVOSPERLEG]);
        Serial.print("(");
        Serial.print(asOffsetsRead[sSN]+asOffsets[sSN], DEC);
        Serial.println(")");
    }

    Serial.print("\nSave Changes? Y/N: ");

    //get user entered data
    while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)))
        ; 

    if ((data == 'Y') || (data == 'y')) {
        // Ok they asked for the data to be saved.  We will store the data with a 
        // number of servos (uint8_t)at the start, followed by a uint8_t for a checksum...followed by our offsets array...
        // Currently we store these values starting at EEPROM address 0. May later change...
        // 

        for (sSN=0; sSN < 6*NUMSERVOSPERLEG; sSN++ ) {
            SSCSerial.print("R");
            SSCSerial.print(32+abSSCServoNum[sSN], DEC);
            SSCSerial.print("=");
            SSCSerial.println(asOffsetsRead[sSN]+asOffsets[sSN], DEC);
            delay(10);
        }

        // Then I need to have the SSC-32 reboot in order to use the new values.
        delay(10);    // give it some time to write stuff out.
        SSCSerial.println("GOBOOT");
        delay(5);        // Give it a little time
        SSCSerial.println("g0000");    // tell it that we are done in the boot section so go run the normall SSC stuff...
        delay(500);                // Give it some time to boot up...

    } else {
        void LoadServosConfig();
    }

    FreeServos();

}
#endif  // OPT_FIND_SERVO_OFFSETS

#endif
