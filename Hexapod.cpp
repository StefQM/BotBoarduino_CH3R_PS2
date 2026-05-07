#include "Hexapod.h"
#include "Hex_Globals.h"

Hexapod g_Hexapod;

Hexapod::Hexapod() {
    init();
}

void Hexapod::init() {
    lTimerStart = 0;
    lTimerEnd = 0;
    CycleTime = 0;
    ServoMoveTime = 150;
    PrevServoMoveTime = 150;

    TotalTransX = TotalTransY = TotalTransZ = 0;
    TotalXBal1 = TotalYBal1 = TotalZBal1 = 0;

    NomGaitSpeed = 70;
    TLDivFactor = 8;
    NrLiftedPos = 3;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TravelRequest = false;
    StepsInGait = 1;
    GaitStep = 1;
    for(int i=0; i<6; i++) GaitLegNr[i] = 0;
    LastLeg = false;

    IKSolution = false;
    IKSolutionWarning = false;
    IKSolutionError = false;
    BodyRotOffsetX = BodyRotOffsetY = BodyRotOffsetZ = 0;

    sin4 = 0; cos4 = 0; AngleRad4 = 0; Atan4 = 0; XYhyp2 = 0;

    fWalking = false;
    fContinueWalking = false;
    fLowVoltageShutdown = false;
    Voltage = 0;
    Eyes = LedA = LedB = LedC = 0;

    PrevSelectedLeg = 255;
    AllDown = false;
}

void Hexapod::GaitSelect(void)
{
    switch (g_InControlState.GaitType)  {
        case 0:
            GaitLegNr[cLR] = 1; GaitLegNr[cRF] = 3; GaitLegNr[cLM] = 5;
            GaitLegNr[cRR] = 7; GaitLegNr[cLF] = 9; GaitLegNr[cRM] = 11;
            NrLiftedPos = 3; HalfLiftHeigth = 3; TLDivFactor = 8; StepsInGait = 12; NomGaitSpeed = 70;
            break;
        case 1:
            GaitLegNr[cLR] = 5; GaitLegNr[cRF] = 1; GaitLegNr[cLM] = 1;
            GaitLegNr[cRR] = 1; GaitLegNr[cLF] = 5; GaitLegNr[cRM] = 5;
            NrLiftedPos = 3; HalfLiftHeigth = 3; TLDivFactor = 4; StepsInGait = 8; NomGaitSpeed = 70;
            break;
        case 2:
            GaitLegNr[cRF] = 3; GaitLegNr[cLM] = 4; GaitLegNr[cRR] = 5;
            GaitLegNr[cLF] = 9; GaitLegNr[cRM] = 10; GaitLegNr[cLR] = 11;
            NrLiftedPos = 3; HalfLiftHeigth = 3; TLDivFactor = 8; StepsInGait = 12; NomGaitSpeed = 60;
            break;
        case 3:
            GaitLegNr[cRF] = 4; GaitLegNr[cLM] = 5; GaitLegNr[cRR] = 6;
            GaitLegNr[cLF] = 12; GaitLegNr[cRM] = 13; GaitLegNr[cLR] = 14;
            NrLiftedPos = 5; HalfLiftHeigth = 1; TLDivFactor = 10; StepsInGait = 16; NomGaitSpeed = 60;
            break;
        case 4:
            GaitLegNr[cLR] = 1; GaitLegNr[cRF] = 21; GaitLegNr[cLM] = 5;
            GaitLegNr[cRR] = 13; GaitLegNr[cLF] = 9; GaitLegNr[cRM] = 17;
            NrLiftedPos = 3; HalfLiftHeigth = 3; TLDivFactor = 20; StepsInGait = 24; NomGaitSpeed = 70;
            break;
    }
}    

void Hexapod::GaitSeq(void)
{
    TravelRequest = (abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) 
          || (abs(g_InControlState.TravelLength.y)>cTravelDeadZone) || (g_InControlState.ForceGaitStepCnt != 0);
    if (NrLiftedPos == 5) LiftDivFactor = 4;    
    else LiftDivFactor = 2;

    LastLeg = 0;
    for (byte i = 0; i <= 5; i++) {
        if (i == 5) LastLeg = 1;
        Gait(i);
    }
}

void Hexapod::Gait (byte GaitCurrentLegNr)
{
    if (!TravelRequest) {    
        g_InControlState.TravelLength.x=0;
        g_InControlState.TravelLength.z=0;
        g_InControlState.TravelLength.y=0;
    }
    if ((TravelRequest && (NrLiftedPos==1 || NrLiftedPos==3 || NrLiftedPos==5) && 
            GaitStep==GaitLegNr[GaitCurrentLegNr]) || (!TravelRequest && GaitStep==GaitLegNr[GaitCurrentLegNr] && ((abs(g_Legs[GaitCurrentLegNr].gaitPosX)>2) || 
                (abs(g_Legs[GaitCurrentLegNr].gaitPosZ)>2) || (abs(g_Legs[GaitCurrentLegNr].gaitRotY)>2)))) { 
        g_Legs[GaitCurrentLegNr].gaitPosX = 0;
        g_Legs[GaitCurrentLegNr].gaitPosY = -g_InControlState.LegLiftHeight;
        g_Legs[GaitCurrentLegNr].gaitPosZ = 0;
        g_Legs[GaitCurrentLegNr].gaitRotY = 0;
    }
    else if (((NrLiftedPos==2 && GaitStep==GaitLegNr[GaitCurrentLegNr]) || (NrLiftedPos>=3 && 
            (GaitStep==GaitLegNr[GaitCurrentLegNr]-1 || GaitStep==GaitLegNr[GaitCurrentLegNr]+(StepsInGait-1))))
            && TravelRequest) {
        g_Legs[GaitCurrentLegNr].gaitPosX = -g_InControlState.TravelLength.x/LiftDivFactor;
        g_Legs[GaitCurrentLegNr].gaitPosY = -3*g_InControlState.LegLiftHeight/(3+HalfLiftHeigth);
        g_Legs[GaitCurrentLegNr].gaitPosZ = -g_InControlState.TravelLength.z/LiftDivFactor;
        g_Legs[GaitCurrentLegNr].gaitRotY = -g_InControlState.TravelLength.y/LiftDivFactor;
    }    
    else if ((NrLiftedPos>=2) && (GaitStep==GaitLegNr[GaitCurrentLegNr]+1 || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-1)) && TravelRequest) {
        g_Legs[GaitCurrentLegNr].gaitPosX = g_InControlState.TravelLength.x/LiftDivFactor;
        g_Legs[GaitCurrentLegNr].gaitPosY = -3*g_InControlState.LegLiftHeight/(3+HalfLiftHeigth);
        g_Legs[GaitCurrentLegNr].gaitPosZ = g_InControlState.TravelLength.z/LiftDivFactor;
        g_Legs[GaitCurrentLegNr].gaitRotY = g_InControlState.TravelLength.y/LiftDivFactor;
    }
    else if (((NrLiftedPos==5 && (GaitStep==GaitLegNr[GaitCurrentLegNr]-2 ))) && TravelRequest) {
        g_Legs[GaitCurrentLegNr].gaitPosX = -g_InControlState.TravelLength.x/2;
        g_Legs[GaitCurrentLegNr].gaitPosY = -g_InControlState.LegLiftHeight/2;
        g_Legs[GaitCurrentLegNr].gaitPosZ = -g_InControlState.TravelLength.z/2;
        g_Legs[GaitCurrentLegNr].gaitRotY = -g_InControlState.TravelLength.y/2;
     }  		
    else if ((NrLiftedPos==5) && (GaitStep==GaitLegNr[GaitCurrentLegNr]+2 || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-2)) && TravelRequest) {
        g_Legs[GaitCurrentLegNr].gaitPosX = g_InControlState.TravelLength.x/2;
        g_Legs[GaitCurrentLegNr].gaitPosY = -g_InControlState.LegLiftHeight/2;
        g_Legs[GaitCurrentLegNr].gaitPosZ = g_InControlState.TravelLength.z/2;
        g_Legs[GaitCurrentLegNr].gaitRotY = g_InControlState.TravelLength.y/2;
    }
  else if ((GaitStep==GaitLegNr[GaitCurrentLegNr]+NrLiftedPos || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-NrLiftedPos))
            && g_Legs[GaitCurrentLegNr].gaitPosY<0) {
        g_Legs[GaitCurrentLegNr].gaitPosX = g_InControlState.TravelLength.x/2;
        g_Legs[GaitCurrentLegNr].gaitPosZ = g_InControlState.TravelLength.z/2;
        g_Legs[GaitCurrentLegNr].gaitRotY = g_InControlState.TravelLength.y/2;      	
        g_Legs[GaitCurrentLegNr].gaitPosY = 0;
    }
    else {
        g_Legs[GaitCurrentLegNr].gaitPosX = g_Legs[GaitCurrentLegNr].gaitPosX - (g_InControlState.TravelLength.x/TLDivFactor);
        g_Legs[GaitCurrentLegNr].gaitPosY = 0; 
        g_Legs[GaitCurrentLegNr].gaitPosZ = g_Legs[GaitCurrentLegNr].gaitPosZ - (g_InControlState.TravelLength.z/TLDivFactor);
        g_Legs[GaitCurrentLegNr].gaitRotY = g_Legs[GaitCurrentLegNr].gaitRotY - (g_InControlState.TravelLength.y/TLDivFactor);
    }
    if (LastLeg)  { 
        GaitStep++;
        if (GaitStep>StepsInGait) GaitStep = 1;
    }
}  

void Hexapod::BalCalcOneLeg (short PosX, short PosZ, short PosY, byte BalLegNr)
{
    short CPR_X, CPR_Y, CPR_Z;
    long lAtan;
    CPR_Z = (short)pgm_read_word(&cOffsetZ[BalLegNr]) + PosZ;
    CPR_X = (short)pgm_read_word(&cOffsetX[BalLegNr]) + PosX;
    CPR_Y = 150 + PosY;
    TotalTransY += (long)PosY;
    TotalTransZ += (long)CPR_Z;
    TotalTransX += (long)CPR_X;
    lAtan = GetATan2(CPR_X, CPR_Z);
    TotalYBal1 += (lAtan*1800) / 31415;
    lAtan = GetATan2 (CPR_X, CPR_Y);
    TotalZBal1 += ((lAtan*1800) / 31415) -900; 
    lAtan = GetATan2 (CPR_Z, CPR_Y);
    TotalXBal1 += ((lAtan*1800) / 31415) - 900; 
}  

void Hexapod::BalanceBody(void)
{
    TotalTransZ /= BalanceDivFactor;
    TotalTransX /= BalanceDivFactor;
    TotalTransY /= BalanceDivFactor;
    if (TotalYBal1 > 0) TotalYBal1 -= 1800; else TotalYBal1 += 1800;
    if (TotalZBal1 < -1800) TotalZBal1 += 3600;
    if (TotalXBal1 < -1800) TotalXBal1 += 3600;
    TotalYBal1 = -TotalYBal1/BalanceDivFactor;
    TotalXBal1 = -TotalXBal1/BalanceDivFactor;
    TotalZBal1 /= BalanceDivFactor;
}

void Hexapod::CheckAngles(void)
{
    for (byte i = 0; i <=5; i++) {
        g_Legs[i].coxaAngle  = min(max(g_Legs[i].coxaAngle, (short)pgm_read_word(&cCoxaMin1[i])), (short)pgm_read_word(&cCoxaMax1[i]));
        g_Legs[i].femurAngle = min(max(g_Legs[i].femurAngle, (short)pgm_read_word(&cFemurMin1[i])), (short)pgm_read_word(&cFemurMax1[i]));
        g_Legs[i].tibiaAngle = min(max(g_Legs[i].tibiaAngle, (short)pgm_read_word(&cTibiaMin1[i])), (short)pgm_read_word(&cTibiaMax1[i]));
#ifdef c4DOF
        if ((byte)pgm_read_byte(&cTarsLength[i])) {
            g_Legs[i].tarsAngle = min(max(g_Legs[i].tarsAngle, (short)pgm_read_word(&cTarsMin1[i])), (short)pgm_read_word(&cTarsMax1[i]));
        }
#endif
    }
}

bool Hexapod::CheckVoltage() {
#ifdef  cVoltagePin   
#ifdef cTurnOffVol
    Voltage = analogRead(cVoltagePin); 
    Voltage = ((long)Voltage*1955)/1000;
    static byte s_bLVBeepCnt = 0;
    if (!fLowVoltageShutdown) {
        if ((Voltage < cTurnOffVol) || (Voltage >= 1999)) {
	    g_InControlState.BodyPos.x = g_InControlState.BodyPos.y = g_InControlState.BodyPos.z = 0;
	    g_InControlState.BodyRot1.x = g_InControlState.BodyRot1.y = g_InControlState.BodyRot1.z = 0;
	    g_InControlState.TravelLength.x = g_InControlState.TravelLength.z = g_InControlState.TravelLength.y = 0;
	    g_InControlState.SelectedLeg = 255;
	    fLowVoltageShutdown = 1;
            s_bLVBeepCnt = 0;
	    g_InControlState.fHexOn = false;
	}
    } else if ((Voltage > cTurnOnVol) && (Voltage < 1999)) {
            fLowVoltageShutdown = 0;
    } else {
        if (s_bLVBeepCnt < 5) { s_bLVBeepCnt++; MSound(SOUND_PIN, 1, 45, 2000); }
        delay(2000);
    }
#endif	
#endif
  return fLowVoltageShutdown;
}

void Hexapod::GetSinCos(short AngleDeg1) {
    short ABSAngleDeg1 = (AngleDeg1 < 0) ? AngleDeg1 *-1 : AngleDeg1;
    if (AngleDeg1 < 0) AngleDeg1 = 3600-(ABSAngleDeg1-(3600*(ABSAngleDeg1/3600)));
    else AngleDeg1 = ABSAngleDeg1-(3600*(ABSAngleDeg1/3600));
    if (AngleDeg1>=0 && AngleDeg1<=900) {
        sin4 = pgm_read_word(&GetSin[AngleDeg1/5]);
        cos4 = pgm_read_word(&GetSin[(900-(AngleDeg1))/5]);
    } else if (AngleDeg1>900 && AngleDeg1<=1800) {
        sin4 = pgm_read_word(&GetSin[(900-(AngleDeg1-900))/5]);
        cos4 = -pgm_read_word(&GetSin[(AngleDeg1-900)/5]);            
    } else if (AngleDeg1>1800 && AngleDeg1<=2700) {
        sin4 = -pgm_read_word(&GetSin[(AngleDeg1-1800)/5]);
        cos4 = -pgm_read_word(&GetSin[(2700-AngleDeg1)/5]);
    } else if(AngleDeg1>2700 && AngleDeg1<=3600) {
        sin4 = -pgm_read_word(&GetSin[(3600-AngleDeg1)/5]);
        cos4 = pgm_read_word(&GetSin[(AngleDeg1-2700)/5]);            
    }
}

long Hexapod::GetArcCos(short cos4_in) {
    boolean NegativeValue = (cos4_in < 0);
    if (NegativeValue) cos4_in = -cos4_in;
    cos4_in = min(cos4_in, (short)c4DEC);
    if ((cos4_in>=0) && (cos4_in<9000)) AngleRad4 = ((long)(byte)pgm_read_byte(&GetACos[cos4_in/79])*616)/c1DEC;
    else if ((cos4_in>=9000) && (cos4_in<9900)) AngleRad4 = ((long)(byte)pgm_read_byte(&GetACos[(cos4_in-9000)/8+114])*616)/c1DEC;
    else if ((cos4_in>=9900) && (cos4_in<=10000)) AngleRad4 = ((long)(byte)pgm_read_byte(&GetACos[(cos4_in-9900)/2+227])*616)/c1DEC;
    if (NegativeValue) AngleRad4 = 31416 - AngleRad4;
    return AngleRad4;
}

unsigned long Hexapod::isqrt32(unsigned long n) {
    unsigned long root = 0, remainder = n, place = 0x40000000;
    while (place > remainder) place = place >> 2;
    while (place) {
        if (remainder >= root + place) { remainder = remainder - root - place; root = root + (place << 1); }
        root = root >> 1; place = place >> 2;
    }
    return root;
}

short Hexapod::GetATan2(short AtanX_in, short AtanY_in) {
    XYhyp2 = isqrt32(((long)AtanX_in*AtanX_in*c4DEC) + ((long)AtanY_in*AtanY_in*c4DEC));
    if (XYhyp2 > 0) GetArcCos(((long)AtanX_in*(long)c6DEC) /(long) XYhyp2);
    else AngleRad4 = 0;
    if (AtanY_in < 0) Atan4 = -AngleRad4; else Atan4 = AngleRad4;
    return Atan4;
}
