#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include "common.h"

// Prototypes for functions in .ino
void StartUpdateServos();
boolean TerminalMonitor();
void GaitSelect();
void GaitSeq();
void SingleLegControl();
void WriteOutputs();
void CheckAngles();
void BalCalcOneLeg(short, short, short, byte);
void BalanceBody();
void BodyFK(short, short, short, short, byte);
void LegIK(short, short, short, byte);
void GetSinCos(short AngleDecimal);
void robot_setup();
void robot_loop();

// Include the actual robot code
#include "../../BotBoarduino_CH3R_PS2.ino"

// Include gait test after robot code so it sees its globals
#include "test_gait.h"

void run_ik_tests() {
    std::cout << "Running IK Tests..." << std::endl;
    std::ofstream csv("test/golden_data/ik_snapshot.csv");
    csv << "TestID,Leg,IKFeetX,IKFeetY,IKFeetZ,BodyRotX,BodyRotY,BodyRotZ,CoxaAngle,FemurAngle,TibiaAngle" << std::endl;

    int testId = 0;

    // Restore original range of positions
    short test_coords[] = {-100, -50, 0, 50, 100};
    short test_rots[] = {-100, 0, 100};

    for (short rx : test_rots) {
        std::cout << "Testing BodyRotX: " << rx << std::endl;
        for (short ry : test_rots) {
            for (short rz : test_rots) {
                g_InControlState.BodyRot1.x = rx;
                g_InControlState.BodyRot1.y = ry;
                g_InControlState.BodyRot1.z = rz;

                for (short tx : test_coords) {
                    for (short ty : test_coords) {
                        for (short tz : test_coords) {
                            testId++;
                            std::cout << "TestID: " << testId << " rx=" << rx << " ry=" << ry << " rz=" << rz << " tx=" << tx << " ty=" << ty << " tz=" << tz << std::endl;
                            
                            // For each leg
                            for (int leg = 0; leg < 6; ++leg) {
                                // Reset outputs
                                g_Legs[leg].coxaAngle = 0;
                                g_Legs[leg].femurAngle = 0;
                                g_Legs[leg].tibiaAngle = 0;

                                if (leg <= 2) { // Right legs
                                    g_Legs[leg].calculateBodyFK(-g_Legs[leg].posX + tx, g_Legs[leg].posZ + tz, g_Legs[leg].posY + ty, 0, 
                                                              BodyRotOffsetX, BodyRotOffsetY, BodyRotOffsetZ, TotalXBal1, TotalYBal1, TotalZBal1);
                                    g_Legs[leg].calculateLegIK(g_Legs[leg].posX - tx + g_Legs[leg].bodyFKPosX, 
                                                              g_Legs[leg].posY + ty - g_Legs[leg].bodyFKPosY,
                                                              g_Legs[leg].posZ + tz - g_Legs[leg].bodyFKPosZ);
                                } else { // Left legs
                                    g_Legs[leg].calculateBodyFK(g_Legs[leg].posX - tx, g_Legs[leg].posZ + tz, g_Legs[leg].posY + ty, 0,
                                                              BodyRotOffsetX, BodyRotOffsetY, BodyRotOffsetZ, TotalXBal1, TotalYBal1, TotalZBal1);
                                    g_Legs[leg].calculateLegIK(g_Legs[leg].posX + tx - g_Legs[leg].bodyFKPosX,
                                                              g_Legs[leg].posY + ty - g_Legs[leg].bodyFKPosY,
                                                              g_Legs[leg].posZ + tz - g_Legs[leg].bodyFKPosZ);
                                }

                                csv << testId << "," << leg << "," << tx << "," << ty << "," << tz << "," 
                                    << rx << "," << ry << "," << rz << ","
                                    << g_Legs[leg].coxaAngle << "," << g_Legs[leg].femurAngle << "," << g_Legs[leg].tibiaAngle << std::endl;
                            }
                            // if (testId % 10 == 0) std::cout << "Finished TestID: " << testId << std::endl;
                        }
                    }
                }
            }
        }
    }
    csv.close();
    std::cout << "IK Tests Complete. Snapshot saved to test/golden_data/ik_snapshot.csv" << std::endl;
}

int main() {
    try {
        std::cout << "Starting Test Runner..." << std::endl;
        // Basic setup
        robot_setup();
        
        std::cout << "Starting IK Tests..." << std::endl;
        run_ik_tests();
        std::cout << "Starting Gait Tests..." << std::endl;
        run_gait_tests();
        std::cout << "All Tests Complete." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Crashed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Crashed with unknown exception" << std::endl;
        return 1;
    }
    
    return 0;
}
