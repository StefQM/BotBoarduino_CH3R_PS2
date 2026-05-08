#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include "common.h"
#include "test_utils.h"

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

bool run_ik_tests() {
    std::cout << "\n[PHASE] Starting Inverse Kinematics (IK) Regression Tests..." << std::endl;
    
    std::ifstream golden_file("test/golden_data/ik_snapshot.csv");
    bool has_golden = golden_file.is_open();
    
    if (has_golden) {
        std::string header;
        std::getline(golden_file, header); // Skip header
        std::cout << "[INFO] Comparing against Golden Master: test/golden_data/ik_snapshot.csv" << std::endl;
    } else {
        std::cout << "[WARN] No Golden Master found. Generating new baseline snapshot." << std::endl;
    }

    std::ofstream out_csv("test/golden_data/ik_snapshot_current.csv");
    out_csv << "TestID,Leg,IKFeetX,IKFeetY,IKFeetZ,BodyRotX,BodyRotY,BodyRotZ,CoxaAngle,FemurAngle,TibiaAngle" << std::endl;

    int testId = 0;
    int failures = 0;
    int matches = 0;

    short test_coords[] = {-100, -50, 0, 50, 100};
    short test_rots[] = {-100, 0, 100};

    for (short rx : test_rots) {
        for (short ry : test_rots) {
            for (short rz : test_rots) {
                g_InControlState.BodyRot1.x = rx;
                g_InControlState.BodyRot1.y = ry;
                g_InControlState.BodyRot1.z = rz;

                for (short tx : test_coords) {
                    for (short ty : test_coords) {
                        for (short tz : test_coords) {
                            testId++;
                            
                            for (int leg = 0; leg < 6; ++leg) {
                                g_Legs[leg].coxaAngle = 0;
                                g_Legs[leg].femurAngle = 0;
                                g_Legs[leg].tibiaAngle = 0;

                                if (leg <= 2) { // Right legs
                                    g_Legs[leg].calculateBodyFK(-g_Legs[leg].posX + tx, g_Legs[leg].posY + ty, g_Legs[leg].posZ + tz, 0, 
                                                              g_Hexapod.BodyRotOffsetX, g_Hexapod.BodyRotOffsetY, g_Hexapod.BodyRotOffsetZ, g_Hexapod.TotalXBal1, g_Hexapod.TotalYBal1, g_Hexapod.TotalZBal1);
                                    g_Legs[leg].calculateLegIK(g_Legs[leg].posX - tx + g_Legs[leg].bodyFKPosX,
                                                              g_Legs[leg].posY + ty - g_Legs[leg].bodyFKPosY,
                                                              g_Legs[leg].posZ + tz - g_Legs[leg].bodyFKPosZ);
                                } else { // Left legs
                                    g_Legs[leg].calculateBodyFK(g_Legs[leg].posX - tx, g_Legs[leg].posY + ty, g_Legs[leg].posZ + tz, 0, 
                                                              g_Hexapod.BodyRotOffsetX, g_Hexapod.BodyRotOffsetY, g_Hexapod.BodyRotOffsetZ, g_Hexapod.TotalXBal1, g_Hexapod.TotalYBal1, g_Hexapod.TotalZBal1);
                                    g_Legs[leg].calculateLegIK(g_Legs[leg].posX + tx - g_Legs[leg].bodyFKPosX,
                                                              g_Legs[leg].posY + ty - g_Legs[leg].bodyFKPosY,
                                                              g_Legs[leg].posZ + tz - g_Legs[leg].bodyFKPosZ);
                                }
                                // Write current result
                                out_csv << testId << "," << leg << "," << tx << "," << ty << "," << tz << "," 
                                    << rx << "," << ry << "," << rz << ","
                                    << g_Legs[leg].coxaAngle << "," << g_Legs[leg].femurAngle << "," << g_Legs[leg].tibiaAngle << std::endl;

                                // Compare with golden if available
                                if (has_golden) {
                                    std::string line;
                                    if (std::getline(golden_file, line)) {
                                        std::vector<std::string> parts = split_csv(line);
                                        if (parts.size() >= 11) {
                                            short g_coxa = std::stoi(parts[8]);
                                            short g_femur = std::stoi(parts[9]);
                                            short g_tibia = std::stoi(parts[10]);

                                            if (g_Legs[leg].coxaAngle != g_coxa || 
                                                g_Legs[leg].femurAngle != g_femur || 
                                                g_Legs[leg].tibiaAngle != g_tibia) {
                                                
                                                failures++;
                                                if (failures < 10) { // Limit error output
                                                    std::cout << "[FAIL] TestID " << testId << " Leg " << leg << " mismatch!" << std::endl;
                                                    std::cout << "  Expected: Coxa=" << g_coxa << " Femur=" << g_femur << " Tibia=" << g_tibia << std::endl;
                                                    std::cout << "  Actual:   Coxa=" << g_Legs[leg].coxaAngle << " Femur=" << g_Legs[leg].femurAngle << " Tibia=" << g_Legs[leg].tibiaAngle << std::endl;
                                                }
                                            } else {
                                                matches++;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    out_csv.close();
    if (has_golden) golden_file.close();

    std::cout << "[RESULT] IK Tests Finished. Matches: " << matches << ", Failures: " << failures << std::endl;
    if (has_golden && failures > 0) {
        std::cout << "[ERROR] IK REGRESSION FAILED!" << std::endl;
        return false;
    } else if (has_golden) {
        std::cout << "[SUCCESS] IK REGRESSION PASSED." << std::endl;
    }
    return true;
}

int main() {
    try {
        std::cout << "Starting Test Runner..." << std::endl;
        // Basic setup
        robot_setup();
        
        bool ik_pass = run_ik_tests();
        bool gait_pass = run_gait_tests();
        
        std::cout << "\n[FINAL SUMMARY]" << std::endl;
        std::cout << "IK Regression:   " << (ik_pass ? "PASSED" : "FAILED") << std::endl;
        std::cout << "Gait Regression: " << (gait_pass ? "PASSED" : "FAILED") << std::endl;

        if (!ik_pass || !gait_pass) {
            std::cout << "\n[RESULT] TEST SUITE FAILED." << std::endl;
            return 1;
        }

        std::cout << "\n[RESULT] ALL TESTS PASSED." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Crashed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Crashed with unknown exception" << std::endl;
        return 1;
    }
    
    return 0;
}
