#include "test_utils.h"

extern PS2X ps2x; // From PS2_controller.cpp

bool run_gait_tests() {
    std::cout << "\n[PHASE] Starting Gait & Trajectory Regression Tests..." << std::endl;
    
    std::ifstream golden_file("test/golden_data/gait_snapshot.csv");
    bool has_golden = golden_file.is_open();
    
    if (has_golden) {
        std::string header;
        std::getline(golden_file, header); // Skip header
        std::cout << "[INFO] Comparing against Golden Master: test/golden_data/gait_snapshot.csv" << std::endl;
    } else {
        std::cout << "[WARN] No Golden Master found. Generating new baseline snapshot." << std::endl;
    }

    std::ofstream out_csv("test/golden_data/gait_snapshot_current.csv");
    out_csv << "Cycle,GaitStep,fWalking,fContinueWalking,ServoMoveTime,SerialOutput";
    for(int i=0; i<6; i++) {
        out_csv << ",L" << i << "X,L" << i << "Y,L" << i << "Z";
    }
    out_csv << std::endl;

    // Reset robot state for deterministic testing
    g_mock_millis = 0;
    g_mock_micros = 0;
    g_InControlState.fHexOn = true;
    ps2x.analogs[PSS_LY] = 0; // Push stick forward

    int failures = 0;
    int matches = 0;

    for (int cycle = 0; cycle < 100; ++cycle) {
        g_mock_millis += 20;
        g_mock_micros += 20000;

        SoftwareSerial::clear();
        robot_loop();

        std::string escapedSerial = SoftwareSerial::lastOutput;
        size_t pos = 0;
        while ((pos = escapedSerial.find(',', pos)) != std::string::npos) {
            escapedSerial.replace(pos, 1, ";");
            pos += 1;
        }
        while ((pos = escapedSerial.find("\r\n", 0)) != std::string::npos) {
            escapedSerial.replace(pos, 2, " ");
        }

        // Record current
        out_csv << cycle << "," << (int)GaitStep << "," << fWalking << "," << fContinueWalking << "," << ServoMoveTime << "," << escapedSerial;
        std::vector<short> current_coords;
        for (int i = 0; i < 6; ++i) {
            out_csv << "," << g_Legs[i].gaitPosX << "," << g_Legs[i].gaitPosY << "," << g_Legs[i].gaitPosZ;
            current_coords.push_back(g_Legs[i].gaitPosX);
            current_coords.push_back(g_Legs[i].gaitPosY);
            current_coords.push_back(g_Legs[i].gaitPosZ);
        }
        out_csv << std::endl;

        // Verify
        if (has_golden) {
            std::string line;
            if (std::getline(golden_file, line)) {
                std::vector<std::string> parts = split_csv(line);
                if (parts.size() >= 24) { // 6 meta + 18 coords
                    bool cycle_fail = false;
                    
                    // Check walking flags, move time, and raw serial output
                    if (std::stoi(parts[2]) != fWalking || 
                        std::stoi(parts[4]) != ServoMoveTime ||
                        parts[5] != escapedSerial) {
                        cycle_fail = true;
                    }

                    // Check coordinates
                    for (int i = 0; i < 18; ++i) {
                        if (std::stoi(parts[6+i]) != current_coords[i]) {
                            cycle_fail = true;
                            break;
                        }
                    }

                    if (cycle_fail) {
                        failures++;
                        if (failures < 5) {
                            std::cout << "[FAIL] Cycle " << cycle << " state deviation!" << std::endl;
                            std::cout << "  Expected fWalking=" << parts[2] << " ServoMoveTime=" << parts[4] << std::endl;
                            std::cout << "  Actual   fWalking=" << fWalking << " ServoMoveTime=" << ServoMoveTime << std::endl;
                        }
                    } else {
                        matches++;
                    }
                }
            }
        }
    }

    out_csv.close();
    if (has_golden) golden_file.close();

    std::cout << "[RESULT] Gait Tests Finished. Matches: " << matches << ", Failures: " << failures << std::endl;
    if (has_golden && failures > 0) {
        std::cout << "[ERROR] GAIT REGRESSION FAILED!" << std::endl;
        return false;
    } else if (has_golden) {
        std::cout << "[SUCCESS] GAIT REGRESSION PASSED." << std::endl;
    }
    return true;
}
