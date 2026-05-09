#ifndef TEST_SERVO_H
#define TEST_SERVO_H

#include "common.h"
#include "test_utils.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

std::string to_hex(const std::string& s) {
    static const char* const lut = "0123456789ABCDEF";
    std::string out;
    for (unsigned char c : s) {
        out.push_back(lut[c >> 4]);
        out.push_back(lut[c & 15]);
    }
    return out;
}

void snapshot_servo(std::ofstream& csv, const std::string& label, short angles[6][3], word moveTime) {
    SoftwareSerial::clear();
    g_ServoDriver.BeginServoUpdate();
    for (int i = 0; i < 6; i++) {
        g_ServoDriver.OutputServoInfoForLeg(i, angles[i][0], angles[i][1], angles[i][2]);
    }
    g_ServoDriver.CommitServoDriver(moveTime);
    csv << label << "," << to_hex(SoftwareSerial::lastOutput) << "\n";
}

bool run_servo_tests() {
    std::cout << "\n[PHASE] Starting Servo Driver (SSC32) Regression Tests..." << std::endl;
    
    std::ifstream golden_file("test/golden_data/servo_snapshot.csv");
    bool has_golden = golden_file.is_open();
    
    if (has_golden) {
        std::cout << "[INFO] Comparing against Golden Master: test/golden_data/servo_snapshot.csv" << std::endl;
    } else {
        std::cout << "[WARN] No Golden Master found. Generating new baseline snapshot." << std::endl;
    }

    std::ofstream out_csv("test/golden_data/servo_snapshot_current.csv");
    out_csv << "label,output_hex\n";

    short angles[6][3];
    
    auto run_and_record = [&](const std::string& label, short test_angles[6][3], word moveTime) {
        snapshot_servo(out_csv, label, test_angles, moveTime);
    };

    // Test 1: All zeros
    for(int i=0; i<6; i++) for(int j=0; j<3; j++) angles[i][j] = 0;
    run_and_record("all_zero", angles, 200);

    // Test 2: All max
    for(int i=0; i<6; i++) {
        angles[i][0] = 650;
        angles[i][1] = 750;
        angles[i][2] = 900;
    }
    run_and_record("all_max", angles, 500);

    // Test 3: All min
    for(int i=0; i<6; i++) {
        angles[i][0] = -650;
        angles[i][1] = -1050;
        angles[i][2] = -530;
    }
    run_and_record("all_min", angles, 100);

    // Test 4: Mixed
    for(int i=0; i<6; i++) {
        angles[i][0] = (i % 2 == 0) ? 200 : -200;
        angles[i][1] = (i % 2 == 0) ? 300 : -300;
        angles[i][2] = (i % 2 == 0) ? 400 : -400;
    }
    run_and_record("mixed", angles, 300);

    // Test 5: Free Servos
    SoftwareSerial::clear();
    g_ServoDriver.FreeServos();
    out_csv << "free_servos," << to_hex(SoftwareSerial::lastOutput) << "\n";

    out_csv.close();

    if (!has_golden) {
        std::cout << "[RESULT] Servo snapshots saved to test/golden_data/servo_snapshot_current.csv" << std::endl;
        return true;
    }

    // Comparison logic
    std::ifstream current_file("test/golden_data/servo_snapshot_current.csv");
    std::string line_g, line_c;
    int failures = 0;
    int line_num = 0;

    // Skip headers
    std::getline(golden_file, line_g);
    std::getline(current_file, line_c);

    while (std::getline(golden_file, line_g) && std::getline(current_file, line_c)) {
        line_num++;
        if (line_g != line_c) {
            failures++;
            if (failures < 10) {
                std::cout << "[FAIL] Mismatch at line " << line_num << std::endl;
                std::cout << "  Expected: " << line_g << std::endl;
                std::cout << "  Actual:   " << line_c << std::endl;
            }
        }
    }
    
    golden_file.close();
    current_file.close();

    if (failures > 0) {
        std::cout << "[RESULT] Servo Tests Finished. Failures: " << failures << std::endl;
        std::cout << "[ERROR] SERVO REGRESSION FAILED!" << std::endl;
        return false;
    }

    std::cout << "[SUCCESS] SERVO REGRESSION PASSED." << std::endl;
    return true;
}

#endif
