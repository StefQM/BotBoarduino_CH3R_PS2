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

bool compare_servo_hex(const std::string& actual_hex, const std::string& expected_hex, int pwm_tolerance, bool strict) {
    if (strict) return actual_hex == expected_hex;
    if (actual_hex.length() != expected_hex.size()) return false;
    
    // Parse 3-byte (6 hex char) chunks: [ID+0x80][PosHigh][PosLow]
    for (size_t i = 0; i < actual_hex.length(); i += 6) {
        if (i + 6 > actual_hex.length()) break;
        
        std::string a_chunk = actual_hex.substr(i, 6);
        std::string e_chunk = expected_hex.substr(i, 6);
        
        // IDs must match exactly
        if (a_chunk.substr(0, 2) != e_chunk.substr(0, 2)) return false;
        
        // Parse PWM values
        uint16_t a_pwm = std::stoul(a_chunk.substr(2, 4), nullptr, 16);
        uint16_t e_pwm = std::stoul(e_chunk.substr(2, 4), nullptr, 16);
        
        if (!is_near((int)a_pwm, (int)e_pwm, pwm_tolerance)) return false;
    }
    return true;
}

bool run_servo_tests() {
    std::cout << "\n[PHASE] Starting Servo Driver (SSC32) Regression Tests..." << std::endl;
    
    std::ifstream legacy_file("test/golden_data/servo_snapshot.csv");
    std::ifstream float_file("test/golden_data/servo_snapshot_float.csv");
    
    bool has_legacy = legacy_file.is_open();
    bool has_float = float_file.is_open();
    
    if (has_legacy) std::cout << "[INFO] Legacy Master: test/golden_data/servo_snapshot.csv (PWM Tolerance: 10)" << std::endl;
    if (has_float)  std::cout << "[INFO] Float Master:  test/golden_data/servo_snapshot_float.csv (Strict)" << std::endl;

    std::ofstream out_csv("test/golden_data/servo_snapshot_current.csv");
    out_csv << "label,output_hex\n";

    short angles[6][3];
    auto run_and_record = [&](const std::string& label, short test_angles[6][3], word moveTime) {
        snapshot_servo(out_csv, label, test_angles, moveTime);
    };

    for(int i=0; i<6; i++) for(int j=0; j<3; j++) angles[i][j] = 0;
    run_and_record("all_zero", angles, 200);
    for(int i=0; i<6; i++) { angles[i][0] = 650; angles[i][1] = 750; angles[i][2] = 900; }
    run_and_record("all_max", angles, 500);
    for(int i=0; i<6; i++) { angles[i][0] = -650; angles[i][1] = -1050; angles[i][2] = -530; }
    run_and_record("all_min", angles, 100);
    for(int i=0; i<6; i++) { angles[i][0] = (i % 2 == 0) ? 200 : -200; angles[i][1] = (i % 2 == 0) ? 300 : -300; angles[i][2] = (i % 2 == 0) ? 400 : -400; }
    run_and_record("mixed", angles, 300);
    
    SoftwareSerial::clear();
    g_ServoDriver.FreeServos();
    out_csv << "free_servos," << to_hex(SoftwareSerial::lastOutput) << "\n";
    out_csv.close();

    std::ifstream current_file("test/golden_data/servo_snapshot_current.csv");
    std::string line_l, line_f, line_c;
    int legacy_failures = 0, float_failures = 0, line_num = 0;

    std::getline(current_file, line_c); // Skip header
    if (has_legacy) std::getline(legacy_file, line_l);
    if (has_float)  std::getline(float_file, line_f);

    while (std::getline(current_file, line_c)) {
        line_num++;
        std::vector<std::string> parts_c = split_csv(line_c);
        if (has_legacy && std::getline(legacy_file, line_l)) {
            std::vector<std::string> parts_l = split_csv(line_l);
            if (!compare_servo_hex(parts_c[1], parts_l[1], 10, false)) legacy_failures++;
        }
        if (has_float && std::getline(float_file, line_f)) {
            std::vector<std::string> parts_f = split_csv(line_f);
            if (!compare_servo_hex(parts_c[1], parts_f[1], 0, true)) float_failures++;
        }
    }
    
    if (has_legacy) legacy_file.close();
    if (has_float) float_file.close();
    current_file.close();

    if (legacy_failures > 0) std::cout << "[FAIL] Legacy Servo Regression deviation outside tolerance!" << std::endl;
    if (float_failures > 0)   std::cout << "[FAIL] Float Servo Regression deviate from perfect baseline!" << std::endl;

    if (legacy_failures == 0) std::cout << "[SUCCESS] SERVO REGRESSION PASSED (within legacy tolerance)." << std::endl;
    return (legacy_failures == 0);
}

#endif
