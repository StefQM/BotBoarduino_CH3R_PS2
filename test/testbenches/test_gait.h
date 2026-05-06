#ifndef TEST_GAIT_H
#define TEST_GAIT_H

#include "common.h"
#include <fstream>

extern PS2X ps2x; // From PS2_controller.cpp

void run_gait_tests() {
    std::cout << "Running Gait Tests..." << std::endl;
    std::ofstream csv("test/golden_data/gait_snapshot.csv");
    csv << "Cycle,GaitStep,fWalking,fContinueWalking,ServoMoveTime,SerialOutput";
    for(int i=0; i<6; i++) {
        csv << ",L" << i << "X,L" << i << "Y,L" << i << "Z";
    }
    csv << std::endl;

    // 1. Turn on the robot
    g_InControlState.fHexOn = true;
    
    // 2. Simulate pushing the left stick forward
    ps2x.analogs[PSS_LY] = 0; 

    // Run for 100 cycles to capture several gait steps
    for (int cycle = 0; cycle < 100; ++cycle) {
        // Advance time manually to simulate a stable 20ms loop
        g_mock_millis += 20;
        g_mock_micros += 20000;

        SoftwareSerial::clear();
        robot_loop();

        // Escape commas in serial output for CSV safety
        std::string escapedSerial = SoftwareSerial::lastOutput;
        size_t pos = 0;
        while ((pos = escapedSerial.find(',', pos)) != std::string::npos) {
            escapedSerial.replace(pos, 1, ";");
            pos += 1;
        }
        // Remove newlines
        while ((pos = escapedSerial.find("\r\n", 0)) != std::string::npos) {
            escapedSerial.replace(pos, 2, " ");
        }

        csv << cycle << "," << (int)GaitStep << "," << fWalking << "," << fContinueWalking << "," << ServoMoveTime << "," << escapedSerial;
        for (int i = 0; i < 6; ++i) {
            csv << "," << g_Legs[i].gaitPosX << "," << g_Legs[i].gaitPosY << "," << g_Legs[i].gaitPosZ;
        }
        csv << std::endl;
    }

    csv.close();
    std::cout << "Gait Tests Complete. Snapshot saved to test/golden_data/gait_snapshot.csv" << std::endl;
}

#endif
