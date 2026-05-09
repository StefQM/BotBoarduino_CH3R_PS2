#ifndef TEST_INPUT_H
#define TEST_INPUT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "common.h"
#include "test_utils.h"

extern PS2X ps2x;

void set_ps2_joysticks(uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry) {
    ps2x.analogs[PSS_LX] = lx;
    ps2x.analogs[PSS_LY] = ly;
    ps2x.analogs[PSS_RX] = rx;
    ps2x.analogs[PSS_RY] = ry;
}

void click_ps2_button(uint16_t button) {
    // Button pressed
    ps2x.last_buttons = 0xFFFF;
    ps2x.buttons = 0xFFFF & ~button;
    g_InputController.ControlInput();
    
    // Button released
    ps2x.last_buttons = ps2x.buttons;
    ps2x.buttons = 0xFFFF;
    g_InputController.ControlInput();
}

void snapshot_input(std::ofstream& csv, const std::string& label) {
    csv << label << ","
        << (int)g_InControlState.fHexOn << ","
        << g_InControlState.BodyPos.x << ","
        << g_InControlState.BodyPos.y << ","
        << g_InControlState.BodyPos.z << ","
        << g_InControlState.BodyRot1.x << ","
        << g_InControlState.BodyRot1.y << ","
        << g_InControlState.BodyRot1.z << ","
        << (int)g_InControlState.GaitType << ","
        << g_InControlState.LegLiftHeight << ","
        << g_InControlState.TravelLength.x << ","
        << g_InControlState.TravelLength.y << ","
        << g_InControlState.TravelLength.z << ","
        << (int)g_InControlState.SelectedLeg << ","
        << g_InControlState.SLLeg.x << ","
        << g_InControlState.SLLeg.y << ","
        << g_InControlState.SLLeg.z << ","
        << (int)g_InControlState.fSLHold << ","
        << (int)g_InControlState.BalanceMode << ","
        << (int)g_InControlState.InputTimeDelay << ","
        << (int)g_InControlState.SpeedControl << "\n";
}

bool run_input_tests() {
    std::cout << "Running InputController Regression Tests..." << std::endl;
    std::ofstream csv("test/golden_data/input_snapshot_current.csv");
    csv << "Label,fHexOn,BodyPosX,BodyPosY,BodyPosZ,BodyRotX,BodyRotY,BodyRotZ,GaitType,LegLiftHeight,TravelLengthX,TravelLengthY,TravelLengthZ,SelectedLeg,SLLegX,SLLegY,SLLegZ,fSLHold,BalanceMode,InputTimeDelay,SpeedControl\n";

    // 1. Init
    g_InputController.Init();
    snapshot_input(csv, "Init");

    // 2. Turn On
    click_ps2_button(PSB_START);
    snapshot_input(csv, "TurnOn");

    // 3. Joystick sweeps in WALKMODE (Method 1)
    set_ps2_joysticks(0, 128, 128, 128); g_InputController.ControlInput(); snapshot_input(csv, "Walk_LX_min");
    set_ps2_joysticks(255, 128, 128, 128); g_InputController.ControlInput(); snapshot_input(csv, "Walk_LX_max");
    set_ps2_joysticks(128, 0, 128, 128); g_InputController.ControlInput(); snapshot_input(csv, "Walk_LY_min");
    set_ps2_joysticks(128, 255, 128, 128); g_InputController.ControlInput(); snapshot_input(csv, "Walk_LY_max");
    set_ps2_joysticks(128, 128, 0, 128); g_InputController.ControlInput(); snapshot_input(csv, "Walk_RX_min");
    set_ps2_joysticks(128, 128, 255, 128); g_InputController.ControlInput(); snapshot_input(csv, "Walk_RX_max");
    set_ps2_joysticks(128, 128, 128, 0); g_InputController.ControlInput(); snapshot_input(csv, "Walk_RY_min");
    set_ps2_joysticks(128, 128, 128, 255); g_InputController.ControlInput(); snapshot_input(csv, "Walk_RY_max");
    set_ps2_joysticks(128, 128, 128, 128); // Reset

    // 4. Toggle DoubleTravel (R2), sweep
    click_ps2_button(PSB_R2);
    set_ps2_joysticks(0, 0, 128, 128); g_InputController.ControlInput(); snapshot_input(csv, "DoubleTravel_Sweep");
    set_ps2_joysticks(128, 128, 128, 128); // Reset

    // 5. Toggle WalkMethod (R3), sweep
    click_ps2_button(PSB_R3);
    set_ps2_joysticks(128, 128, 128, 0); g_InputController.ControlInput(); snapshot_input(csv, "WalkMethod2_RY_min");
    set_ps2_joysticks(128, 128, 128, 255); g_InputController.ControlInput(); snapshot_input(csv, "WalkMethod2_RY_max");
    set_ps2_joysticks(128, 128, 128, 128); // Reset
    click_ps2_button(PSB_R3); // Back to Method 1

    // 6. Toggle DoubleHeight (R1)
    click_ps2_button(PSB_R1);
    snapshot_input(csv, "DoubleHeight_On");
    click_ps2_button(PSB_R1);
    snapshot_input(csv, "DoubleHeight_Off");

    // 7. Cycle Gaits (Select)
    for(int i=0; i<NUM_GAITS + 1; i++) {
        click_ps2_button(PSB_SELECT);
        snapshot_input(csv, "GaitCycle_" + std::to_string(i));
    }

    // 8. Adjust Speed (Pad Left/Right)
    click_ps2_button(PSB_PAD_LEFT);
    snapshot_input(csv, "Speed_Slower");
    click_ps2_button(PSB_PAD_RIGHT);
    snapshot_input(csv, "Speed_Faster");

    // 9. Toggle Sit/Stand (Triangle)
    click_ps2_button(PSB_TRIANGLE);
    snapshot_input(csv, "SitStand_Toggle1");
    click_ps2_button(PSB_TRIANGLE);
    snapshot_input(csv, "SitStand_Toggle2");

    // 10. Adjust BodyHeight (Pad Up/Down)
    click_ps2_button(PSB_PAD_UP);
    snapshot_input(csv, "BodyHeight_Up");
    click_ps2_button(PSB_PAD_DOWN);
    snapshot_input(csv, "BodyHeight_Down");

    // 11. Toggle Balance (Square)
    click_ps2_button(PSB_SQUARE);
    snapshot_input(csv, "Balance_On");
    click_ps2_button(PSB_SQUARE);
    snapshot_input(csv, "Balance_Off");

    // 12. Switch to TRANSLATEMODE (L1), sweep
    click_ps2_button(PSB_L1);
    set_ps2_joysticks(0, 0, 255, 0); g_InputController.ControlInput(); snapshot_input(csv, "Translate_Sweep");
    set_ps2_joysticks(128, 128, 128, 128); // Reset
    click_ps2_button(PSB_L1); // Back to WALKMODE

    // 13. Switch to ROTATEMODE (L2), sweep
    click_ps2_button(PSB_L2);
    set_ps2_joysticks(0, 0, 255, 0); g_InputController.ControlInput(); snapshot_input(csv, "Rotate_Sweep");
    set_ps2_joysticks(128, 128, 128, 128); // Reset
    click_ps2_button(PSB_L2); // Back to WALKMODE

    // 14. Switch to SINGLELEGMODE (Circle), cycle legs (Select), sweep, toggle SLHold (R2)
    click_ps2_button(PSB_CIRCLE);
    snapshot_input(csv, "SingleLeg_On");
    for(int i=0; i<6; i++) {
        click_ps2_button(PSB_SELECT);
        snapshot_input(csv, "SingleLeg_Cycle_" + std::to_string(i));
    }
    set_ps2_joysticks(0, 0, 128, 255); g_InputController.ControlInput(); snapshot_input(csv, "SingleLeg_Sweep");
    set_ps2_joysticks(128, 128, 128, 128); // Reset
    click_ps2_button(PSB_R2);
    snapshot_input(csv, "SingleLeg_Hold_On");
    click_ps2_button(PSB_CIRCLE); // Back to WALKMODE

    // 15. Switch to GPPLAYERMODE (Cross), cycle seqs, start seq
    click_ps2_button(PSB_CROSS);
    snapshot_input(csv, "GPPlayer_On");
    for(int i=0; i<6; i++) {
        click_ps2_button(PSB_SELECT);
        snapshot_input(csv, "GPPlayer_Cycle_" + std::to_string(i));
    }
    click_ps2_button(PSB_R2);
    snapshot_input(csv, "GPPlayer_StartSeq");
    click_ps2_button(PSB_CROSS); // Back to WALKMODE

    // 16. Turn Off (Start)
    click_ps2_button(PSB_START);
    snapshot_input(csv, "TurnOff");

    // 17. Simulate PS2 connection loss (invalid analog mode)
    ps2x.analogs[1] = 0x00; // Invalid mode
    for(int i=0; i<6; i++) {
        g_InputController.ControlInput();
        snapshot_input(csv, "ConnLoss_Tick_" + std::to_string(i));
    }
    ps2x.analogs[1] = 0x70; // Restore mode

    csv.close();

    // Comparison logic
    std::ifstream goldFile("test/golden_data/input_snapshot.csv");
    if (!goldFile.is_open()) {
        std::cout << "No golden baseline found. Generated new current snapshot." << std::endl;
        return true; // First run success
    }

    std::ifstream currFile("test/golden_data/input_snapshot_current.csv");
    std::string goldLine, currLine;
    int lineNum = 0;
    bool match = true;
    while (std::getline(goldFile, goldLine) && std::getline(currFile, currLine)) {
        lineNum++;
        if (goldLine != currLine) {
            std::cout << "Mismatch at line " << lineNum << ":" << std::endl;
            std::cout << "  Gold: " << goldLine << std::endl;
            std::cout << "  Curr: " << currLine << std::endl;
            match = false;
        }
    }
    
    if (std::getline(goldFile, goldLine) || std::getline(currFile, currLine)) {
        std::cout << "File length mismatch!" << std::endl;
        match = false;
    }

    if (match) {
        std::cout << "[SUCCESS] INPUT REGRESSION PASSED." << std::endl;
    } else {
        std::cout << "[ERROR] INPUT REGRESSION FAILED!" << std::endl;
    }

    return match;
}

#endif
