#ifndef PS2X_LIB_H_MOCK
#define PS2X_LIB_H_MOCK

#include "Arduino.h"

#define PSB_SELECT      0x0001
#define PSB_L3          0x0002
#define PSB_R3          0x0004
#define PSB_START       0x0008
#define PSB_PAD_UP      0x0010
#define PSB_PAD_RIGHT   0x0020
#define PSB_PAD_DOWN    0x0040
#define PSB_PAD_LEFT    0x0080
#define PSB_L2          0x0100
#define PSB_R2          0x0200
#define PSB_L1          0x0400
#define PSB_R1          0x0800
#define PSB_TRIANGLE    0x1000
#define PSB_CIRCLE      0x2000
#define PSB_CROSS       0x4000
#define PSB_SQUARE      0x8000

#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

class PS2X {
public:
    uint8_t config_gamepad(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool press = false, bool rumble = false) { return 0; }
    void read_gamepad(bool motor1 = false, uint8_t motor2 = 0) {}
    
    // Test helpers to inject state
    uint16_t buttons = 0xFFFF; // All up
    uint16_t last_buttons = 0xFFFF;
    uint8_t analogs[21];

    PS2X() {
        for(int i=0; i<21; i++) analogs[i] = 128; // Centered
    }

    bool Button(uint16_t button) { return !(buttons & button); }
    bool ButtonPressed(uint16_t button) { return !(buttons & button) && (last_buttons & button); }
    uint8_t Analog(uint8_t button) { return analogs[button]; }
    
    void reconfig_gamepad() {}
};

#endif // PS2X_LIB_H_MOCK
