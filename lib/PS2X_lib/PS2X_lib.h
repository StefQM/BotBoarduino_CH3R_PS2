#ifndef PS2X_lib_h
#define PS2X_lib_h

#include <Arduino.h>

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
    uint8_t config_gamepad(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures = false, bool rumble = false);
    bool read_gamepad(bool motor1 = false, uint8_t motor2 = 0);
    void reconfig_gamepad();
    
    bool Button(uint16_t button);
    bool ButtonPressed(uint16_t button);
    bool ButtonReleased(uint16_t button);
    bool NewButtonState();
    bool NewButtonState(uint16_t button);
    
    uint8_t Analog(uint8_t button);

private:
    uint8_t _clk;
    uint8_t _cmd;
    uint8_t _att;
    uint8_t _dat;
    
    uint16_t _buttons;
    uint16_t _last_buttons;
    uint8_t _analog[21];
    
    uint8_t _shiftinout(uint8_t byte);
};

#endif
