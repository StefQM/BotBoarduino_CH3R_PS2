#include "PS2X_lib.h"

#define CTRL_CLK 20
#define CTRL_BYTE_DELAY 50

uint8_t PS2X::config_gamepad(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures, bool rumble) {
    _clk = clk; _cmd = cmd; _att = att; _dat = dat;
    
    pinMode(_clk, OUTPUT);
    pinMode(_att, OUTPUT);
    pinMode(_cmd, OUTPUT);
    pinMode(_dat, INPUT_PULLUP);
    
    digitalWrite(_clk, HIGH);
    digitalWrite(_att, HIGH);
    
    Serial.println("[PS2] Starting initialization...");
    delay(500); // Give the receiver plenty of time to stabilize
    
    for (int retry = 0; retry < 10; retry++) {
        // 1. Enter Config Mode
        digitalWrite(_att, LOW);
        delayMicroseconds(CTRL_BYTE_DELAY);
        _shiftinout(0x01);
        _shiftinout(0x43); 
        _shiftinout(0x00);
        _shiftinout(0x01);
        _shiftinout(0x00);
        digitalWrite(_att, HIGH);
        delay(20);
        
        // 2. Set to Analog Mode (Lock it)
        digitalWrite(_att, LOW);
        delayMicroseconds(CTRL_BYTE_DELAY);
        _shiftinout(0x01);
        _shiftinout(0x44);
        _shiftinout(0x00);
        _shiftinout(0x01); // 0x01 = Analog
        _shiftinout(0x03); // 0x03 = Lock
        _shiftinout(0x00);
        _shiftinout(0x00);
        _shiftinout(0x00);
        _shiftinout(0x00);
        digitalWrite(_att, HIGH);
        delay(20);

        // 3. Exit Config Mode
        digitalWrite(_att, LOW);
        delayMicroseconds(CTRL_BYTE_DELAY);
        _shiftinout(0x01);
        _shiftinout(0x43);
        _shiftinout(0x00);
        _shiftinout(0x00);
        _shiftinout(0x5A);
        _shiftinout(0x5A);
        _shiftinout(0x5A);
        _shiftinout(0x5A);
        _shiftinout(0x5A);
        digitalWrite(_att, HIGH);
        delay(50);
        
        read_gamepad();
        
        if ((_analog[1] & 0xF0) == 0x70) {
            Serial.print("[PS2] Initialization SUCCESS on attempt ");
            Serial.println(retry + 1);
            return 0;
        }
        
        Serial.print("[PS2] Attempt ");
        Serial.print(retry + 1);
        Serial.print(" failed. ID: 0x");
        Serial.println(_analog[1], HEX);
        delay(100);
    }

    Serial.println("[PS2] ALL INITIALIZATION ATTEMPTS FAILED.");
    return 1;
}

uint8_t PS2X::_shiftinout(uint8_t byte) {
    uint8_t res = 0;
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(_cmd, (byte & (1 << i)) ? HIGH : LOW);
        digitalWrite(_clk, LOW);
        delayMicroseconds(CTRL_CLK);
        if (digitalRead(_dat)) res |= (1 << i);
        digitalWrite(_clk, HIGH);
        delayMicroseconds(CTRL_CLK);
    }
    return res;
}

bool PS2X::read_gamepad(bool motor1, uint8_t motor2) {
    _last_buttons = _buttons;
    
    digitalWrite(_att, LOW);
    delayMicroseconds(CTRL_BYTE_DELAY);
    
    _shiftinout(0x01); // Start
    uint8_t id = _shiftinout(0x42); // Read data
    _shiftinout(0x00); // Dummy
    
    uint8_t b1 = _shiftinout(0x00);
    uint8_t b2 = _shiftinout(0x00);
    _buttons = (b2 << 8) | b1;
    
    _analog[PSS_RX] = _shiftinout(0x00);
    _analog[PSS_RY] = _shiftinout(0x00);
    _analog[PSS_LX] = _shiftinout(0x00);
    _analog[PSS_LY] = _shiftinout(0x00);
    
    _analog[1] = id; // Store ID in the mode slot for verification
    
    digitalWrite(_att, HIGH);
    
    return true;
}

bool PS2X::Button(uint16_t button) {
    return ~(_buttons) & button;
}

bool PS2X::ButtonPressed(uint16_t button) {
    return NewButtonState(button) && Button(button);
}

bool PS2X::ButtonReleased(uint16_t button) {
    return NewButtonState(button) && !Button(button);
}

bool PS2X::NewButtonState() {
    return (_buttons ^ _last_buttons) != 0;
}

bool PS2X::NewButtonState(uint16_t button) {
    return (_buttons ^ _last_buttons) & button;
}

uint8_t PS2X::Analog(uint8_t button) {
    return _analog[button];
}

void PS2X::reconfig_gamepad() {
    config_gamepad(_clk, _cmd, _att, _dat);
}
