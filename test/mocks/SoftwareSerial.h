#ifndef SOFTWARE_SERIAL_H_MOCK
#define SOFTWARE_SERIAL_H_MOCK

#include "Arduino.h"
#include <string>

class SoftwareSerial {
public:
    SoftwareSerial(uint8_t receivePin, uint8_t transmitPin) {}
    void begin(uint32_t speed) {}
    void listen() {}
    
    size_t print(const char* s) { 
        lastOutput += s;
        return strlen(s); 
    }
    size_t print(int n, int base = 10) {
        std::string s = std::to_string(n);
        lastOutput += s;
        return s.length();
    }
    size_t println(const char* s = "") {
        size_t n = print(s);
        lastOutput += "\r\n";
        return n + 2;
    }
    size_t println(int n, int base = 10) {
        size_t len = print(n, base);
        lastOutput += "\r\n";
        return len + 2;
    }
    size_t write(uint8_t b) {
        lastOutput += (char)b;
        return 1;
    }
    size_t write(const uint8_t* buf, size_t len) {
        lastOutput.append((const char*)buf, len);
        return len;
    }
    
    int available() { return 0; }
    int read() { return -1; }
    void flush() {}

    // Test helper
    static std::string lastOutput;
    static void clear() { lastOutput = ""; }
};

#endif // SOFTWARE_SERIAL_H_MOCK
