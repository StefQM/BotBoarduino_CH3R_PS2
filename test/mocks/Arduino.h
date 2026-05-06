#ifndef ARDUINO_H_MOCK
#define ARDUINO_H_MOCK

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <vector>

// Basic Arduino Types
typedef uint8_t  byte;
typedef uint16_t word;
typedef bool     boolean;

#define PROGMEM
#define pgm_read_byte(addr) (*(const uint8_t*)(addr))
#define pgm_read_word(addr) (*(const uint16_t*)(addr))
#define pgm_read_dword(addr) (*(const uint32_t*)(addr))

#define LOW  0x0
#define HIGH 0x1
#define INPUT 0x0
#define OUTPUT 0x1

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

// Mock Timing functions (to be controlled by testbench)
extern uint32_t g_mock_millis;
extern uint32_t g_mock_micros;

inline uint32_t millis() { 
    g_mock_millis++; // Auto-increment to prevent infinite loops
    return g_mock_millis; 
}
inline uint32_t micros() { 
    g_mock_micros += 10; // Auto-increment to prevent infinite loops
    return g_mock_micros; 
}
inline void delay(uint32_t ms) { g_mock_millis += ms; g_mock_micros += (ms * 1000); }
inline void delayMicroseconds(uint32_t us) { g_mock_micros += us; }

// Mock Serial
class MockSerial {
public:
    void begin(uint32_t baud) {}
    void print(const char* s) { std::cout << s; }
    void print(int n, int base = DEC) { std::cout << n; }
    void println(const char* s = "") { std::cout << s << std::endl; }
    void println(int n, int base = DEC) { std::cout << n << std::endl; }
    void write(const char* s) { std::cout << s; }
    void write(uint8_t b) { std::cout << (char)b; }
    void write(const uint8_t* buf, size_t len) { std::cout.write((const char*)buf, len); }
    int read() { return -1; }
    int available() { return 0; }
    void flush() {}
};

extern MockSerial Serial;
#ifdef DBGSerial
#undef DBGSerial
#endif
#define DBGSerial Serial

// Math helpers
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

// Mock Pins
inline void pinMode(uint8_t pin, uint8_t mode) {}
inline int digitalRead(uint8_t pin) { return LOW; }
inline void digitalWrite(uint8_t pin, uint8_t val) {}

// Mock AVR macros
#define digitalPinToPort(p) (p)
#define portOutputRegister(p) (&g_mock_port)
#define digitalPinToBitMask(p) (1 << (p % 8))
extern uint8_t g_mock_port;

#endif // ARDUINO_H_MOCK
