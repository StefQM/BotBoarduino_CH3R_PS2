#ifndef SoftSerial_h
#define SoftSerial_h

#include <Arduino.h>

class SoftwareSerial : public Stream {
public:
    SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
    ~SoftwareSerial();

    void begin(long speed);
    void listen();
    void end();
    bool isListening() { return true; }
    void flush();
    
    virtual int peek();
    virtual size_t write(uint8_t byte);
    virtual int read();
    virtual int available();

    using Print::write;

private:
    uint8_t _receivePin;
    uint8_t _transmitPin;
    uint32_t _bitDelay;
    
    // Minimal receive buffer for basic response handling
    static const int RX_BUF_SIZE = 64;
    uint8_t _rx_buffer[RX_BUF_SIZE];
    volatile int _rx_buffer_head;
    volatile int _rx_buffer_tail;
};

#endif
