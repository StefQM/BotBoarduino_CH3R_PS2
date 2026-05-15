#include "SoftwareSerial.h"

SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic) :
    _receivePin(receivePin), _transmitPin(transmitPin), _rx_buffer_head(0), _rx_buffer_tail(0) {
    pinMode(_transmitPin, OUTPUT);
    digitalWrite(_transmitPin, HIGH);
    pinMode(_receivePin, INPUT_PULLUP);
}

SoftwareSerial::~SoftwareSerial() {
    end();
}

void SoftwareSerial::begin(long speed) {
    _bitDelay = 1000000L / speed;
}

void SoftwareSerial::listen() {
    // Basic polling-based receiver doesn't need much state
}

void SoftwareSerial::end() {
}

void SoftwareSerial::flush() {
    _rx_buffer_head = _rx_buffer_tail = 0;
}

size_t SoftwareSerial::write(uint8_t b) {
    uint32_t wait = _bitDelay;
    
    // Start bit
    digitalWrite(_transmitPin, LOW);
    delayMicroseconds(wait);
    
    // Data bits
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(_transmitPin, (b & (1 << i)) ? HIGH : LOW);
        delayMicroseconds(wait);
    }
    
    // Stop bit
    digitalWrite(_transmitPin, HIGH);
    delayMicroseconds(wait);
    
    return 1;
}

int SoftwareSerial::available() {
    // This is a minimal blocking-read implementation for the SSC-32 response.
    // In a full implementation, this would use interrupts. 
    // For the Phoenix code, we mostly care about 'write'.
    return (_rx_buffer_head + RX_BUF_SIZE - _rx_buffer_tail) % RX_BUF_SIZE;
}

int SoftwareSerial::read() {
    if (_rx_buffer_head == _rx_buffer_tail) return -1;
    uint8_t b = _rx_buffer[_rx_buffer_tail];
    _rx_buffer_tail = (_rx_buffer_tail + 1) % RX_BUF_SIZE;
    return b;
}

int SoftwareSerial::peek() {
    if (_rx_buffer_head == _rx_buffer_tail) return -1;
    return _rx_buffer[_rx_buffer_tail];
}
