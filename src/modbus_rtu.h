// Minimal Modbus RTU master for ESP32, written against the spec rather than
// pulled from a library, so the framing and error handling stay visible.
#pragma once

#include <Arduino.h>

enum class MbResult : uint8_t {
  Ok = 0,
  Timeout,       // slave never answered
  CrcError,      // reply arrived corrupted
  Exception,     // slave answered with a Modbus exception code
  BadReply,      // wrong address/function/length echoed back
};

const char *mbResultName(MbResult result);

class ModbusRtuMaster {
 public:
  ModbusRtuMaster(HardwareSerial &serial, uint32_t baud,
                  uint32_t responseTimeoutMs = 300);

  void begin(int8_t rxPin, int8_t txPin);

  // Function 0x04. `out` must hold at least `qty` words.
  MbResult readInputRegisters(uint8_t slave, uint16_t start, uint16_t qty,
                              uint16_t *out);

  // Function 0x03.
  MbResult readHoldingRegisters(uint8_t slave, uint16_t start, uint16_t qty,
                                uint16_t *out);

  uint8_t lastExceptionCode() const { return _lastExceptionCode; }

 private:
  MbResult readRegisters(uint8_t function, uint8_t slave, uint16_t start,
                         uint16_t qty, uint16_t *out);
  size_t receiveFrame(uint8_t *buf, size_t maxLen);

  HardwareSerial &_serial;
  uint32_t _baud;
  uint32_t _responseTimeoutMs;
  uint32_t _interFrameMicros;
  uint8_t _lastExceptionCode = 0;
};

uint16_t modbusCrc16(const uint8_t *buf, size_t len);
