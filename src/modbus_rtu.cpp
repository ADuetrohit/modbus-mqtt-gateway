#include "modbus_rtu.h"

uint16_t modbusCrc16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 1) {
        crc = (uint16_t)((crc >> 1) ^ 0xA001);
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

const char *mbResultName(MbResult result) {
  switch (result) {
    case MbResult::Ok:        return "OK";
    case MbResult::Timeout:   return "TIMEOUT";
    case MbResult::CrcError:  return "CRC_ERROR";
    case MbResult::Exception: return "EXCEPTION";
    case MbResult::BadReply:  return "BAD_REPLY";
  }
  return "UNKNOWN";
}

ModbusRtuMaster::ModbusRtuMaster(HardwareSerial &serial, uint32_t baud,
                                 uint32_t responseTimeoutMs)
    : _serial(serial), _baud(baud), _responseTimeoutMs(responseTimeoutMs) {
  // The spec fixes the inter-frame gap at 1.75 ms above 19200 baud; below
  // that it is 3.5 character times.
  if (baud > 19200) {
    _interFrameMicros = 1750;
  } else {
    _interFrameMicros = (35UL * 11UL * 1000000UL) / (baud * 10UL);
  }
}

void ModbusRtuMaster::begin(int8_t rxPin, int8_t txPin) {
  _serial.begin(_baud, SERIAL_8N1, rxPin, txPin);
}

MbResult ModbusRtuMaster::readInputRegisters(uint8_t slave, uint16_t start,
                                             uint16_t qty, uint16_t *out) {
  return readRegisters(0x04, slave, start, qty, out);
}

MbResult ModbusRtuMaster::readHoldingRegisters(uint8_t slave, uint16_t start,
                                               uint16_t qty, uint16_t *out) {
  return readRegisters(0x03, slave, start, qty, out);
}

size_t ModbusRtuMaster::receiveFrame(uint8_t *buf, size_t maxLen) {
  size_t len = 0;
  uint32_t deadline = millis() + _responseTimeoutMs;

  // Wait for the first byte within the response timeout.
  while (!_serial.available()) {
    if ((int32_t)(millis() - deadline) >= 0) {
      return 0;
    }
    delay(1);
  }

  // Then read until the line has been quiet for t3.5, which ends the frame.
  uint32_t lastByteMicros = micros();
  while ((micros() - lastByteMicros) < _interFrameMicros) {
    if (_serial.available()) {
      if (len < maxLen) {
        buf[len++] = (uint8_t)_serial.read();
      } else {
        _serial.read();  // drop the overflow but keep draining the line
      }
      lastByteMicros = micros();
    }
  }
  return len;
}

MbResult ModbusRtuMaster::readRegisters(uint8_t function, uint8_t slave,
                                        uint16_t start, uint16_t qty,
                                        uint16_t *out) {
  _lastExceptionCode = 0;

  uint8_t request[8];
  request[0] = slave;
  request[1] = function;
  request[2] = (uint8_t)(start >> 8);
  request[3] = (uint8_t)(start & 0xFF);
  request[4] = (uint8_t)(qty >> 8);
  request[5] = (uint8_t)(qty & 0xFF);
  uint16_t crc = modbusCrc16(request, 6);
  request[6] = (uint8_t)(crc & 0xFF);
  request[7] = (uint8_t)(crc >> 8);

  // Discard anything stale before transmitting.
  while (_serial.available()) {
    _serial.read();
  }

  _serial.write(request, sizeof(request));
  _serial.flush();

  uint8_t reply[256];
  size_t replyLen = receiveFrame(reply, sizeof(reply));
  if (replyLen == 0) {
    return MbResult::Timeout;
  }
  if (replyLen < 4) {
    return MbResult::BadReply;
  }

  uint16_t receivedCrc =
      (uint16_t)reply[replyLen - 2] | ((uint16_t)reply[replyLen - 1] << 8);
  if (modbusCrc16(reply, replyLen - 2) != receivedCrc) {
    return MbResult::CrcError;
  }
  if (reply[0] != slave) {
    return MbResult::BadReply;
  }
  if (reply[1] == (function | 0x80)) {
    _lastExceptionCode = reply[2];
    return MbResult::Exception;
  }
  if (reply[1] != function) {
    return MbResult::BadReply;
  }

  uint8_t byteCount = reply[2];
  if (byteCount != qty * 2 || replyLen < (size_t)(3 + byteCount + 2)) {
    return MbResult::BadReply;
  }

  for (uint16_t i = 0; i < qty; i++) {
    out[i] = ((uint16_t)reply[3 + i * 2] << 8) | reply[4 + i * 2];
  }
  return MbResult::Ok;
}
