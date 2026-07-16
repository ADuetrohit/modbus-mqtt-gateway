// Wokwi custom chip: 3-phase-style AC energy meter, Modbus RTU slave.
//
// Speaks real Modbus RTU on a 9600 8N1 UART: function 0x03 (holding) and
// 0x04 (input) register reads, CRC-16/MODBUS, t3.5 inter-frame framing,
// exception responses, and a response turnaround delay.
//
// The faultMode attribute injects failures so the gateway's retry and
// timeout paths can be exercised in CI:
//   0 = healthy
//   1 = silent (meter does not answer -> master must time out and retry)
//   2 = corrupted CRC on the reply -> master must reject the frame
//
// Input register map (0-based, big-endian, 16-bit words):
//   0      voltage        V   x10
//   1      current        A   x10
//   2..3   active power   W   (u32, high word first)
//   4      frequency      Hz  x10
//   5      power factor       x100
//   6..7   active energy  Wh  (u32, high word first, integrated live)

#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MB_MAX_FRAME 256
#define REG_COUNT 8

#define MB_FN_READ_HOLDING 0x03
#define MB_FN_READ_INPUT 0x04

#define MB_EX_ILLEGAL_FUNCTION 0x01
#define MB_EX_ILLEGAL_ADDRESS 0x02
#define MB_EX_ILLEGAL_VALUE 0x03

#define FAULT_NONE 0
#define FAULT_SILENT 1
#define FAULT_BAD_CRC 2

// 9600 baud, 8N1 -> 11 bits/char -> 1.146 ms/char. t3.5 ~= 4.01 ms.
#define BAUD_RATE 9600
#define T35_MICROS 4010

// Modbus requires the slave to wait at least t3.5 before answering. Real
// meters take longer; 5 ms keeps the master's timeout path meaningful.
#define RESPONSE_DELAY_MICROS 5000

#define ENERGY_TICK_MICROS 1000000

typedef struct {
  uart_dev_t uart;
  timer_t t35_timer;
  timer_t resp_timer;
  timer_t energy_timer;

  uint32_t attr_voltage;
  uint32_t attr_current;
  uint32_t attr_fault;
  uint32_t attr_address;

  uint8_t rx_buf[MB_MAX_FRAME];
  uint32_t rx_len;
  bool overflow;

  uint8_t tx_buf[MB_MAX_FRAME];
  uint32_t tx_len;

  // Energy integrated in hundredths of a watt-hour to keep resolution
  // without floating point.
  uint64_t energy_wh_x100;
} chip_state_t;

static uint16_t mb_crc16(const uint8_t *buf, uint32_t len) {
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < len; i++) {
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

static uint32_t meter_power_w(chip_state_t *chip) {
  uint32_t volt_x10 = attr_read(chip->attr_voltage) * 10;
  uint32_t amp_x10 = attr_read(chip->attr_current);
  // (V*10 * A*10) / 100 = VA, then x0.95 power factor.
  uint64_t va = ((uint64_t)volt_x10 * (uint64_t)amp_x10) / 100ULL;
  return (uint32_t)((va * 95ULL) / 100ULL);
}

static void meter_build_registers(chip_state_t *chip, uint16_t *regs) {
  uint32_t power_w = meter_power_w(chip);
  uint32_t energy_wh = (uint32_t)(chip->energy_wh_x100 / 100ULL);

  regs[0] = (uint16_t)(attr_read(chip->attr_voltage) * 10);
  regs[1] = (uint16_t)attr_read(chip->attr_current);
  regs[2] = (uint16_t)(power_w >> 16);
  regs[3] = (uint16_t)(power_w & 0xFFFF);
  regs[4] = 500;  // 50.0 Hz
  regs[5] = 95;   // 0.95 power factor
  regs[6] = (uint16_t)(energy_wh >> 16);
  regs[7] = (uint16_t)(energy_wh & 0xFFFF);
}

static void queue_exception(chip_state_t *chip, uint8_t function, uint8_t code) {
  chip->tx_buf[0] = (uint8_t)attr_read(chip->attr_address);
  chip->tx_buf[1] = (uint8_t)(function | 0x80);
  chip->tx_buf[2] = code;
  uint16_t crc = mb_crc16(chip->tx_buf, 3);
  chip->tx_buf[3] = (uint8_t)(crc & 0xFF);
  chip->tx_buf[4] = (uint8_t)(crc >> 8);
  chip->tx_len = 5;
}

static void queue_register_reply(chip_state_t *chip, uint8_t function,
                                 uint16_t start, uint16_t qty) {
  uint16_t regs[REG_COUNT];
  meter_build_registers(chip, regs);

  chip->tx_buf[0] = (uint8_t)attr_read(chip->attr_address);
  chip->tx_buf[1] = function;
  chip->tx_buf[2] = (uint8_t)(qty * 2);

  uint32_t pos = 3;
  for (uint16_t i = 0; i < qty; i++) {
    uint16_t value = regs[start + i];
    chip->tx_buf[pos++] = (uint8_t)(value >> 8);
    chip->tx_buf[pos++] = (uint8_t)(value & 0xFF);
  }

  uint16_t crc = mb_crc16(chip->tx_buf, pos);
  if (attr_read(chip->attr_fault) == FAULT_BAD_CRC) {
    crc ^= 0xFFFF;  // deliberately wrong: master must discard the frame
  }
  chip->tx_buf[pos++] = (uint8_t)(crc & 0xFF);
  chip->tx_buf[pos++] = (uint8_t)(crc >> 8);
  chip->tx_len = pos;
}

static void process_frame(chip_state_t *chip) {
  if (chip->rx_len < 4) {
    return;  // too short to hold address + function + CRC
  }

  uint8_t address = chip->rx_buf[0];
  uint8_t own_address = (uint8_t)attr_read(chip->attr_address);
  if (address != own_address && address != 0x00) {
    return;  // addressed to a different slave
  }

  uint16_t received_crc =
      (uint16_t)chip->rx_buf[chip->rx_len - 2] |
      ((uint16_t)chip->rx_buf[chip->rx_len - 1] << 8);
  if (mb_crc16(chip->rx_buf, chip->rx_len - 2) != received_crc) {
    return;  // Modbus says: stay silent on a corrupt request
  }

  uint8_t function = chip->rx_buf[1];
  if (function != MB_FN_READ_HOLDING && function != MB_FN_READ_INPUT) {
    queue_exception(chip, function, MB_EX_ILLEGAL_FUNCTION);
  } else if (chip->rx_len != 8) {
    queue_exception(chip, function, MB_EX_ILLEGAL_VALUE);
  } else {
    uint16_t start = ((uint16_t)chip->rx_buf[2] << 8) | chip->rx_buf[3];
    uint16_t qty = ((uint16_t)chip->rx_buf[4] << 8) | chip->rx_buf[5];

    if (qty < 1 || qty > 125) {
      queue_exception(chip, function, MB_EX_ILLEGAL_VALUE);
    } else if ((uint32_t)start + (uint32_t)qty > REG_COUNT) {
      queue_exception(chip, function, MB_EX_ILLEGAL_ADDRESS);
    } else {
      queue_register_reply(chip, function, start, qty);
    }
  }

  // A broadcast (address 0) is never answered.
  if (address == 0x00) {
    chip->tx_len = 0;
    return;
  }
  if (attr_read(chip->attr_fault) == FAULT_SILENT) {
    chip->tx_len = 0;
    return;
  }
  if (chip->tx_len > 0) {
    timer_start(chip->resp_timer, RESPONSE_DELAY_MICROS, false);
  }
}

static void on_frame_gap(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (!chip->overflow) {
    process_frame(chip);
  }
  chip->rx_len = 0;
  chip->overflow = false;
}

static void on_response_due(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->tx_len > 0) {
    uart_write(chip->uart, chip->tx_buf, chip->tx_len);
    chip->tx_len = 0;
  }
}

static void on_energy_tick(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  // Wh x100 accrued in one second at the present load.
  chip->energy_wh_x100 += ((uint64_t)meter_power_w(chip) * 100ULL) / 3600ULL;
}

static void on_uart_rx(void *user_data, uint8_t byte) {
  chip_state_t *chip = (chip_state_t *)user_data;

  if (chip->rx_len < MB_MAX_FRAME) {
    chip->rx_buf[chip->rx_len++] = byte;
  } else {
    chip->overflow = true;
  }

  // Every byte restarts the t3.5 silence timer; when it finally fires the
  // frame is complete. This is how real Modbus RTU delimits frames.
  timer_start(chip->t35_timer, T35_MICROS, false);
}

static void on_uart_write_done(void *user_data) {
  (void)user_data;  // reply fully shifted out; nothing to chain
}

void chip_init(void) {
  chip_state_t *chip = (chip_state_t *)calloc(1, sizeof(chip_state_t));

  chip->attr_voltage = attr_init("voltage", 230);
  chip->attr_current = attr_init("current", 52);  // 5.2 A
  chip->attr_fault = attr_init("faultMode", FAULT_NONE);
  chip->attr_address = attr_init("slaveAddress", 1);

  const uart_config_t uart_config = {
      .user_data = chip,
      .rx = pin_init("RX", INPUT_PULLUP),
      .tx = pin_init("TX", OUTPUT_HIGH),  // UART line idles high
      .baud_rate = BAUD_RATE,
      .rx_data = on_uart_rx,
      .write_done = on_uart_write_done,
  };
  chip->uart = uart_init(&uart_config);

  const timer_config_t t35_config = {
      .user_data = chip,
      .callback = on_frame_gap,
  };
  chip->t35_timer = timer_init(&t35_config);

  const timer_config_t resp_config = {
      .user_data = chip,
      .callback = on_response_due,
  };
  chip->resp_timer = timer_init(&resp_config);

  const timer_config_t energy_config = {
      .user_data = chip,
      .callback = on_energy_tick,
  };
  chip->energy_timer = timer_init(&energy_config);

  timer_start(chip->energy_timer, ENERGY_TICK_MICROS, true);
}
