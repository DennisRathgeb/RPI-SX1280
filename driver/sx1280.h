/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SX1280 LoRa Radio Driver Header
 *
 * Copyright (C) 2025
 */

#ifndef __SX1280_DRIVER_H__
#define __SX1280_DRIVER_H__

#include <linux/types.h>

/*
 * ============================================================================
 * SX1280 Command Opcodes
 * ============================================================================
 */

/* Radio Status and Control */
#define RADIO_GET_STATUS                    0xC0
#define RADIO_WRITE_REGISTER                0x18
#define RADIO_READ_REGISTER                 0x19
#define RADIO_WRITE_BUFFER                  0x1A
#define RADIO_READ_BUFFER                   0x1B

/* Operating Mode Commands */
#define RADIO_SET_SLEEP                     0x84
#define RADIO_SET_STANDBY                   0x80
#define RADIO_SET_FS                        0xC1
#define RADIO_SET_TX                        0x83
#define RADIO_SET_RX                        0x82
#define RADIO_SET_RXDUTYCYCLE               0x94
#define RADIO_SET_CAD                       0xC5
#define RADIO_SET_TXCONTINUOUSWAVE          0xD1
#define RADIO_SET_TXCONTINUOUSPREAMBLE      0xD2

/* Configuration Commands */
#define RADIO_SET_PACKETTYPE                0x8A
#define RADIO_GET_PACKETTYPE                0x03
#define RADIO_SET_RFFREQUENCY               0x86
#define RADIO_SET_TXPARAMS                  0x8E
#define RADIO_SET_CADPARAMS                 0x88
#define RADIO_SET_BUFFERBASEADDRESS         0x8F
#define RADIO_SET_MODULATIONPARAMS          0x8B
#define RADIO_SET_PACKETPARAMS              0x8C

/* Status and Statistics Commands */
#define RADIO_GET_RXBUFFERSTATUS            0x17
#define RADIO_GET_PACKETSTATUS              0x1D
#define RADIO_GET_RSSIINST                  0x1F

/* IRQ Commands */
#define RADIO_SET_DIOIRQPARAMS              0x8D
#define RADIO_GET_IRQSTATUS                 0x15
#define RADIO_CLR_IRQSTATUS                 0x97

/* Calibration and Configuration */
#define RADIO_CALIBRATE                     0x89
#define RADIO_SET_REGULATORMODE             0x96
#define RADIO_SET_SAVECONTEXT               0xD5
#define RADIO_SET_AUTOTX                    0x98
#define RADIO_SET_AUTOFS                    0x9E
#define RADIO_SET_LONGPREAMBLE              0x9B
#define RADIO_SET_RANGING_ROLE              0xA3

/*
 * ============================================================================
 * Register Addresses (16-bit)
 * ============================================================================
 */

/* LoRa Packet Parameters */
#define REG_LR_PAYLOADLENGTH                0x0901
#define REG_LR_PACKETPARAMS                 0x0903

/* LoRa Sync Word */
#define REG_LR_SYNCWORDBASEADDRESS1         0x09CE
#define REG_LR_SYNCWORDBASEADDRESS2         0x09D3
#define REG_LR_SYNCWORDBASEADDRESS3         0x09D8

/* CRC and Whitening */
#define REG_LR_CRCSEEDBASEADDR              0x09C8
#define REG_LR_CRCPOLYBASEADDR              0x09C6
#define REG_LR_WHITSEEDBASEADDR             0x09C5

/* Ranging (for future use) */
#define REG_LR_RANGINGIDCHECKLENGTH         0x0931
#define REG_LR_DEVICERANGINGADDR            0x0916
#define REG_LR_REQUESTRANGINGADDR           0x0912
#define REG_LR_RANGINGRESULTBASEADDR        0x0961
#define REG_LR_RANGINGRESULTSFREEZE         0x097F

/* Status */
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB 0x0954

/*
 * ============================================================================
 * IRQ Flags (16-bit bitmasks)
 * ============================================================================
 */

#define IRQ_RADIO_NONE                      0x0000
#define IRQ_TX_DONE                         0x0001
#define IRQ_RX_DONE                         0x0002
#define IRQ_SYNCWORD_VALID                  0x0004
#define IRQ_SYNCWORD_ERROR                  0x0008
#define IRQ_HEADER_VALID                    0x0010
#define IRQ_HEADER_ERROR                    0x0020
#define IRQ_CRC_ERROR                       0x0040
#define IRQ_RANGING_SLAVE_RESPONSE_DONE     0x0080
#define IRQ_RANGING_SLAVE_REQUEST_DISCARDED 0x0100
#define IRQ_RANGING_MASTER_RESULT_VALID     0x0200
#define IRQ_RANGING_MASTER_RESULT_TIMEOUT   0x0400
#define IRQ_RANGING_SLAVE_REQUEST_VALID     0x0800
#define IRQ_CAD_DONE                        0x1000
#define IRQ_CAD_ACTIVITY_DETECTED           0x2000
#define IRQ_RX_TX_TIMEOUT                   0x4000
#define IRQ_PREAMBLE_DETECTED               0x8000
#define IRQ_RADIO_ALL                       0xFFFF

/*
 * ============================================================================
 * Hardware Constants
 * ============================================================================
 */

#define SX1280_XTAL_FREQ                    52000000UL  /* 52 MHz crystal */
#define SX1280_FREQ_STEP                    198.364     /* Hz per step (XTAL_FREQ / 2^18) */

/* Timing Constants (microseconds) */
#define SX1280_RESET_DELAY_MS               20          /* Reset pulse width */
#define SX1280_BOOT_DELAY_MS                50          /* Boot time after reset */
#define SX1280_BUSY_TIMEOUT_MS              100         /* Max wait for BUSY pin */

/*
 * ============================================================================
 * Operating Modes
 * ============================================================================
 */

/* Standby Modes (parameter for RADIO_SET_STANDBY) */
#define STDBY_RC                            0x00    /* Standby with RC oscillator */
#define STDBY_XOSC                          0x01    /* Standby with crystal oscillator */

/* Packet Types (parameter for RADIO_SET_PACKETTYPE) */
#define PACKET_TYPE_GFSK                    0x00
#define PACKET_TYPE_LORA                    0x01
#define PACKET_TYPE_RANGING                 0x02
#define PACKET_TYPE_FLRC                    0x03
#define PACKET_TYPE_BLE                     0x04

/* Regulator Modes (parameter for RADIO_SET_REGULATORMODE) */
#define REGULATOR_LDO                       0x00    /* Use LDO */
#define REGULATOR_DC_DC                     0x01    /* Use DC-DC converter */

/*
 * ============================================================================
 * LoRa Modulation Parameters
 * ============================================================================
 */

/* Spreading Factor */
#define LORA_SF5                            0x05
#define LORA_SF6                            0x06
#define LORA_SF7                            0x07
#define LORA_SF8                            0x08
#define LORA_SF9                            0x09
#define LORA_SF10                           0x0A
#define LORA_SF11                           0x0B
#define LORA_SF12                           0x0C

/* Bandwidth */
#define LORA_BW_203                         0x0A    /* 203.125 kHz */
#define LORA_BW_406                         0x18    /* 406.25 kHz */
#define LORA_BW_812                         0x26    /* 812.5 kHz */
#define LORA_BW_1625                        0x34    /* 1625 kHz */

/* Coding Rate */
#define LORA_CR_4_5                         0x01    /* 4/5 */
#define LORA_CR_4_6                         0x02    /* 4/6 */
#define LORA_CR_4_7                         0x03    /* 4/7 */
#define LORA_CR_4_8                         0x04    /* 4/8 */

/* Header Type */
#define LORA_HEADER_EXPLICIT                0x00
#define LORA_HEADER_IMPLICIT                0x01

/* CRC Mode */
#define LORA_CRC_DISABLED                   0x00
#define LORA_CRC_ENABLED                    0x01

/* IQ Mode */
#define LORA_IQ_STANDARD                    0x00
#define LORA_IQ_INVERTED                    0x01

/*
 * ============================================================================
 * TX Parameters
 * ============================================================================
 */

/* TX Power Range */
#define SX1280_TX_POWER_MIN                 -18     /* dBm */
#define SX1280_TX_POWER_MAX                 13      /* dBm */

/* Ramp Time */
#define RAMP_TIME_02_US                     0x00
#define RAMP_TIME_04_US                     0x20
#define RAMP_TIME_06_US                     0x40
#define RAMP_TIME_08_US                     0x60
#define RAMP_TIME_10_US                     0x80
#define RAMP_TIME_12_US                     0xA0
#define RAMP_TIME_16_US                     0xC0
#define RAMP_TIME_20_US                     0xE0

/*
 * ============================================================================
 * Default Configuration Values
 * ============================================================================
 */

#define SX1280_DEFAULT_FREQUENCY            2403000000UL    /* 2403 MHz */
#define SX1280_DEFAULT_SF                   LORA_SF10
#define SX1280_DEFAULT_BW                   LORA_BW_1625
#define SX1280_DEFAULT_CR                   LORA_CR_4_5
#define SX1280_DEFAULT_TX_POWER             13              /* dBm */
#define SX1280_DEFAULT_PREAMBLE_LEN         12              /* symbols */
#define SX1280_DEFAULT_HEADER_TYPE          LORA_HEADER_EXPLICIT
#define SX1280_DEFAULT_PAYLOAD_LEN          255
#define SX1280_DEFAULT_CRC                  LORA_CRC_ENABLED
#define SX1280_DEFAULT_IQ                   LORA_IQ_STANDARD

/*
 * ============================================================================
 * Buffer Configuration
 * ============================================================================
 */

#define SX1280_MAX_PAYLOAD_LENGTH           255
#define SX1280_TX_BASE_ADDR                 0x00
#define SX1280_RX_BASE_ADDR                 0x00

/*
 * ============================================================================
 * Status Register Bits
 * ============================================================================
 */

/* Chip Mode (bits 6:4 of status byte) */
#define STATUS_MODE_SHIFT                   4
#define STATUS_MODE_MASK                    0x70

#define STATUS_MODE_UNUSED                  0x00
#define STATUS_MODE_STDBY_RC                0x20
#define STATUS_MODE_STDBY_XOSC              0x30
#define STATUS_MODE_FS                      0x40
#define STATUS_MODE_RX                      0x50
#define STATUS_MODE_TX                      0x60

/* Command Status (bits 3:1 of status byte) */
#define STATUS_CMD_SHIFT                    1
#define STATUS_CMD_MASK                     0x0E

#define STATUS_CMD_DATA_AVAILABLE           0x02
#define STATUS_CMD_TIMEOUT                  0x04
#define STATUS_CMD_PROCESSING_ERROR         0x06
#define STATUS_CMD_EXECUTION_FAILURE        0x08
#define STATUS_CMD_TX_DONE                  0x0A
#define STATUS_CMD_RX_DONE                  0x0C

/*
 * ============================================================================
 * Function Prototypes
 * ============================================================================
 */

/* Forward declaration */
struct sx1280;

/* Low-level SPI communication functions */
int sx1280_wait_busy(struct sx1280 *dev, unsigned int timeout_ms);
int sx1280_write_command(struct sx1280 *dev, u8 command, const u8 *params, size_t param_len);
int sx1280_read_command(struct sx1280 *dev, u8 command, u8 *data, size_t data_len);
int sx1280_write_register(struct sx1280 *dev, u16 address, const u8 *data, size_t len);
int sx1280_read_register(struct sx1280 *dev, u16 address, u8 *data, size_t len);
int sx1280_write_buffer(struct sx1280 *dev, u8 offset, const u8 *data, size_t len);
int sx1280_read_buffer(struct sx1280 *dev, u8 offset, u8 *data, size_t len);

/* High-level radio control functions */
int sx1280_get_status(struct sx1280 *dev, u8 *status);
int sx1280_reset(struct sx1280 *dev);
int sx1280_set_standby(struct sx1280 *dev, u8 mode);
int sx1280_set_packet_type(struct sx1280 *dev, u8 packet_type);
int sx1280_set_rf_frequency(struct sx1280 *dev, u32 frequency);
int sx1280_set_tx_params(struct sx1280 *dev, s8 power, u8 ramp_time);
int sx1280_set_buffer_base_address(struct sx1280 *dev, u8 tx_base, u8 rx_base);
int sx1280_set_modulation_params(struct sx1280 *dev, u8 sf, u8 bw, u8 cr);
int sx1280_set_packet_params(struct sx1280 *dev, u16 preamble_len, u8 header_type,
                              u8 payload_len, u8 crc, u8 iq_invert);
int sx1280_set_dio_irq_params(struct sx1280 *dev, u16 irq_mask, u16 dio1_mask,
                               u16 dio2_mask, u16 dio3_mask);
int sx1280_get_irq_status(struct sx1280 *dev, u16 *irq_status);
int sx1280_clear_irq_status(struct sx1280 *dev, u16 irq_mask);
int sx1280_set_tx(struct sx1280 *dev, u16 timeout_ms);
int sx1280_set_rx(struct sx1280 *dev, u16 timeout_ms);
int sx1280_get_rx_buffer_status(struct sx1280 *dev, u8 *payload_len, u8 *rx_start_offset);
int sx1280_get_packet_status(struct sx1280 *dev, s8 *rssi, s8 *snr);
int sx1280_set_regulator_mode(struct sx1280 *dev, u8 mode);
int sx1280_calibrate(struct sx1280 *dev, u8 calib_param);

/* Initialization and configuration */
int sx1280_init(struct sx1280 *dev);

#endif /* __SX1280_DRIVER_H__ */
