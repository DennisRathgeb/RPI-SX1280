# SX1280 Linux Kernel Driver - Implementation Progress

## Overview
Building a Linux kernel driver for the Semtech SX1280 2.4GHz LoRa transceiver. The driver will expose a character device interface for user-space communication, with potential future integration into a custom network stack.

**Target Platform:** Raspberry Pi 3/4/5 (SPI0)
**Current Focus:** Stage 1 - SPI Communication Primitives

---

## Implementation Stages

### ✅ Stage 0: Driver Infrastructure (COMPLETE)
**Status:** Done
**Files:** `driver/skeleton.c`

- [x] Basic kernel module structure
- [x] SPI device registration
- [x] GPIO descriptor handling (reset, busy, txen, rxen)
- [x] IRQ registration framework
- [x] Device tree binding ("semtech,sx1280")
- [x] Basic `sx1280_spi_xfer()` function

**Testing:** Module compiles and loads (pending build test)

---

### ✅ Stage 1: SPI Communication Primitives (COMPLETE)
**Status:** Complete
**Date Completed:** 2025-11-23
**Files:** `driver/sx1280_driver.h`, `driver/skeleton.c`, `driver/Makefile`

#### Command Opcodes & Definitions
- [x] All 30+ command opcodes defined in header
- [x] Register addresses for LoRa operation
- [x] IRQ flag definitions (16 flags)
- [x] Hardware constants (XTAL_FREQ, timing, etc.)
- [x] LoRa modulation parameters (SF, BW, CR)
- [x] Default configuration values
- [x] Status register bit definitions

#### Core SPI Functions
- [x] `sx1280_wait_busy()` - Poll BUSY pin until ready
- [x] `sx1280_write_command()` - Send command with parameters
- [x] `sx1280_read_command()` - Read command response (NOP handling)
- [x] `sx1280_write_register()` - Write to 16-bit addressed registers
- [x] `sx1280_read_register()` - Read from registers
- [x] `sx1280_write_buffer()` - Write TX payload buffer
- [x] `sx1280_read_buffer()` - Read RX payload buffer

#### Test Functions
- [x] `sx1280_get_status()` - First test to verify SPI works
- [x] Add test call in `sx1280_probe()` with status decode

#### Build System
- [x] Complete Makefile with build/clean/load/unload targets
- [x] Module info and metadata

**Implementation Details:**
- All SPI functions properly implement the SX1280 protocol
- BUSY pin polling with configurable timeout (100ms default)
- Mutex locking for SPI bus protection
- Proper NOP byte handling for read operations
- Special handling for RADIO_GET_STATUS command
- Detailed kernel log messages for debugging
- Status byte decoding in probe function

**Testing Checkpoint:**
- [ ] Module compiles without errors (ready to test)
- [ ] Module loads successfully (`dmesg` shows probe)
- [ ] SPI communication works (status read returns valid data)
- [ ] BUSY pin handling is correct
- [ ] No kernel warnings or errors

**Next Steps:** Build and test the module, then proceed to Stage 2

---

### ⏳ Stage 2: Hardware Reset & Basic Initialization (PENDING)
**Status:** Not Started
**Dependencies:** Stage 1 complete

#### Functions to Implement
- [ ] `sx1280_reset()` - Hardware reset sequence
- [ ] `sx1280_set_standby()` - Enter STDBY_RC mode
- [ ] `sx1280_get_status()` - Read radio status register (already have basic version)
- [ ] `sx1280_set_regulator_mode()` - Configure LDO/DC-DC

**Testing Checkpoint:**
- [ ] Reset sequence works reliably
- [ ] Device enters standby mode
- [ ] Status register reads show correct mode bits
- [ ] Device responds after reset

---

### ⏳ Stage 3: LoRa Configuration (PENDING)
**Status:** Not Started
**Dependencies:** Stage 2 complete

#### Functions to Implement
- [ ] `sx1280_set_packet_type()` - Configure for LoRa (0x01)
- [ ] `sx1280_set_rf_frequency()` - Set channel (2403 MHz default)
- [ ] `sx1280_set_modulation_params()` - SF, BW, CR
- [ ] `sx1280_set_packet_params()` - Preamble, header, CRC, length
- [ ] `sx1280_set_tx_params()` - Power (dBm) and ramp time
- [ ] `sx1280_set_buffer_base_address()` - TX/RX buffer locations
- [ ] `sx1280_calibrate()` - Initial calibration

#### Default LoRa Parameters
- Frequency: 2403 MHz
- Spreading Factor: SF10
- Bandwidth: 1625 kHz
- Coding Rate: 4/5
- TX Power: +13 dBm
- Preamble Length: 12 symbols
- Header: Explicit
- CRC: Enabled

**Testing Checkpoint:**
- [ ] All LoRa parameters configure without errors
- [ ] Register reads confirm settings applied
- [ ] No SPI communication errors

---

### ⏳ Stage 4: Transmit Functionality (PENDING)
**Status:** Not Started
**Dependencies:** Stage 3 complete

#### Functions to Implement
- [ ] `sx1280_set_dio_irq_params()` - Configure IRQ flags
- [ ] `sx1280_get_irq_status()` - Read IRQ status
- [ ] `sx1280_clear_irq_status()` - Clear IRQ flags
- [ ] `sx1280_set_tx()` - Start transmission
- [ ] `sx1280_transmit_packet()` - Complete TX sequence with polling

**Testing Checkpoint:**
- [ ] Can load payload and transmit
- [ ] TX_DONE IRQ flag sets correctly
- [ ] No errors during transmission
- [ ] (Future: Verify with spectrum analyzer or second module)

---

### ⏳ Stage 5: Receive Functionality (PENDING)
**Status:** Not Started
**Dependencies:** Stage 4 complete

#### Functions to Implement
- [ ] `sx1280_set_rx()` - Start reception
- [ ] `sx1280_get_rx_buffer_status()` - Get payload length/offset
- [ ] `sx1280_get_packet_status()` - Read RSSI/SNR
- [ ] `sx1280_receive_packet()` - Complete RX sequence with polling

**Testing Checkpoint:**
- [ ] Can enter RX mode
- [ ] RX_DONE IRQ flag sets when packet received
- [ ] Can read received payload
- [ ] RSSI/SNR values are reasonable
- [ ] (Future: Test with second SX1280 module)

---

### ⏳ Stage 6: Character Device Interface (PENDING)
**Status:** Not Started
**Dependencies:** Stages 4 & 5 complete

#### Implementation Tasks
- [ ] Add char device registration
- [ ] Implement `sx1280_fops` (file_operations)
- [ ] `sx1280_open()` - Open device
- [ ] `sx1280_release()` - Close device
- [ ] `sx1280_read()` - Receive packet (blocking/non-blocking)
- [ ] `sx1280_write()` - Transmit packet
- [ ] `sx1280_ioctl()` - Configuration commands
- [ ] Create ioctl command definitions

#### IOCTL Commands to Define
- [ ] `SX1280_SET_FREQUENCY` - Set RF frequency
- [ ] `SX1280_GET_FREQUENCY` - Get RF frequency
- [ ] `SX1280_SET_SPREADING_FACTOR` - Set SF (5-12)
- [ ] `SX1280_SET_BANDWIDTH` - Set BW (203-1625 kHz)
- [ ] `SX1280_SET_CODING_RATE` - Set CR (4/5 - 4/8)
- [ ] `SX1280_SET_TX_POWER` - Set TX power (-18 to +13 dBm)
- [ ] `SX1280_SET_PREAMBLE_LENGTH` - Set preamble
- [ ] `SX1280_GET_RSSI` - Get last RSSI
- [ ] `SX1280_GET_SNR` - Get last SNR

**Testing Checkpoint:**
- [ ] `/dev/sx1280-0` device node created
- [ ] User-space app can open/close device
- [ ] Can configure via ioctl
- [ ] Can transmit with `write()`
- [ ] Can receive with `read()`
- [ ] Blocking/non-blocking modes work

---

### ⏳ Stage 7: IRQ-Driven Operation (PENDING)
**Status:** Not Started
**Dependencies:** Stage 6 complete

#### Implementation Tasks
- [ ] Complete `sx1280_irq_thread()` implementation
- [ ] Handle TX_DONE interrupt
- [ ] Handle RX_DONE interrupt
- [ ] Handle RX_TX_TIMEOUT interrupt
- [ ] Handle CRC_ERROR interrupt
- [ ] Add wait queues for blocking operations
- [ ] Implement RX packet queue
- [ ] Remove polling from TX/RX functions

**Testing Checkpoint:**
- [ ] Interrupts fire correctly
- [ ] TX completes via IRQ (not polling)
- [ ] RX works via IRQ (not polling)
- [ ] Multiple packets can be queued
- [ ] No race conditions or deadlocks
- [ ] Performance improved vs polling

---

### ⏳ Stage 8: Advanced Features & Optimization (PENDING)
**Status:** Not Started
**Dependencies:** Stage 7 complete

#### Power Management
- [ ] Implement sleep mode entry/exit
- [ ] Runtime PM support
- [ ] Power consumption optimization

#### Error Recovery
- [ ] CRC error handling and retries
- [ ] Timeout recovery
- [ ] TX failure handling
- [ ] RX overflow handling

#### Statistics & Monitoring
- [ ] Packet counters (TX/RX success/fail)
- [ ] RSSI history tracking
- [ ] Error counters
- [ ] sysfs interface for statistics

#### Performance & Optimization
- [ ] DMA support for SPI transfers (if beneficial)
- [ ] TX/RX buffer management optimization
- [ ] Latency measurements and optimization

#### Future: Network Stack Integration
- [ ] Plan netdev integration for custom stack
- [ ] Design packet format for network layer
- [ ] Implement MAC layer if needed

---

## Testing Log

### Build Tests
**Date:** TBD
**Status:** Not tested

```bash
# Build commands
make -C /lib/modules/$(uname -r)/build M=$(pwd)/driver modules
```

**Results:** TBD

---

### SPI Communication Tests
**Date:** TBD
**Status:** Not tested

**Test 1: Read Status Register**
- Command: `RADIO_GET_STATUS` (0xC0)
- Expected: Status byte indicating mode (should be STDBY_RC after boot)
- Result: TBD

**Test 2: Write/Read Register**
- Write test pattern to register
- Read back and verify
- Result: TBD

---

### Functional Tests
**Date:** TBD
**Status:** Not tested

**Test 1: Transmit Test Packet**
- Load known payload
- Transmit
- Verify TX_DONE IRQ
- Result: TBD

**Test 2: Receive Test Packet**
- Configure RX mode
- Wait for packet
- Read payload
- Verify CRC
- Result: TBD

---

## Known Issues

### Current Issues
*None yet - Stage 1 in progress*

### Resolved Issues
*None yet*

---

## Hardware Configuration

**Raspberry Pi:** 3/4/5
**SPI Bus:** SPI0 (`/dev/spidev0.0`)
**SPI Mode:** Mode 0 (CPOL=0, CPHA=0)
**SPI Speed:** 5 MHz (default, max 8 MHz)

### GPIO Pins
*Configured in HAT EEPROM device tree overlay*

- NSS (Chip Select): SPI0 CS0
- RESET: GPIO TBD (check DT)
- BUSY: GPIO TBD (check DT)
- DIO1: GPIO TBD (check DT, mapped to IRQ)
- TXEN: GPIO TBD (optional, check DT)
- RXEN: GPIO TBD (optional, check DT)

---

## Performance Metrics

*To be filled in during testing*

### Latency
- TX latency (user write() to on-air): TBD
- RX latency (on-air to user read()): TBD

### Throughput
- Max TX rate: TBD
- Max RX rate: TBD

### Power Consumption
- TX current: TBD
- RX current: TBD
- Standby current: TBD
- Sleep current: TBD

---

## References

### Documentation
- SX1280 Datasheet: `docs/DS_SX1280-1_V3.3.pdf`
- RIOT OS Reference (placeholder): `riot/`
- Linux SPI Framework: `Documentation/spi/`
- Linux GPIO Framework: `Documentation/driver-api/gpio/`

### Key Register Addresses
- `REG_LR_PAYLOADLENGTH`: 0x0901
- `REG_LR_PACKETPARAMS`: 0x0903
- `REG_LR_SYNCWORDBASEADDRESS1`: 0x09CE
- `REG_LR_CRCSEEDBASEADDR`: 0x09C8
- `REG_LR_CRCPOLYBASEADDR`: 0x09C6

### Hardware Constants
- `XTAL_FREQ`: 52,000,000 Hz
- `FREQ_STEP`: ~198.364 Hz (XTAL_FREQ / 2^18)

---

## Build & Installation

### Build Commands
```bash
cd /home/drathgeb/Documents/winch/SX1280_driver_linux/RPI-SX1280/driver
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
```

### Install Module
```bash
sudo insmod sx1280.ko
dmesg | tail  # Check probe messages
```

### Remove Module
```bash
sudo rmmod sx1280
```

### Check Device
```bash
ls -l /dev/sx1280-*  # After chardev implementation
```

---

**Last Updated:** 2025-11-23
**Current Stage:** 1 (SPI Communication Primitives)
**Next Milestone:** Complete Stage 1 SPI functions and verify basic communication
