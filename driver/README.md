# SX1280 Linux Kernel Driver

## Stage 1: SPI Communication Primitives - COMPLETE ✅

This directory contains a Linux kernel driver for the Semtech SX1280 2.4GHz LoRa transceiver.

### Current Status

**Stage 1 is complete** with the following functionality:
- ✅ All SPI communication primitives implemented
- ✅ BUSY pin handling
- ✅ Command read/write functions
- ✅ Register read/write functions
- ✅ Buffer read/write functions
- ✅ Status register test in probe function

### Files

- `skeleton.c` - Main driver implementation
- `sx1280_driver.h` - Header with opcodes, constants, and prototypes
- `Makefile` - Build system
- `README.md` - This file

### Building the Driver

#### Prerequisites

```bash
# Install kernel headers (if not already installed)
sudo apt-get install raspberrypi-kernel-headers

# Or for generic Debian/Ubuntu:
sudo apt-get install linux-headers-$(uname -r)
```

#### Build

```bash
cd /home/drathgeb/Documents/winch/SX1280_driver_linux/RPI-SX1280/driver
make
```

Expected output:
```
make -C /lib/modules/.../build M=/path/to/driver modules
...
  CC [M]  /path/to/driver/skeleton.o
  LD [M]  /path/to/driver/sx1280.o
  MODPOST /path/to/driver/Module.symvers
  CC [M]  /path/to/driver/sx1280.mod.o
  LD [M]  /path/to/driver/sx1280.ko
```

If successful, you'll have `sx1280.ko` - the kernel module.

#### Clean

```bash
make clean
```

### Testing the Driver

#### Prerequisites

1. **Hardware Setup:**
   - SX1280 module connected to Raspberry Pi SPI0
   - GPIO pins configured in device tree (from HAT EEPROM)
   - Power connected to SX1280 module

2. **Device Tree:**
   Your HAT's EEPROM should have a device tree overlay defining:
   ```
   &spi0 {
       sx1280@0 {
           compatible = "semtech,sx1280";
           reg = <0>;
           spi-max-frequency = <5000000>;
           reset-gpios = <&gpio XX GPIO_ACTIVE_LOW>;
           busy-gpios = <&gpio YY GPIO_ACTIVE_HIGH>;
           interrupts = <&gpio ZZ IRQ_TYPE_EDGE_RISING>;
           /* Optional: txen-gpios, rxen-gpios */
       };
   };
   ```

#### Load the Module

```bash
# Load the module
sudo make load

# Or manually:
sudo insmod sx1280.ko
```

#### Check Driver Output

```bash
# View recent kernel messages
make dmesg

# Or manually:
dmesg | tail -30
```

#### Expected Output (Success)

```
[  xxx.xxxxxx] sx1280 spi0.0: SX1280 detected! Status=0x2X
[  xxx.xxxxxx] sx1280 spi0.0:   Chip Mode: 0x2 (STANDBY_RC)
[  xxx.xxxxxx] sx1280 spi0.0:   Command Status: 0xX
[  xxx.xxxxxx] sx1280 spi0.0: SPI communication test PASSED
```

The status byte should show:
- **Chip Mode:** `0x2` (STANDBY_RC) - chip is in standby with RC oscillator
- **Status byte:** Usually `0x2X` where X varies

This confirms:
✅ SPI bus is working
✅ BUSY pin is functional
✅ SX1280 is responding to commands

#### Troubleshooting

**Error: "Timeout waiting for BUSY pin"**
- Check BUSY GPIO pin connection
- Verify GPIO number in device tree
- Check power to SX1280 module

**Error: "SPI transfer failed"**
- Check SPI wiring (MOSI, MISO, SCLK, CS)
- Verify SPI bus is enabled (`ls /dev/spidev0.0`)
- Check SPI frequency (should be 5MHz or lower)

**Error: "probe failed"**
- Check device tree overlay is loaded (`dtoverlay -l`)
- Verify compatible string matches: "semtech,sx1280"
- Check GPIO pins are not in use by other drivers

**Module won't load**
- Check kernel version matches headers: `uname -r`
- Look for build errors: `make clean && make`
- Check kernel log: `dmesg | grep -i error`

#### Unload the Module

```bash
# Unload the module
sudo make unload

# Or manually:
sudo rmmod sx1280
```

### What's Next?

Stage 1 provides the foundation for all SPI communication with the SX1280. Next stages will implement:

**Stage 2:** Hardware reset and basic initialization
- Reset sequence
- Standby mode
- Regulator configuration

**Stage 3:** LoRa configuration
- Set frequency
- Configure modulation (SF, BW, CR)
- Set packet parameters
- TX power configuration

**Stage 4-5:** TX and RX functionality
- Transmit packets
- Receive packets
- IRQ handling

**Stage 6:** Character device interface
- `/dev/sx1280-0` device node
- User-space read/write/ioctl API

See `../progress.md` for the complete roadmap.

### Development Notes

#### Code Structure

```c
// Low-level SPI primitives (Stage 1 - COMPLETE)
sx1280_wait_busy()         // Poll BUSY pin
sx1280_write_command()     // Send command
sx1280_read_command()      // Read command response
sx1280_write_register()    // Write register(s)
sx1280_read_register()     // Read register(s)
sx1280_write_buffer()      // Write TX buffer
sx1280_read_buffer()       // Read RX buffer
sx1280_get_status()        // Read status (test function)

// High-level functions (Stage 2+)
sx1280_reset()             // TODO: Hardware reset
sx1280_init()              // TODO: Initialize radio
sx1280_set_frequency()     // TODO: Set RF frequency
// ... more to come
```

#### Testing SPI Functions

You can add additional test code in `sx1280_probe()` to verify other functions:

```c
// Example: Test register read/write
u8 test_data = 0x55;
ret = sx1280_write_register(dev, 0x0901, &test_data, 1);
ret = sx1280_read_register(dev, 0x0901, &test_data, 1);
dev_info(&spi->dev, "Register test: read 0x%02x\n", test_data);
```

### License

GPL-2.0

### Author

Dennis' future self (as per MODULE_AUTHOR)
