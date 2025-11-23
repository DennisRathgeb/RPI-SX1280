# SPDX-License-Identifier: GPL-2.0
#
# Makefile for SX1280 LoRa Radio Driver
#

# Module name
obj-m := sx1280.o

# Source files (without .c extension)
sx1280-objs := skeleton.o

# Kernel build directory (default to current running kernel)
KDIR ?= /lib/modules/$(shell uname -r)/build

# Current directory
PWD := $(shell pwd)

# Default target: build the module
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean build artifacts
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# Install module to kernel modules directory
install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

# Load the module
load:
	sudo insmod sx1280.ko

# Unload the module
unload:
	sudo rmmod sx1280

# Reload module (for testing)
reload: unload load

# View kernel messages
dmesg:
	dmesg | tail -30

# Help target
help:
	@echo "SX1280 Driver Build Targets:"
	@echo "  make          - Build the kernel module"
	@echo "  make clean    - Remove build artifacts"
	@echo "  make install  - Install module to system"
	@echo "  make load     - Load the module (insmod)"
	@echo "  make unload   - Unload the module (rmmod)"
	@echo "  make reload   - Unload and reload module"
	@echo "  make dmesg    - Show recent kernel messages"
	@echo ""
	@echo "Test sequence:"
	@echo "  1. make"
	@echo "  2. make load"
	@echo "  3. make dmesg"

.PHONY: all clean install load unload reload dmesg help
