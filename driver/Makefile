obj-m := sx1280.o


KDIR ?= /home/drathgeb/src/rpi-linux-6.12

PWD  := $(shell pwd)
BDIR := $(PWD)/build

BUILD_FILES := *.o *.ko *.mod *.mod.c *.order *.symvers \
               .*.cmd .tmp_versions modules.order

all:
	mkdir -p $(BDIR)
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	@mv -f $(BUILD_FILES) $(BDIR) 2>/dev/null || true

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -rf $(BDIR)

tidy:
	mkdir -p $(BDIR)
	@mv -f $(BUILD_FILES) $(BDIR) 2>/dev/null || true
	
# Install module to kernel modules directory
install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

# Load the module (from build dir)
load:
	sudo insmod $(BDIR)/sx1280.ko

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
	@echo "  make          - Build the kernel module into ./build/"
	@echo "  make clean    - Remove build artifacts"
	@echo "  make install  - Install module to system"
	@echo "  make load     - Load the module (insmod from ./build)"
	@echo "  make unload   - Unload the module (rmmod)"
	@echo "  make reload   - Unload and reload module"
	@echo "  make dmesg    - Show recent kernel messages"

.PHONY: all clean install load unload reload dmesg help
