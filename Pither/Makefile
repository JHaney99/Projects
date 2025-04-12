# Toolchain prefix
CROSS_COMPILE = arm-none-eabi-

# Compile and assembler flags
CFLAGS = -Wall -O2 -nostdlib -ffreestanding -Iinclude
ASFLAGS = -mcpu=arm1176jzf-s

# File paths
BUILD_DIR = build
SRC_DIR = src
BOOT_DIR = boot
LINKER = linker/linker.ld

# Output Files
KERNEL_ELF = $(BUILD_DIR)/kernel.elf
KERNEL_IMG = $(BUILD_DIR)/kernel.img

# Source files
BOOT_SRC = $(BOOT_DIR)/boot.S
KERNEL_SRC = $(SRC_DIR)/kernel.c

BOOT_OBJ  = $(BUILD_DIR)/boot.o
KERNEL_OBJ = $(BUILD_DIR)/kernel.o

# Default build target
all: $(KERNEL_IMG)

# Build binary image
$(KERNEL_IMG): $(KERNEL_ELF)
	$(CROSS_COMPILE)objcopy $(KERNEL_ELF) -O binary $@

# Link object files to ELF
$(KERNEL_ELF): $(BOOT_OBJ) $(KERNEL_OBJ)
	$(CROSS_COMPILE)ld -T $(LINKER) -o $@ $^

# Assemble bootloader
$(BOOT_OBJ): $(BOOT_SRC)
	$(CROSS_COMPILE)as $(ASFLAGS) $< -o $@

# Compile kernel C code
$(KERNEL_OBJ): $(KERNEL_SRC)
	$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -rf $(BUILD_DIR)/*.o $(BUILD_DIR)/*.elf $(BUILD_DIR)/*.img

.PHONY: all clean
