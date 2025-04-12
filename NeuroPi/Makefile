#toolchain prefix (gcc-arm-none-eabi)
CROSS_COMPILE = arm-none-eabi-

# Compiler and assembler flags
CFLAGS = -Wall -02 -nostdlib -ffreestanding -Iinclude
ASFLAGS = -mcpu=arm1176jzf-s

# File paths
BUILD_DIR = build
SRC_DIR = src
BOOT_DIR = boot
LINKER = linker/linker.ld

# Output Files
KERNEL_ELF = $(BUILD_DIR)/kernel.elf
KERNEL_IMG = $(BUILD_DIR)/kernel.img

# Source Files
BOOT_SRC = $(BOOT_DIR)/boot.S
KERNEL_SRC = $(BOOT_DIR)/kernel.c

BOOT_OBJ = $(BUILD_DIR)/boot.o
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
	$(CROSS_COMPILE)as $(ASFLAGS) &< -o $@

# Compile kernel C Code
$(KERNEL_OBJ): $(KERNEL_SRC)
	$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -rf $(BUILD_DIR)/*.o $(BUILD_DR)/*.elf $(BUILD_DIR)/*.img

.PHONY: all clean
