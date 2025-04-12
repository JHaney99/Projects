CROSS_COMPILE=arm-none-eabi-
CFLAGS=-Wall -O2 -nostdlib -ffreestanding -Iinclude

all: build/kernel.img

build/kernel.img: build/boot.o build/kernel.o
	$(CROSS_COMPILE)ld -T linker/linker.ld -o build/kernel.elf $^
	$(CROSS_COMPILE)objcopy build/kernel.elf -O binary build/kernel.img

build/boot.o: boot/boot.S
	$(CROSS_COMPILE)as $< -o $@

build/kernel.o: src/kernel.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@

clean:
	rm -rf build/*.o build/*.elf build/*.img
