ENTRY(_start)

SECTIONS {

	. = 0x00000000;
	.vectors : { KEEP(*(.vectors)) }

	. = 0x10000;  // Hex, matches where qemu laods the img file. All memory below is referenced from there
	.text : { *(.text) }
	.rodata : { *(.rodata) }
	.data : { *(.data) }
	.bss : { *(.bss COMMON) }
}
