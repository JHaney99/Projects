ENTRY(_start)

SECTIONS {
	. = 0x00000000
	.vectors : {
		KEEP(*(.vectors))
	}


	. = 0x10000;

	.text : { *(.text) }
	.rodata : { *(.rodata) }
	.data : { *(.data) }
	.bss : { *(.bss COMMON) }
}
