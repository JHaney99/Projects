ENTRY(_start)

SECTIONS {
	/*
	. = 0x00000000;
	trying to place vector table here but since I load entire image to 0x10000, in RAM nothing is contained in 0x000000
	.vectors : { KEEP(*(.vectors)) }
	*/

	. = 0x10000;  /* Hex, matches where qemu laods the img file. All memory below is referenced from there */
	.vectors : {
		_vector_start = .; 
		KEEP(*(.vectors))
		_vector_end = .;
		}

	.text : { 
		*(.text) 
		}

	.rodata : { 
		*(.rodata) 
		}

	.data : { 
		*(.data) 
		}

	.bss : { 
		*(.bss COMMON) 
		}
}
