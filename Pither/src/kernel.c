#include <stdint.h> // Includes definitions for fixed-width integer types (such as uint32_t). Useful in embedded development where memory and layout must be precise

#define UART0_DR ((volatile uint32_t*)(0x20201000)) // Pi specific for memory

void uart_puts(const char *str) {
	while (*str) {
		*UART0_DR = (uint32_t)(*str++);

	}
}

void kernel_main(void) {   // This is the main function for my kernel. Its like main()
	*UART0_DR = 'X';
	while (1); // halt
}
