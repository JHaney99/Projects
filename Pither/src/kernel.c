#include <stdint.h> // Includes definitions for fixed-width integer types (such as uint32_t). Useful in embedded development where memory and layout must be precise

#define UART0 ((volatile uint32_t*)0x101f1000) // Pi specific for memory

void uart_puts(const char *str) {
	while (*str) {
		UART0[0] = *str++;
	}
}

void kernel_main(void) {   // This is the main function for my kernel. Its like main()i
	uart_puts("Welcome to Pither\n");
	while(1);
}
