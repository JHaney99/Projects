#include <stdio.h>

#define UART0_DR ((volatile uint32_t*)(0x020201000))

void uart_puts(const char *str) {
	while (*str) {
		*UART0_DR = (uint32_t)(*str++);

	}
}

void kernel_main(void) {
	uart_puts("Welcome to NeuroPi\n");
	while (1); // halt
}
