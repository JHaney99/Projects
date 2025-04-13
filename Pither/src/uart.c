#include "mmio.h"
#include "uart.h"

#define UART0_BASE 0x101f1000 // For QEMU versatilepb WHERE UART PINS/MEMORY WOULD BE
#define UART0_DR	(UART0_BASE + 0x00)

void uart_init(void) {
	// Placeholder to control baud rate, control registers, etc
}

void uart_write_char(char c){
	mmio_write(UART0_DR, (uint32_t)c); //Write a character to a register?
}

void uart_write_string(const char* str) {
	while (*str) {
		uart_write_char(*str++); //Iterate through the string and write character to location using mmio_write
	}
}
