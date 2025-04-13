#include "uart.h"

void kernel_main(void) {   // This is the main function for my kernel. Its like main()
	uart_init();
	uart_write_string("Welcome to Pither via UART HAL\n");

	while(1);
}
