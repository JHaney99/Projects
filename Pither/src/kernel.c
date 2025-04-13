#include "uart.h"
#include "irq.h"
#include <stdint.h>

#define VIC_INTENABLE (*(volatile uint32_t*)0x10140010)
#define VIC_SOFTINT (*(volatile uint32_t*)0x10140008)

#define TIMER1_BASE     0x10111000
#define TIMER1_LOAD     (*(volatile uint32_t*)(TIMER1_BASE + 0x00))
#define TIMER1_CTRL     (*(volatile uint32_t*)(TIMER1_BASE + 0x08))
#define TIMER1_INTCLR   (*(volatile uint32_t*)(TIMER1_BASE + 0x0C))


void kernel_main(void) {   // This is the main function for my kernel. Its like main()
	uart_init();
	uart_write_string("Welcome to Pither via UART HAL\n");

	uart_write_string("IRQ testing\n");

	// Enable software interrupt bit
	VIC_INTENABLE = (1 << 3);
	
	// Trigger the interrupt
	VIC_SOFTINT = (1 << 3);

	while(1);
}
