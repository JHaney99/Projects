#include "irq.h"
#include "mmio.h"
#include "uart.h"

#define VIC_IRQSTATUS (*(volatile uint32_t*)0x10140000)
#define VIC_SOFTINT (*(volatile uint32_t*)0x10140008)
#define VIC_INTENABLE (*(volatile uint32_t*)0x10140010)

void c_irq_handler(void) {
    uint32_t status = VIC_IRQSTATUS;

    if (status & (1 << 3)) {
        // IRQ from software interrupt channel 3
        uart_write_string("[IRQ] Software interrupt received\n");

        // Clear the software interrupt
        VIC_SOFTINT &= ~(1 << 3);
    }
}
