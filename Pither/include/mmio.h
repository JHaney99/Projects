#ifndef MMIO_H
#define MMIO_H

#include <stdint.h>

// Static inline functions are 'allowed' to be in a .h file. Static inline is almost like copy and pasting the code into another file in which it is called
static inline void mmio_write(uint32_t reg, uint32_t data) { // uint32_t - unsigned 32 bits ---Width of each register. If dealing with 64 bit registers, would probably need to use 64. Data going to registers will also be the size of the register (duh)
	*(volatile uint32_t*)reg = data; // Setting a register equal to the value of data. (volatile uint32_t*)reg points to the register (reg) *(volatile uint32_t*)reg = data goes to the actualy memory location and stores the value data in there
}

static inline uint32_t mmio_read(uint32_t reg) {
	return *(volatile uint32_t*)reg; // Returns the value of whatever is in the register. If I just did (volatile uint32_t*)reg it would return the memory location itself
}

#endif