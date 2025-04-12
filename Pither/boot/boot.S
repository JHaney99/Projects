.section .text
.global _start

_start:
	ldr sp, =stack_top //setup stack
	
	// Delay loop
	mov r2, #0x100000
delay:
	subs r2, r2, #1
	bne delay

	// Write character 'B' directly to UART0
	ldr r0, =0x101f1000 // UART0 base address
	mov r1, #'B' // ASCII for 'A'
	str r1, [r0] // Write it to UART data register

	b .  // infinite loop

.section .bss
.align 12
stack:
	.space 4096
stack_top:
