.section .text
.global _start

_start:
	ldr sp, =stack_top	// Set stack pointer
	bl kernel_main		// Branch to C kernel

.section .bss
.align 12
stack:
	.space 4096
stack_top:	     
