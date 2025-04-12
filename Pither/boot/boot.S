.section .text
.global _start

_start:
	ldr sp, =stack_top //setup stack
	bl kernel_main
	b .

.section .bss
.align 12
stack:
	.space 4096
stack_top:	
