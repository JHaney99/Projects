.section .vectors, "a"
.global _start

_start:
	b reset			// 0x00
	b undefined		// 0x04
	b swi_handler	// 0x08
	b prefetch_abort // 0x0c
	b data_abort	// 0x10
	b .				// 0x14 (reserved)
	b irq_handler	// 0x18
	b fiq_handler	// 0x1C

.section .text
.global reset

reset:
	// Copy vector table from current location to 0x0000000
	ldr r0, =_vector_start
	ldr r1, =_vector_end
	mov r2, #0x00000000

copy_vectors:
	cmp r0, r1
	ldrcc r3, [r0], #4
	strcc r3, [r2], #4
	bcc copy_vectors

	ldr sp, =stack_top		// setup stack
	mrs r0, cpsr
	bic r0, r0, #0x80		// enable IRQs (clear I-BIT)
	msr cpsr_c, r0

	bl kernel_main
	b .

undefined:
	b .

swi_handler:
	b .

prefetch_abort:
	b .

data_abort:
	b .

irq_handler:
	push {r0-r12, lr}
	bl c_irq_handler
	pop {r0-r12, lr}
	subs pc, lr, #4

fiq_handler:
	b .

.section .bss
.align 12
stack:
	.space 4096
stack_top:

