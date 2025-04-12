.section .text  // tells the assembler to place the following code inside of the .text secton, which is where the executable code WILL live
.global _start  //will make the _start visible globally across the system so that the linker will know that this is the entry point --- I.E. where the first CPU instruction will exectute

_start: // Creating the start function (similar to main in c)
	ldr sp, =stack_top	// Set stack pointer (loads the address of stack_top to the stack pointer register (sp). In ARM sp MUST point to the top of the stack as the stack grows downward
	bl kernel_main		// Branch to C kernel  (bl = branch with link. Jumpts to the c function i created, saving the return address lr (link register). This is how my C code will get called from assembly. kernel_main will be the name of a function in my C code. My real operating system logic will come from there
	b .

.section .bss  // Creates zero-initialzed global/static variables. Just a bunch of zeros (0000000). Stack memory will usually be placed here since it is a chunk of zero'd out space
.align 12 // Ensures that the next label coming (stack:) is alligned to 2^12 bytes (4096 bytes). Aids performance
stack:
	.space 4096 // Reserves 4096 bytes (4 KB) of space for the stack
stack_top:	    // Label pointing to the end of the 4kb stack that I reserved. Since stack grows downward, the top would be the last address of this region
