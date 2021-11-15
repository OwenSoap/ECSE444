/*
* asmproduct.s
*/

// unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmProduct, which is expected by lab1math.h
.global asmProduct

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
* void asmProduct(float *arraya, float *arrayb, uint32_t size, float *res);
*
* R0 = pointer to array a
* R1 = pointer to array b
* R2 = array size
* R3 = result array
*/

asmProduct:
	PUSH {R4 - R6} // saving R4 to R6 according to calling covention

loop:
	SUBS R2, R2, #1 // size = size - 1
	BLT done // loop finishes when R2 < 0
	ADD R4, R0, R2, LSL #2 // calculate base address (in R4) for array a
	VLDR.f32 S1, [R4] // load element in array a into fp register S1 (from R5)
	ADD R5, R1, R2, LSL #2 // calculate base address (in R5) for array b
	VLDR.f32 S2, [R5] // load element in array ab into fp register S2 (from R5)
	VMUL.f32 S3, S1, S2 // multiply S1 and S2 and save into S3
	ADD R6, R3, R2, LSL #2 // calculate base address (in R6) for result array
	VSTR.f32 S3, [R6] // store the product result in the pointer to result
	B loop // next iteration

done:
	POP {R4 - R6} // restore contex
	BX LR // return

