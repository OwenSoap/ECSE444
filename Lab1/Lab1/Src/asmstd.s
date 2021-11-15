/*
* asmstd.s
*/

// unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmStd, which is expected by lab1math.h
.global asmStd

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
* void asmStd(float *array, uint32_t size, float *res);
*
* R0 = pointer to array
* R1 = size
* R2 = pointer to result
*/

asmStd:
	PUSH {R3 - R7} // saving R3 to R7 according to calling convention
	VPUSH {S0 - S6} // saving S0 to S6 according to calling convention
	MOV R3, R0 // assign pointer of the array to R3
	MOV R4, R1 // assign size to R4
	MOV R5, R1 // assign size to R5

sum:
	SUBS R1, R1, #1 // size = size - 1
	BLT miu // loop finishes when R1 < 0
	ADD R6, R0, R1, LSL #2 // calculate base address (in R6) for array
	VLDR.f32 S1, [R6] // load element in array into fp register S1 (from R6)
	VADD.f32 S0, S0, S1 // add sum result into S0
	B sum // next iteration

miu:
	VMOV.f32 S2, R4 // assign the size to S2
	VCVT.f32.u32 S2, S2 // covert N to 32bits
	VDIV.f32 S3, S0, S2 // divide the sum by size and load miu to S3
	SUBS R4, R4, #1 // N = N-1
	VMOV.f32 S4, R4 // assign N-1 to S4
	VCVT.f32.u32 S4, S4 // covert N-1 to 32bits

nor:
	SUBS R5, R5, #1 // size = size - 1
	BLT done // loop finishes when R5 < 0
	ADD R7, R3, R5, LSL #2 // calculate base address (in R7) for array
	VLDR.f32 S5, [R7] // load element in array into fp register S1 (from R7)
	VSUB.f32 S5, S5, S3 // substract element by miu
	VMUL.f32 S5, S5, S5 // take square of the result
	VDIV.f32 S5, S5, S4 // divide the square by N-1
	VADD.f32 S6, S6, S5 // add sum result into S6
	B nor // next iteration

done:
	VSQRT.f32 S6, S6 // take square root of the sum
	VSTR.f32 S6, [R2] // store the standard deviation result to result
	VPOP {S0 - S6} //restore contex
	POP {R3 - R7} //restore contex
	BX LR // return
