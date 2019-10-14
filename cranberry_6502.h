#ifndef __CRANBERRY_6502_H
#define __CRANBERRY_6502_H

//
// Sources:
// http://archive.6502.org/books/mcs6500_family_programming_manual.pdf
// http://atarihq.com/danb/files/64doc.txt
// https://stackoverflow.com/questions/45305891/6502-cycle-timing-per-instruction
// https://www.mdawson.net/vic20chrome/cpu/mos_6500_mpu_preliminary_may_1976.pdf
// https://www.masswerk.at/6502/6502_instruction_set.html
// http://www.weihenstephan.org/~michaste/pagetable/6502/6502.jpg
// http://forum.6502.org/viewtopic.php?f=4&t=2512
// https://www.pagetable.com/?p=39
// http://forum.6502.org/viewtopic.php?f=8&t=5429
// http://www.6502asm.com/beta/index.html
// 

#include <stdint.h>

typedef struct
{
	uint8_t RES : 1;
	uint8_t IRQ : 1;
	uint8_t NMI : 1;
} cran6502_interrupts_t;

void cran6502_map(uint16_t address, uint8_t* memory);
void cran6502_interrupt(cran6502_interrupts_t signals);
void cran6502_clock_cycle(void);

#endif // __CRANBERRY_6502_H

#ifdef CRANBERRY_6502_IMPLEMENTATION

#include <string.h>

enum
{
	T0 = 0x01,
	T1 = 0x02,
	T1X = 0x04,
	T2 = 0x08,
	T3 = 0x10,
	T4 = 0x20,
	T5 = 0x40,
	T6 = 0x80
};

static struct
{
	uint8_t C : 1;
	uint8_t Z : 1;
	uint8_t I : 1;
	uint8_t D : 1;
	uint8_t B : 1;
	uint8_t _ : 1;
	uint8_t V : 1;
	uint8_t N : 1;
} SR;

// IR starts at 0xEA (NOP)
static uint8_t X, Y, AC, SP, IR = 0xEA, ABL, ABH, AD;
static uint8_t PCL, PCH, PCLS, PCHS;
static uint8_t TI[] = { [T1] = 0, [T1X] = 1,[T2] = 2,[T3] = 3,[T4] = 4,[T5] = 5,[T6] = 6 };
static uint8_t PD, T = T0, IRQ;
static uint8_t DOR, DL;
static uint8_t AI, BI;
static uint8_t* MEM;

void cran6502_map(uint16_t address, uint8_t* memory)
{
	MEM = memory;
}

void cran6502_interrupt(cran6502_interrupts_t signals)
{
	IRQ |= *(uint8_t*)&signals;
}

// 0x01 if equal, 0x00 if not.
static uint64_t cran6502_eq(uint64_t l, uint64_t r)
{
	uint64_t m = ~(l ^ r);

	m &= m >> 1;
	m &= m >> 2;
	m &= m >> 4;
	m &= m >> 8;
	m &= m >> 16;
	m &= m >> 32;
	return m;
}

static uint64_t cran6502_repeat(uint64_t l)
{
	l |= l << 1;
	l |= l << 2;
	l |= l << 4;
	l |= l << 8;
	l |= l << 16;
	l |= l << 32;
	return l;
}

// Multiplexor: if (c) ? l : r
static uint64_t cran6502_mux64(uint64_t l, uint64_t r, uint64_t c)
{
	c = cran6502_repeat(c);
	return (l & ~c) | (r & c);
}

static uint8_t cran6502_mux8(uint64_t l, uint64_t r, uint64_t c)
{
	c = cran6502_repeat(c);
	return (uint8_t)((l & ~c) | (r & c));
}

// Mask: if (c) ? l : 0
static uint64_t cran6502_mask(uint64_t l, uint64_t c)
{
	c = cran6502_repeat(c);
	return l & c;
}

// Format: [1: unit, 7: extra]
enum cran6502_signals
{
	cran6502_NOP = 0x00000000,

	cran6502_UNIT_MASK = 0xE0000000,
	cran6502_UNIT_ALU = 0x80000000,

	cran6502_UNIT_MEM_MASK = 0x60000000,
	cran6502_UNIT_MEM_READ = 0x40000000,
	cran6502_UNIT_MEM_WRITE = 0x20000000,

	// BUS
	cran6502_BUS_MASK = 0x10000000,
	cran6502_BUS_DB_SB = 0x00000000,
	cran6502_BUS_ADL_ADH = 0x10000000,

	cran6502_BUS_OP_MASK = 0x0FFF0000,

	// DB Load ops
	cran6502_BUS_DB_LOAD_MASK = 0x00070000,
	cran6502_BUS_DB_AC  = 0x00010000,
	cran6502_BUS_DB_DOR = 0x00020000,
	cran6502_BUS_DB_BI  = 0x00030000,
	cran6502_BUS_DB_PCL = 0x00040000,
	cran6502_BUS_DB_PCH = 0x00050000,

	// DB Store ops
	cran6502_BUS_DB_STORE_MASK = 0x00380000,
	cran6502_BUS_DL_DB = 0x00080000,
	cran6502_BUS_AC_DB = 0x00100000,
	cran6502_BUS_SR_DB = 0x00180000,

	// DB ops
	cran6502_BUS_DB_DL_BI = cran6502_BUS_DB_SB | cran6502_BUS_DL_DB | cran6502_BUS_DB_BI,
	cran6502_BUS_DB_DL_AC = cran6502_BUS_DB_SB | cran6502_BUS_DL_DB | cran6502_BUS_DB_AC,
	cran6502_BUS_DB_AC_DOR = cran6502_BUS_DB_SB | cran6502_BUS_AC_DB | cran6502_BUS_DB_DOR,

	// SB Load ops
	cran6502_BUS_SB_LOAD_MASK = 0x01C00000,
	cran6502_BUS_SB_X  = 0x00400000,
	cran6502_BUS_SB_AC = 0x00800000,
	cran6502_BUS_SB_Y  = 0x00C00000,
	cran6502_BUS_SB_SP = 0x01000000,
	cran6502_BUS_SB_AI = 0x01400000,

	// SB Store ops
	cran6502_BUS_SB_STORE_MASK = 0x0E000000,
	cran6502_BUS_DL_SB = 0x02000000,
	cran6502_BUS_SP_SB = 0x04000000,
	cran6502_BUS_AC_SB = 0x06000000,
	cran6502_BUS_X_SB  = 0x08000000,
	cran6502_BUS_Y_SB  = 0x0A000000,
	cran6502_BUS_AD_SB = 0x0C000000,

	// SB ops
	cran6502_BUS_SB_AC_AI = cran6502_BUS_DB_SB | cran6502_BUS_AC_SB | cran6502_BUS_SB_AI,
	cran6502_BUS_SB_AD_AC = cran6502_BUS_DB_SB | cran6502_BUS_AD_SB | cran6502_BUS_SB_AC,
	cran6502_BUS_SB_X_AI = cran6502_BUS_DB_SB | cran6502_BUS_X_SB | cran6502_BUS_SB_AI,

	// ADL Load ops
	cran6502_BUS_ADL_LOAD_MASK = 0x00070000,
	cran6502_BUS_ADL_ABL = 0x00010000,

	// ADL Store ops
	cran6502_BUS_ADL_STORE_MASK = 0x00380000,
	cran6502_BUS_DL_ADL = 0x00080000,
	cran6502_BUS_PCL_ADL = 0x00100000,

	// ADL ops
	cran6502_BUS_ADL_DL_ABL = cran6502_BUS_ADL_ADH | cran6502_BUS_DL_ADL | cran6502_BUS_ADL_ABL,
	cran6502_BUS_ADL_PCL_ABL = cran6502_BUS_ADL_ADH | cran6502_BUS_PCL_ADL | cran6502_BUS_ADL_ABL,

	// ADH Load ops
	cran6502_BUS_ADH_LOAD_MASK = 0x01C00000,
	cran6502_BUS_ADH_ABH = 0x00400000,

	// ADH Store ops
	cran6502_BUS_ADH_STORE_MASK = 0x0E000000,
	cran6502_BUS_ZERO_ADH = 0x02000000,
	cran6502_BUS_PCH_ADH = 0x04000000,

	// ADH ops
	cran6502_BUS_ADH_ZERO_ABH = cran6502_BUS_ADL_ADH | cran6502_BUS_ZERO_ADH | cran6502_BUS_ADH_ABH,
	cran6502_BUS_ADH_PCH_ABH = cran6502_BUS_ADL_ADH | cran6502_BUS_PCH_ADH | cran6502_BUS_ADH_ABH,

	// ALU
	cran6502_ALU_OP_MASK = 0x00007FFF,
	cran6502_ALU_OP_ADD = 0x00000001,
	cran6502_ALU_OP_AND = 0x00000002,
	cran6502_ALU_OP_EOR = 0x00000003,
	cran6502_ALU_OP_OR  = 0x00000004,
	cran6502_ALU_OP_SHR  = 0x00000005,

	// ALU ops
	cran6502_ALU_ADD = cran6502_UNIT_ALU | cran6502_ALU_OP_ADD,

	// PC
	cran6502_PC_INC = 0x00001000,
	cran6502_BUS_ADH_PC_AB = cran6502_BUS_ADL_PCL_ABL | cran6502_BUS_ADH_PCH_ABH,
	cran6502_PC_READ = cran6502_PC_INC | cran6502_UNIT_MEM_READ
};

#define PHASE_01(a) (uint64_t)(a) << 32
#define PHASE_02(a) (a)

// Indexed by TI[T]
// T1 is always a NOP, it's an OP_FETCH cycle
static const uint64_t ROM[UINT8_MAX][7] =
{
	[0x69] = { PHASE_02(cran6502_PC_READ), PHASE_01(cran6502_BUS_ADH_PC_AB) | PHASE_02(cran6502_PC_READ | cran6502_BUS_DB_DL_BI | cran6502_BUS_SB_AC_AI), PHASE_01(cran6502_ALU_ADD) | PHASE_02(cran6502_BUS_SB_AD_AC) }, // ADC imm
	[0x85] = { PHASE_02(cran6502_PC_READ), PHASE_01(cran6502_BUS_ADH_PC_AB) | PHASE_02(cran6502_PC_READ | cran6502_BUS_ADL_DL_ABL | cran6502_BUS_ADH_ZERO_ABH), PHASE_01(cran6502_BUS_DB_AC_DOR) | PHASE_02(cran6502_UNIT_MEM_WRITE) }, // STA, zpg
	[0xA9] = { PHASE_02(cran6502_PC_READ), PHASE_01(cran6502_BUS_ADH_PC_AB) | PHASE_02(cran6502_PC_READ | cran6502_BUS_DB_DL_AC) }, // LDA, imm
	[0xEA] = { 0 }, // NOP
};

static const uint64_t OP_FETCH = PHASE_01(cran6502_BUS_ADH_PC_AB) | PHASE_02(cran6502_PC_READ);

void cran6502_backend(uint64_t UOP)
{
	// ALU
	{
		if ((UOP & cran6502_UNIT_ALU) != 0)
		{
			// TODO: Support flag modifications.
			AD = cran6502_mux8(AD, AI + BI, cran6502_eq(UOP & cran6502_ALU_OP_MASK, cran6502_ALU_OP_ADD));
			AD = cran6502_mux8(AD, AI & BI, cran6502_eq(UOP & cran6502_ALU_OP_MASK, cran6502_ALU_OP_AND));
			AD = cran6502_mux8(AD, AI ^ BI, cran6502_eq(UOP & cran6502_ALU_OP_MASK, cran6502_ALU_OP_EOR));
			AD = cran6502_mux8(AD, AI | BI, cran6502_eq(UOP & cran6502_ALU_OP_MASK, cran6502_ALU_OP_OR));
			AD = cran6502_mux8(AD, AI >> BI, cran6502_eq(UOP & cran6502_ALU_OP_MASK, cran6502_ALU_OP_SHR));
		}
	}

	// BUS
	{
		switch (UOP & cran6502_BUS_MASK)
		{
		case cran6502_BUS_DB_SB:
		{
			uint8_t DB = 0;
			// Store to DB
			DB = cran6502_mux8(DB, DL, cran6502_eq(UOP & cran6502_BUS_DB_STORE_MASK, cran6502_BUS_DL_DB));
			DB = cran6502_mux8(DB, AC, cran6502_eq(UOP & cran6502_BUS_DB_STORE_MASK, cran6502_BUS_AC_DB));
			DB = cran6502_mux8(DB, *(uint8_t*)&SR, cran6502_eq(UOP & cran6502_BUS_DB_STORE_MASK, cran6502_BUS_SR_DB));

			// Load from DB
			AC = cran6502_mux8(AC, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_AC));
			DOR = cran6502_mux8(DOR, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_DOR));
			BI = cran6502_mux8(BI, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_BI));
			PCL = cran6502_mux8(PCL, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_PCL));
			PCH = cran6502_mux8(PCH, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_PCH));

			uint8_t SB = 0;
			// Store to SB
			SB = cran6502_mux8(SB, DL, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_DL_SB));
			SB = cran6502_mux8(SB, SP, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_SP_SB));
			SB = cran6502_mux8(SB, AC, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_AC_SB));
			SB = cran6502_mux8(SB, X, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_X_SB));
			SB = cran6502_mux8(SB, Y, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_Y_SB));
			SB = cran6502_mux8(SB, AD, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_AD_SB));

			// Load from SB
			X = cran6502_mux8(X, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_X));
			AC = cran6502_mux8(AC, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_AC));
			Y = cran6502_mux8(Y, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_Y));
			SP = cran6502_mux8(SP, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_SP));
			AI = cran6502_mux8(AI, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_AI));
		}
		break;

		case cran6502_BUS_ADL_ADH:
		{
			uint8_t ADL = 0;
			// Store to ADL
			ADL = cran6502_mux8(ADL, DL, cran6502_eq(UOP & cran6502_BUS_ADL_STORE_MASK, cran6502_BUS_DL_ADL));
			ADL = cran6502_mux8(ADL, PCL, cran6502_eq(UOP & cran6502_BUS_ADL_STORE_MASK, cran6502_BUS_PCL_ADL));

			// Load from ADL
			ABL = cran6502_mux8(ABL, ADL, cran6502_eq(UOP & cran6502_BUS_ADL_LOAD_MASK, cran6502_BUS_ADL_ABL));

			uint8_t ADH = 0;
			// Store to ADH
			ADH = cran6502_mux8(ADH, 0, cran6502_eq(UOP & cran6502_BUS_ADH_STORE_MASK, cran6502_BUS_ZERO_ADH));
			ADH = cran6502_mux8(ADH, PCH, cran6502_eq(UOP & cran6502_BUS_ADH_STORE_MASK, cran6502_BUS_PCH_ADH));

			// Load from ADH
			ABH = cran6502_mux8(ABH, ADH, cran6502_eq(UOP & cran6502_BUS_ADH_LOAD_MASK, cran6502_BUS_ADH_ABH));
		}
		break;
		}
	}
}

void cran6502_clock_cycle(void)
{
	// --phase 01--

	uint64_t UOP = 0;
	{
		// TIMING
		uint8_t T_2_DECODE;
		{
			T = T << 1;
			T_2_DECODE = T;
		}

		// DECODE
		{
			UOP = ROM[IR][TI[T_2_DECODE]];
			T = cran6502_mux8(T, T1, cran6502_eq(((UOP >> 32) & cran6502_UNIT_MEM_MASK) | (UOP & cran6502_UNIT_MEM_MASK), 0)); // Is this a non-mem op?
			UOP = cran6502_mux64(UOP, UOP | OP_FETCH, cran6502_eq(((UOP >> 32) & cran6502_UNIT_MEM_MASK) | (UOP & cran6502_UNIT_MEM_MASK), 0));
		}

		// PC
		{
			PCLS = PCL;
			PCHS = PCH;

			uint8_t h = PCLS & 0x80;
			PCLS++;
			uint8_t c = (h & (PCHS ^ h)) >> 7; // High bit not set anymore? That's our carry bit.
			PCHS += c;
		}

		cran6502_backend(UOP >> 32);
	}

	// --phase 02--
	{
		// DATA BUS TRISTATE BUFFERS
		{
			PD = cran6502_mux8(PD, MEM[(ABH << 8) | ABL], cran6502_eq(UOP & cran6502_UNIT_MEM_READ, cran6502_UNIT_MEM_READ));
			DL = cran6502_mux8(DL, MEM[(ABH << 8) | ABL], cran6502_eq(UOP & cran6502_UNIT_MEM_READ, cran6502_UNIT_MEM_READ));

			MEM[(ABH << 8) | ABL] = cran6502_mux8(MEM[(ABH << 8) | ABL], DOR, cran6502_eq(UOP & cran6502_UNIT_MEM_WRITE, cran6502_UNIT_MEM_WRITE));
		}

		// PREDECODE
		uint8_t PD_2_IR;
		{
			// TODO: Interrupts
			PD_2_IR = PD;
		}

		// DECODE
		{
			IR = cran6502_mux8(IR, PD_2_IR, cran6502_eq(T, T1)); // Use T, if we prefetched, then we can set ourselves up for the next instruction cycle.
		}

		// PC
		{
			PCL = cran6502_mux8(PCL, PCLS, cran6502_eq(UOP & cran6502_PC_INC, cran6502_PC_INC));
			PCH = cran6502_mux8(PCH, PCHS, cran6502_eq(UOP & cran6502_PC_INC, cran6502_PC_INC));
		}

		cran6502_backend(UOP & 0xFFFFFFFF);
	}
}

#endif // CRANBERRY_6502_IMPLEMENTATION
