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

void cran6502_map(uint16_t address, uint8_t* memory, uint8_t size);
void cran6502_interrupt(cran6502_interrupts_t signals);
void cran6502_clock_cycle(void);

#endif // __CRANBERRY_6502_H

#ifdef CRANBERRY_6502_IMPLEMENTATION

#include <string.h>

// Processor Registers
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

// GENERAL PURPOSE (ish)
static uint8_t X, Y, AC, SP, IR, ABL, ABH, AD;

static uint8_t PCL, PCH;

// Front-end
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

uint8_t TI[] = { [T1] = 0, [T1X] = 1,[T2] = 2,[T3] = 3,[T4] = 4,[T5] = 5,[T6] = 6 };

static uint8_t PD, T = T0, IRQ;

// BUS
enum
{
	RW_NONE = 0x00,
	RW_WRITE = 0x01,
	RW_READ = 0x02,
};

static uint8_t DOR, DL, RW;

// ALU
static uint8_t AI, BI;

// Memory
static uint8_t MEM[UINT16_MAX];

void cran6502_map(uint16_t address, uint8_t* memory, uint8_t size)
{
	memcpy(&MEM[address], memory, size);
}

void cran6502_interrupt(cran6502_interrupts_t signals)
{
	IRQ |= *(uint8_t*)&signals;
}

static void cran6502_inc_PC(void)
{
	uint8_t h = PCL & 0x80;
	PCL++;
	uint8_t c = (h & (PCH ^ h)) >> 7; // High bit not set anymore? That's our carry bit.
	PCH += c;
}

// 0x01 if equal, 0x00 if not.
static uint32_t cran6502_eq(uint32_t l, uint32_t r)
{
	uint32_t m = ~(l ^ r);

	m &= m >> 1;
	m &= m >> 2;
	m &= m >> 4;
	m &= m >> 8;
	m &= m >> 16;
	return m;
}

static uint32_t cran6502_repeat(uint32_t l)
{
	l |= l << 1;
	l |= l << 2;
	l |= l << 4;
	l |= l << 8;
	l |= l << 16;
	return l;
}

// Multiplexor: if (c) ? l : r
static uint32_t cran6502_mux(uint32_t l, uint32_t r, uint32_t c)
{
	c = cran6502_repeat(c);
	return (l & ~c) | (r & c);
}

// Mask: if (c) ? l : 0
static uint32_t cran6502_mask(uint32_t l, uint32_t c)
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
	cran6502_UNIT_BUS = 0x40000000,
	cran6502_UNIT_MEM = 0x10000000,

	// BUS
	cran6502_UNIT_BUS_MASK = 0x1FFF0000,

	cran6502_BUS_MASK = 0x20000000,
	cran6502_BUS_DB_SB = 0x00000000,
	cran6502_BUS_ADH_ADL = 0x20000000,

	cran6502_BUS_OP_MASK = 0x1FFF0000,

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
	cran6502_BUS_DL_BI = cran6502_UNIT_BUS | cran6502_BUS_DB_SB | cran6502_BUS_DL_DB | cran6502_BUS_DB_BI,
	cran6502_BUS_DL_AC = cran6502_UNIT_BUS | cran6502_BUS_DB_SB | cran6502_BUS_DL_DB | cran6502_BUS_DB_AC,

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
	cran6502_BUS_AC_AI = cran6502_UNIT_BUS | cran6502_BUS_DB_SB | cran6502_BUS_AC_SB | cran6502_BUS_SB_AI,
	cran6502_BUS_AD_AC = cran6502_UNIT_BUS | cran6502_BUS_DB_SB | cran6502_BUS_AD_SB | cran6502_BUS_SB_AC,

	// ALU
	cran6502_ALU_OP_MASK = 0x0000FFFF,
	cran6502_ALU_OP_ADD = 0x00000001,

	// ALU ops
	cran6502_ALU_ADD = cran6502_UNIT_ALU | cran6502_ALU_OP_ADD,
};

// Indexed by TI[T]
// T1 is always a NOP, it's an OP_FETCH cycle
uint32_t ROM[UINT8_MAX][7] =
{
	[0x69] = { cran6502_UNIT_MEM, cran6502_UNIT_MEM | cran6502_BUS_DL_BI | cran6502_BUS_AC_AI, cran6502_ALU_ADD | cran6502_BUS_AD_AC }, // ADC imm
	[0xA9] = { cran6502_UNIT_MEM, cran6502_UNIT_MEM | cran6502_BUS_DL_AC }
};

void cran6502_clock_cycle(void)
{
	// --phase 01--

	uint32_t UOP = 0;
	{
		// TIMING
		uint8_t T_2_DECODE;
		{
			T = T << 1;
			T_2_DECODE = T;
		}

		// DECODE
		{
			if (T_2_DECODE == T1)
			{
				ABL = PCL;
				ABH = PCH;
				cran6502_inc_PC();

				RW = RW_READ;
			}
			else
			{
				UOP = ROM[IR][TI[T_2_DECODE]];

				if (UOP == cran6502_NOP)
				{
					if (T_2_DECODE != T1) // If we're not in OP_FETCH, then this NOP is a terminator for our microops.
					{
						T = T0; // We're done with this OP, move to the next.
					}
				}
				else if ((UOP & cran6502_UNIT_MEM) == 0) // Not a memory op? We can prefetch our next instruction
				{
					ABL = PCL;
					ABH = PCH;
					cran6502_inc_PC();

					RW = RW_READ;

					T = T1; // Reset T, sent to decode unit already anyways. We want to skip fetch next instruction cycle.
				}
			}
		}

		// DATA FETCH
		{
			if ((UOP & cran6502_UNIT_MEM) != 0) // Command doesn't use ALU? Great, data fetch
			{
				ABL = PCL;
				ABH = PCH;
				cran6502_inc_PC();
				RW = RW_READ;
			}
		}

		// ALU
		{
			if ((UOP & cran6502_UNIT_ALU) != 0)
			{
				switch (UOP & cran6502_ALU_OP_MASK)
				{
				case cran6502_ALU_OP_ADD:
					AD = AI + BI;
					SR.C = ((AI & BI) & 0x80) >> 7; // TODO: Fix
					break;
				}
			}
		}
	}

	// --phase 02--
	{
		// DATA BUS TRISTATE BUFFERS
		{
			if (RW == RW_READ)
			{
				PD = MEM[(ABH << 8) | ABL];
				DL = MEM[(ABH << 8) | ABL];
			}
			else if (RW == RW_WRITE)
			{
				MEM[(ABH << 8) | ABL] = DOR;
			}

			RW = RW_NONE;
		}

		// PREDECODE
		uint8_t PD_2_IR;
		{
			// TODO: Interrupts
			PD_2_IR = PD;
		}

		// DECODE
		{
			IR = cran6502_mux(IR, PD_2_IR, cran6502_eq(T, T1)); // Use T, if we prefetched, then we can set ourselves up for the next instruction cycle.
		}

		// BUS
		{
			if ((UOP & cran6502_UNIT_MASK) == cran6502_UNIT_BUS)
			{
				switch (UOP & cran6502_BUS_MASK)
				{
				case cran6502_BUS_DB_SB:
				{
					uint8_t DB = 0;
					// Store to DB
					DB = cran6502_mux(DB, DL, cran6502_eq(UOP & cran6502_BUS_DB_STORE_MASK, cran6502_BUS_DL_DB));
					DB = cran6502_mux(DB, AC, cran6502_eq(UOP & cran6502_BUS_DB_STORE_MASK, cran6502_BUS_AC_DB));
					DB = cran6502_mux(DB, *(uint8_t*)&SR, cran6502_eq(UOP & cran6502_BUS_DB_STORE_MASK, cran6502_BUS_SR_DB));

					// Load from DB
					AC = cran6502_mux(AC, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_AC));
					DOR = cran6502_mux(DOR, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_DOR));
					BI = cran6502_mux(BI, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_BI));
					PCL = cran6502_mux(PCL, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_PCL));
					PCH = cran6502_mux(PCH, DB, cran6502_eq(UOP & cran6502_BUS_DB_LOAD_MASK, cran6502_BUS_DB_PCH));

					uint8_t SB = 0;
					// Store to SB
					SB = cran6502_mux(SB, DL, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_DL_SB));
					SB = cran6502_mux(SB, SP, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_SP_SB));
					SB = cran6502_mux(SB, AC, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_AC_SB));
					SB = cran6502_mux(SB, X, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_X_SB));
					SB = cran6502_mux(SB, Y, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_Y_SB));
					SB = cran6502_mux(SB, AD, cran6502_eq(UOP & cran6502_BUS_SB_STORE_MASK, cran6502_BUS_AD_SB));

					// Load from SB
					X = cran6502_mux(X, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_X));
					AC = cran6502_mux(AC, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_AC));
					Y = cran6502_mux(Y, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_Y));
					SP = cran6502_mux(SP, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_SP));
					AI = cran6502_mux(AI, SB, cran6502_eq(UOP & cran6502_BUS_SB_LOAD_MASK, cran6502_BUS_SB_AI));
				}
				break;
				}
			}
		}

		// ALU

	}
}

#endif // CRANBERRY_6502_IMPLEMENTATION