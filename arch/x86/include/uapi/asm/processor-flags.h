#ifndef _UAPI_ASM_X86_PROCESSOR_FLAGS_H
#define _UAPI_ASM_X86_PROCESSOR_FLAGS_H
/* Various flags defined: can be included from assembler. */

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
/*
 * EFLAGS bits
 */
#define X86_EFLAGS_CF	0x00000001 /* Carry Flag */
#define X86_EFLAGS_BIT1	0x00000002 /* Bit 1 - always on */
#define X86_EFLAGS_PF	0x00000004 /* Parity Flag */
#define X86_EFLAGS_AF	0x00000010 /* Auxiliary carry Flag */
#define X86_EFLAGS_ZF	0x00000040 /* Zero Flag */
#define X86_EFLAGS_SF	0x00000080 /* Sign Flag */
#define X86_EFLAGS_TF	0x00000100 /* Trap Flag */
#define X86_EFLAGS_IF	0x00000200 /* Interrupt Flag */
#define X86_EFLAGS_DF	0x00000400 /* Direction Flag */
#define X86_EFLAGS_OF	0x00000800 /* Overflow Flag */
#define X86_EFLAGS_IOPL	0x00003000 /* IOPL mask */
#define X86_EFLAGS_NT	0x00004000 /* Nested Task */
#define X86_EFLAGS_RF	0x00010000 /* Resume Flag */
#define X86_EFLAGS_VM	0x00020000 /* Virtual Mode */
#define X86_EFLAGS_AC	0x00040000 /* Alignment Check */
#define X86_EFLAGS_VIF	0x00080000 /* Virtual Interrupt Flag */
#define X86_EFLAGS_VIP	0x00100000 /* Virtual Interrupt Pending */
#define X86_EFLAGS_ID	0x00200000 /* CPUID detection flag */
<<<<<<< HEAD
=======
#include <linux/const.h>

/*
 * EFLAGS bits
 */
#define X86_EFLAGS_CF_BIT	0 /* Carry Flag */
#define X86_EFLAGS_CF		_BITUL(X86_EFLAGS_CF_BIT)
#define X86_EFLAGS_FIXED_BIT	1 /* Bit 1 - always on */
#define X86_EFLAGS_FIXED	_BITUL(X86_EFLAGS_FIXED_BIT)
#define X86_EFLAGS_PF_BIT	2 /* Parity Flag */
#define X86_EFLAGS_PF		_BITUL(X86_EFLAGS_PF_BIT)
#define X86_EFLAGS_AF_BIT	4 /* Auxiliary carry Flag */
#define X86_EFLAGS_AF		_BITUL(X86_EFLAGS_AF_BIT)
#define X86_EFLAGS_ZF_BIT	6 /* Zero Flag */
#define X86_EFLAGS_ZF		_BITUL(X86_EFLAGS_ZF_BIT)
#define X86_EFLAGS_SF_BIT	7 /* Sign Flag */
#define X86_EFLAGS_SF		_BITUL(X86_EFLAGS_SF_BIT)
#define X86_EFLAGS_TF_BIT	8 /* Trap Flag */
#define X86_EFLAGS_TF		_BITUL(X86_EFLAGS_TF_BIT)
#define X86_EFLAGS_IF_BIT	9 /* Interrupt Flag */
#define X86_EFLAGS_IF		_BITUL(X86_EFLAGS_IF_BIT)
#define X86_EFLAGS_DF_BIT	10 /* Direction Flag */
#define X86_EFLAGS_DF		_BITUL(X86_EFLAGS_DF_BIT)
#define X86_EFLAGS_OF_BIT	11 /* Overflow Flag */
#define X86_EFLAGS_OF		_BITUL(X86_EFLAGS_OF_BIT)
#define X86_EFLAGS_IOPL_BIT	12 /* I/O Privilege Level (2 bits) */
#define X86_EFLAGS_IOPL		(_AC(3,UL) << X86_EFLAGS_IOPL_BIT)
#define X86_EFLAGS_NT_BIT	14 /* Nested Task */
#define X86_EFLAGS_NT		_BITUL(X86_EFLAGS_NT_BIT)
#define X86_EFLAGS_RF_BIT	16 /* Resume Flag */
#define X86_EFLAGS_RF		_BITUL(X86_EFLAGS_RF_BIT)
#define X86_EFLAGS_VM_BIT	17 /* Virtual Mode */
#define X86_EFLAGS_VM		_BITUL(X86_EFLAGS_VM_BIT)
#define X86_EFLAGS_AC_BIT	18 /* Alignment Check/Access Control */
#define X86_EFLAGS_AC		_BITUL(X86_EFLAGS_AC_BIT)
#define X86_EFLAGS_AC_BIT	18 /* Alignment Check/Access Control */
#define X86_EFLAGS_AC		_BITUL(X86_EFLAGS_AC_BIT)
#define X86_EFLAGS_VIF_BIT	19 /* Virtual Interrupt Flag */
#define X86_EFLAGS_VIF		_BITUL(X86_EFLAGS_VIF_BIT)
#define X86_EFLAGS_VIP_BIT	20 /* Virtual Interrupt Pending */
#define X86_EFLAGS_VIP		_BITUL(X86_EFLAGS_VIP_BIT)
#define X86_EFLAGS_ID_BIT	21 /* CPUID detection */
#define X86_EFLAGS_ID		_BITUL(X86_EFLAGS_ID_BIT)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

/*
 * Basic CPU control in CR0
 */
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
#define X86_CR0_PE	0x00000001 /* Protection Enable */
#define X86_CR0_MP	0x00000002 /* Monitor Coprocessor */
#define X86_CR0_EM	0x00000004 /* Emulation */
#define X86_CR0_TS	0x00000008 /* Task Switched */
#define X86_CR0_ET	0x00000010 /* Extension Type */
#define X86_CR0_NE	0x00000020 /* Numeric Error */
#define X86_CR0_WP	0x00010000 /* Write Protect */
#define X86_CR0_AM	0x00040000 /* Alignment Mask */
#define X86_CR0_NW	0x20000000 /* Not Write-through */
#define X86_CR0_CD	0x40000000 /* Cache Disable */
#define X86_CR0_PG	0x80000000 /* Paging */
<<<<<<< HEAD
=======
#define X86_CR0_PE_BIT		0 /* Protection Enable */
#define X86_CR0_PE		_BITUL(X86_CR0_PE_BIT)
#define X86_CR0_MP_BIT		1 /* Monitor Coprocessor */
#define X86_CR0_MP		_BITUL(X86_CR0_MP_BIT)
#define X86_CR0_EM_BIT		2 /* Emulation */
#define X86_CR0_EM		_BITUL(X86_CR0_EM_BIT)
#define X86_CR0_TS_BIT		3 /* Task Switched */
#define X86_CR0_TS		_BITUL(X86_CR0_TS_BIT)
#define X86_CR0_ET_BIT		4 /* Extension Type */
#define X86_CR0_ET		_BITUL(X86_CR0_ET_BIT)
#define X86_CR0_NE_BIT		5 /* Numeric Error */
#define X86_CR0_NE		_BITUL(X86_CR0_NE_BIT)
#define X86_CR0_WP_BIT		16 /* Write Protect */
#define X86_CR0_WP		_BITUL(X86_CR0_WP_BIT)
#define X86_CR0_AM_BIT		18 /* Alignment Mask */
#define X86_CR0_AM		_BITUL(X86_CR0_AM_BIT)
#define X86_CR0_NW_BIT		29 /* Not Write-through */
#define X86_CR0_NW		_BITUL(X86_CR0_NW_BIT)
#define X86_CR0_CD_BIT		30 /* Cache Disable */
#define X86_CR0_CD		_BITUL(X86_CR0_CD_BIT)
#define X86_CR0_PG_BIT		31 /* Paging */
#define X86_CR0_PG		_BITUL(X86_CR0_PG_BIT)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

/*
 * Paging options in CR3
 */
<<<<<<< HEAD
<<<<<<< HEAD
#define X86_CR3_PWT	0x00000008 /* Page Write Through */
#define X86_CR3_PCD	0x00000010 /* Page Cache Disable */
#define X86_CR3_PCID_MASK 0x00000fff /* PCID Mask */
=======
#define X86_CR3_PWT_BIT		3 /* Page Write Through */
#define X86_CR3_PWT		_BITUL(X86_CR3_PWT_BIT)
#define X86_CR3_PCD_BIT		4 /* Page Cache Disable */
#define X86_CR3_PCD		_BITUL(X86_CR3_PCD_BIT)
#define X86_CR3_PCID_MASK	_AC(0x00000fff,UL) /* PCID Mask */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
#define X86_CR3_PWT	0x00000008 /* Page Write Through */
#define X86_CR3_PCD	0x00000010 /* Page Cache Disable */
#define X86_CR3_PCID_MASK 0x00000fff /* PCID Mask */
>>>>>>> 2617302... source

/*
 * Intel CPU features in CR4
 */
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
#define X86_CR4_VME	0x00000001 /* enable vm86 extensions */
#define X86_CR4_PVI	0x00000002 /* virtual interrupts flag enable */
#define X86_CR4_TSD	0x00000004 /* disable time stamp at ipl 3 */
#define X86_CR4_DE	0x00000008 /* enable debugging extensions */
#define X86_CR4_PSE	0x00000010 /* enable page size extensions */
#define X86_CR4_PAE	0x00000020 /* enable physical address extensions */
#define X86_CR4_MCE	0x00000040 /* Machine check enable */
#define X86_CR4_PGE	0x00000080 /* enable global pages */
#define X86_CR4_PCE	0x00000100 /* enable performance counters at ipl 3 */
#define X86_CR4_OSFXSR	0x00000200 /* enable fast FPU save and restore */
#define X86_CR4_OSXMMEXCPT 0x00000400 /* enable unmasked SSE exceptions */
#define X86_CR4_VMXE	0x00002000 /* enable VMX virtualization */
#define X86_CR4_RDWRGSFS 0x00010000 /* enable RDWRGSFS support */
#define X86_CR4_PCIDE	0x00020000 /* enable PCID support */
#define X86_CR4_OSXSAVE 0x00040000 /* enable xsave and xrestore */
#define X86_CR4_SMEP	0x00100000 /* enable SMEP support */
#define X86_CR4_SMAP	0x00200000 /* enable SMAP support */
<<<<<<< HEAD
=======
#define X86_CR4_VME_BIT		0 /* enable vm86 extensions */
#define X86_CR4_VME		_BITUL(X86_CR4_VME_BIT)
#define X86_CR4_PVI_BIT		1 /* virtual interrupts flag enable */
#define X86_CR4_PVI		_BITUL(X86_CR4_PVI_BIT)
#define X86_CR4_TSD_BIT		2 /* disable time stamp at ipl 3 */
#define X86_CR4_TSD		_BITUL(X86_CR4_TSD_BIT)
#define X86_CR4_DE_BIT		3 /* enable debugging extensions */
#define X86_CR4_DE		_BITUL(X86_CR4_DE_BIT)
#define X86_CR4_PSE_BIT		4 /* enable page size extensions */
#define X86_CR4_PSE		_BITUL(X86_CR4_PSE_BIT)
#define X86_CR4_PAE_BIT		5 /* enable physical address extensions */
#define X86_CR4_PAE		_BITUL(X86_CR4_PAE_BIT)
#define X86_CR4_MCE_BIT		6 /* Machine check enable */
#define X86_CR4_MCE		_BITUL(X86_CR4_MCE_BIT)
#define X86_CR4_PGE_BIT		7 /* enable global pages */
#define X86_CR4_PGE		_BITUL(X86_CR4_PGE_BIT)
#define X86_CR4_PCE_BIT		8 /* enable performance counters at ipl 3 */
#define X86_CR4_PCE		_BITUL(X86_CR4_PCE_BIT)
#define X86_CR4_OSFXSR_BIT	9 /* enable fast FPU save and restore */
#define X86_CR4_OSFXSR		_BITUL(X86_CR4_OSFXSR_BIT)
#define X86_CR4_OSXMMEXCPT_BIT	10 /* enable unmasked SSE exceptions */
#define X86_CR4_OSXMMEXCPT	_BITUL(X86_CR4_OSXMMEXCPT_BIT)
#define X86_CR4_VMXE_BIT	13 /* enable VMX virtualization */
#define X86_CR4_VMXE		_BITUL(X86_CR4_VMXE_BIT)
#define X86_CR4_SMXE_BIT	14 /* enable safer mode (TXT) */
#define X86_CR4_SMXE		_BITUL(X86_CR4_SMXE_BIT)
#define X86_CR4_FSGSBASE_BIT	16 /* enable RDWRFSGS support */
#define X86_CR4_FSGSBASE	_BITUL(X86_CR4_FSGSBASE_BIT)
#define X86_CR4_PCIDE_BIT	17 /* enable PCID support */
#define X86_CR4_PCIDE		_BITUL(X86_CR4_PCIDE_BIT)
#define X86_CR4_OSXSAVE_BIT	18 /* enable xsave and xrestore */
#define X86_CR4_OSXSAVE		_BITUL(X86_CR4_OSXSAVE_BIT)
#define X86_CR4_SMEP_BIT	20 /* enable SMEP support */
#define X86_CR4_SMEP		_BITUL(X86_CR4_SMEP_BIT)
#define X86_CR4_SMAP_BIT	21 /* enable SMAP support */
#define X86_CR4_SMAP		_BITUL(X86_CR4_SMAP_BIT)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

/*
 * x86-64 Task Priority Register, CR8
 */
<<<<<<< HEAD
<<<<<<< HEAD
#define X86_CR8_TPR	0x0000000F /* task priority register */
=======
#define X86_CR8_TPR		_AC(0x0000000f,UL) /* task priority register */
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
#define X86_CR8_TPR	0x0000000F /* task priority register */
>>>>>>> 2617302... source

/*
 * AMD and Transmeta use MSRs for configuration; see <asm/msr-index.h>
 */

/*
 *      NSC/Cyrix CPU configuration register indexes
 */
#define CX86_PCR0	0x20
#define CX86_GCR	0xb8
#define CX86_CCR0	0xc0
#define CX86_CCR1	0xc1
#define CX86_CCR2	0xc2
#define CX86_CCR3	0xc3
#define CX86_CCR4	0xe8
#define CX86_CCR5	0xe9
#define CX86_CCR6	0xea
#define CX86_CCR7	0xeb
#define CX86_PCR1	0xf0
#define CX86_DIR0	0xfe
#define CX86_DIR1	0xff
#define CX86_ARR_BASE	0xc4
#define CX86_RCR_BASE	0xdc


#endif /* _UAPI_ASM_X86_PROCESSOR_FLAGS_H */
