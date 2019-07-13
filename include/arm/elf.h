/*
 * Copyright (c) 2007, Kohsuke Ohtani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _ARM_ELF_H
#define _ARM_ELF_H


#define EF_ARM_EABI_MASK	0xff000000
#define EF_ARM_EABI_UNKNOWN	0x00000000
#define EF_ARM_EABI_VER1	0x01000000
#define EF_ARM_EABI_VER2	0x02000000
#define EF_ARM_EABI_VER3	0x03000000
#define EF_ARM_EABI_VER4	0x04000000
#define EF_ARM_EABI_VER5	0x05000000

#define EF_ARM_BE8		0x00800000	/* ABI 4,5 */
#define EF_ARM_LE8		0x00400000	/* ABI 4,5 */
#define EF_ARM_MAVERICK_FLOAT	0x00000800	/* ABI 0 */
#define EF_ARM_VFP_FLOAT	0x00000400	/* ABI 0 */
#define EF_ARM_SOFT_FLOAT	0x00000200	/* ABI 0 */
#define EF_ARM_OLD_ABI		0x00000100	/* ABI 0 */
#define EF_ARM_NEW_ABI		0x00000080	/* ABI 0 */
#define EF_ARM_ALIGN8		0x00000040	/* ABI 0 */
#define EF_ARM_PIC		0x00000020	/* ABI 0 */
#define EF_ARM_MAPSYMSFIRST	0x00000010	/* ABI 2 */
#define EF_ARM_APCS_FLOAT	0x00000010	/* ABI 0, floats in fp regs */
#define EF_ARM_DYNSYMSUSESEGIDX	0x00000008	/* ABI 2 */
#define EF_ARM_APCS_26		0x00000008	/* ABI 0 */
#define EF_ARM_SYMSARESORTED	0x00000004	/* ABI 1,2 */
#define EF_ARM_INTERWORK	0x00000004	/* ABI 0 */
#define EF_ARM_HASENTRY		0x00000002	/* All */
#define EF_ARM_RELEXEC		0x00000001	/* All */


/*
 * Relocation type
 */
#define	R_ARM_NONE	0
#define	R_ARM_PC24	1
#define	R_ARM_ABS32	2
#define R_ARM_REL32 3
#define R_ARM_LDR_PC_G0 4
#define R_ARM_ABS16 5
#define R_ARM_ABS12 6
#define R_ARM_THM_ABS5 7
#define R_ARM_ABS8 8
#define R_ARM_SBREL32 9
#define R_ARM_THM_CALL 10
#define R_ARM_THM_PC8 11
#define R_ARM_BREL_ADJ 12
#define R_ARM_TLS_DESC 13
#define R_ARM_THM_SWI8 14
#define R_ARM_XPC25 15
#define R_ARM_THM_XPC22 16
#define R_ARM_TLS_DTPMOD32 17
#define R_ARM_TLS_DTPOFF32 18
#define R_ARM_TLS_TPOFF32 19
#define R_ARM_COPY 20
#define R_ARM_GLOB_DAT 21
#define R_ARM_JUMP_SLOT 22
#define R_ARM_RELATIVE 23
#define R_ARM_GOTOFF32 24
#define R_ARM_BASE_PREL 25
#define R_ARM_GOT_BREL 26
#define	R_ARM_PLT32	27
#define	R_ARM_CALL	28
#define R_ARM_JUMP24	29
#define R_ARM_THM_JUMP24 30
#define R_ARM_BASE_ABS 31
#define R_ARM_ALU_PCREL_7_0 32
#define R_ARM_ALU_PCREL_15_8 33
#define R_ARM_ALU_PCREL_23_15 34
#define R_ARM_LDR_SBREL_11_0_NC 35
#define R_ARM_ALU_SBREL_11_0_NC 36
#define R_ARM_ALU_SBREL_27_20_CK 37
#define R_ARM_TARGET1 38
#define R_ARM_SBREL31 39
#define R_ARM_V4BX	40
#define R_ARM_TARGET2	41
#define R_ARM_PREL31	42
#define R_ARM_MOVW_ABS_NC	43
#define R_ARM_MOVT_ABS	44
#define R_ARM_MOVW_PREL_NC	45
#define R_ARM_MOVT_PREL	46
#define R_ARM_THM_MOVW_ABS_NC	47
#define R_ARM_THM_MOVT_ABS	48
#define R_ARM_THM_MOVW_PREL_NC	49
#define R_ARM_THM_MOVT_PREL 50
#define R_ARM_THM_JUMP19 51
#define R_ARM_THM_JUMP6 52
#define R_ARM_THM_ALU_PREL_11_0 53
#define R_ARM_THM_PC12 54
#define R_ARM_ABS32_NOI 55
#define R_ARM_REL32_NOI 56
#define R_ARM_ALU_PC_G0_NC 57
#define R_ARM_ALU_PC_G0 58
#define R_ARM_ALU_PC_G1_NC 59


#endif /* !_ARM_ELF_H */
