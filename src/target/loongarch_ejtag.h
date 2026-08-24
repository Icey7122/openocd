/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 * LoongArch EJTAG probe interface.
 *
 * The Loongson 2K300 TAP uses a 4-bit instruction register.  The target
 * configuration supplies the IDCODE, ADDRESS, DATA, CONTROL, PCSAMPLE, ALL
 * and FASTDATA codes.
 ***************************************************************************/

#ifndef OPENOCD_TARGET_LOONGARCH_EJTAG_H
#define OPENOCD_TARGET_LOONGARCH_EJTAG_H

#include <jtag/jtag.h>
#include <stdbool.h>

struct loongarch_fastdata_session;

struct loongarch_ejtag {
	struct jtag_tap *tap;

	unsigned int ir_len;	/* LoongArch: 4 */
	uint32_t ir_impcode;	/* 1: implementation marker */
	uint32_t ir_idcode;	/* 2: device identification */
	uint32_t ir_address;	/* 3 */
	uint32_t ir_data;	/* 4 */
	uint32_t ir_control;	/* 5 */
	uint32_t ir_pcsample;	/* 6 */
	uint32_t ir_all;	/* 7: CONTROL + DATA + ADDRESS */
	uint32_t ir_fastdata;	/* 8: SPrAcc + DATA */
	uint32_t ir_bypass;	/* 0 */

	uint32_t impcode;
	uint32_t idcode;
	uint32_t ejtag_ctrl;
	bool use_all;
	bool use_fastdata;
	struct loongarch_fastdata_session *fastdata_session;
};

/* IR=ALL presents the three EJTAG registers as one 160-bit DR.  The JTAG
 * shift order, from TDO/TDI bit 0 upward, is CONTROL, DATA, ADDRESS. */
struct loongarch_ejtag_all {
	uint32_t control;
	uint64_t data;
	uint64_t address;
};

/* IR=FASTDATA presents the processor-access completion flag followed by the
 * 64-bit DATA register.  Shifting zero into SPrAcc completes one pending
 * processor access without a separate CONTROL=0xc000 scan. */
struct loongarch_ejtag_fastdata {
	bool spracc;
	uint64_t data;
};

/* CONTROL (DCSR side of the probe) bits, ECR-compatible.
 * Verified against the official tool's access pattern on 2K300:
 *   steady/poll    : 0x4c000 = PrAcc|ProbEn|ProbVec
 *   assert break   : 0x4d000 = PrAcc|ProbEn|ProbVec|JtagBrk (one shot)
 *   clear PrAcc    : 0xc000  = ProbEn|ProbVec
 *
 * PRnW semantics (official tool behavior and LS2K300 observation):
 *   PRnW = 1 -> the CPU is writing (st.d to dmseg): the probe must READ
 *               the value out of the DATA register;
 *   PRnW = 0 -> the CPU is reading (ld.d from dmseg): the probe must WRITE
 *               the value into the DATA register.
 */
#define LA_CTRL_BRKST		(1u << 3)	/* IN_DEBUG_MODE */
#define LA_CTRL_JTAGBRK		(1u << 12)	/* debug interrupt request */
#define LA_CTRL_PROBEVEC	(1u << 14)	/* 1: dmseg 0xFF200200, 0: BIOS vector */
#define LA_CTRL_PROBEN		(1u << 15)	/* probe enable */
#define LA_CTRL_PRACC		(1u << 18)	/* processor access pending */
#define LA_CTRL_PRNW		(1u << 19)	/* 1 = read, 0 = write */

/* LoongArch return-from-debug instruction word.
 * Extracted from the official la_dbg_tool_usb jtag_cont() (word at
 * rsp+4 == 0x06483800, module length 4). */
#define LA_DERET		0x06483800u

/* LoongArch dmseg mailbox layout.
 * Decoded from the official la_dbg_tool_usb modules (jtag_outx/lisa_aui_x):
 * the mailbox base is 0xDB000000000FFE00 inside the 1MB dmseg window
 * 0xDB00000000000000-0xDB000000000FFFFF.  Slot offsets:
 *   0x1E0  address in, 0x1E8/0x1F0 scratch save, 0x1F0 reg FIFO,
 *   0x200+ data words.  CSR 0x502 is used to preserve r15. */
#define LA_DMSEG_BASE_ADDR	0xDB00000000000000ull
#define LA_DMSEG_SIZE		0x100000ull
#define LA_FASTDATA_ADDR	0xDB000000000D0000ull
#define LA_PRACC_BASE_ADDR	0xDB000000000FFE00ull
#define LA_PRACC_SLOT_ADDR	0x1E0ull	/* host-supplied address (unused) */
#define LA_PRACC_SLOT_SAVE	0x1E8ull	/* r2 spill */
#define LA_PRACC_SLOT_FIFO	0x1F0ull	/* sequential data FIFO */
#define LA_PRACC_SLOT_SAVE2	0x1F8ull	/* r3 spill */
#define LA_PRACC_MAX_WORDS	16

/* CSR used by the official tool to preserve r15 across debug modules */
#define LA_CSR_DEBUG_SAVE	0x502u

/* LoongArch instruction encodings (validated against LLVM tablegen and
 * byte-for-byte against the official la_dbg_tool_usb modules). */
#define LA_LU12I_W(rd, imm20)	(0x14000000u | (((imm20) & 0xFFFFFu) << 5) | ((rd) & 0x1Fu))
#define LA_ORI(rd, rj, imm12)	(0x03800000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_LU32I_D(rd, imm20)	(0x16000000u | (((imm20) & 0xFFFFFu) << 5) | ((rd) & 0x1Fu))
#define LA_LU52I_D(rd, rj, imm12)	(0x03000000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_ADDI_D(rd, rj, imm12)	(0x02C00000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_LD_B(rd, rj, imm12)	(0x28000000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_LD_H(rd, rj, imm12)	(0x28400000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_LD_W(rd, rj, imm12)	(0x28800000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_LD_D(rd, rj, imm12)	(0x28C00000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_ST_B(rd, rj, imm12)	(0x29000000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_ST_H(rd, rj, imm12)	(0x29400000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_ST_W(rd, rj, imm12)	(0x29800000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_ST_D(rd, rj, imm12)	(0x29C00000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_OR(rd, rj, rk)	(0x00150000u | (((rk) & 0x1Fu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_ANDI(rd, rj, imm12)	(0x03400000u | (((imm12) & 0xFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_CPUCFG(rd, rj)	(0x00006C00u | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_IOCSR_2R(op, rd, rj)	((op) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_CSRWR(rd, csr)	(0x04000020u | (((csr) & 0x3FFFu) << 10) | ((rd) & 0x1Fu))
#define LA_CSRRD(rd, csr)	(0x04000000u | (((csr) & 0x3FFFu) << 10) | ((rd) & 0x1Fu))
#define LA_DBAR		0x38720000u
#define LA_IBAR		0x38728000u
#define LA_ANDI_ZERO	0x03400000u
#define LA_B(offs16)	(0x50000000u | (((offs16) & 0xFFFFu) << 10) | (((offs16) >> 16) & 0x3FFu))
#define LA_BEQ(rj, rd, offs16)	(0x58000000u | (((offs16) & 0xFFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_BNE(rj, rd, offs16)	(0x5C000000u | (((offs16) & 0xFFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))
#define LA_JIRL(rd, rj, offs16)	(0x4C000000u | (((offs16) & 0xFFFFu) << 10) | (((rj) & 0x1Fu) << 5) | ((rd) & 0x1Fu))

void loongarch_ejtag_set_instr(struct loongarch_ejtag *ej, uint32_t instr);
void loongarch_ejtag_set_instr_force(struct loongarch_ejtag *ej, uint32_t instr);
int loongarch_ejtag_drscan_32(struct loongarch_ejtag *ej, uint32_t *data);
void loongarch_ejtag_drscan_32_out(struct loongarch_ejtag *ej, uint32_t data);
int loongarch_ejtag_drscan_64(struct loongarch_ejtag *ej, uint64_t *data);
void loongarch_ejtag_drscan_64_out(struct loongarch_ejtag *ej, uint64_t data);
int loongarch_ejtag_read_pcsample(struct loongarch_ejtag *ej, uint64_t *pc);

int loongarch_ejtag_scan_all(struct loongarch_ejtag *ej,
	struct loongarch_ejtag_all *all);
int loongarch_ejtag_scan_fastdata(struct loongarch_ejtag *ej,
	struct loongarch_ejtag_fastdata *fastdata);

int loongarch_ejtag_get_impcode(struct loongarch_ejtag *ej);
int loongarch_ejtag_get_idcode(struct loongarch_ejtag *ej);
int loongarch_ejtag_enter_debug(struct loongarch_ejtag *ej);
int loongarch_ejtag_exit_debug(struct loongarch_ejtag *ej);
void loongarch_ejtag_set_probe_vector(struct loongarch_ejtag *ej, bool dmseg);
int loongarch_ejtag_init(struct loongarch_ejtag *ej);

#endif /* OPENOCD_TARGET_LOONGARCH_EJTAG_H */
