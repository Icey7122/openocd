/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 * LoongArch processor-access (PrAcc) engine.
 *
 * The CPU in debug mode fetches instructions from dmseg.  Every fetch is
 * intercepted by the probe as a PrAcc read, and the host feeds the aligned
 * pair of 32-bit instructions through the 64-bit DATA register.
 * Stores made by the module to dmseg appear as PrAcc writes that the host
 * services by reading the DATA register.  This mirrors OpenOCD's
 * mips32_pracc scheme and the official Loongson tool's
 * ExecuteDebugModule loop.
 ***************************************************************************/

#ifndef OPENOCD_TARGET_LOONGARCH_PRACC_H
#define OPENOCD_TARGET_LOONGARCH_PRACC_H

#include "loongarch_ejtag.h"
#include "loongarch_modules.h"

/*
 * Execute a LoongArch debug module.
 *
 * The CPU must already be in debug mode (loongarch_ejtag_enter_debug).
 * Module stores to mailbox slots are collected into @out_buf; module loads
 * are supplied from @in_buf.
 *
 * Completion: the module is expected to loop back to the TEXT entry
 * (typical `b 1b` tail).  If @exit_debug is set, a PrAcc timeout after
 * all code has been fed is treated as success (the module executed
 * LA_DERET and left debug mode).
 */
int loongarch_pracc_exec(struct loongarch_ejtag *ej,
	unsigned int code_len, const uint32_t *code,
	uint64_t *out_buf, unsigned int out_entries, unsigned int *out_count,
	unsigned int *out_events,
		/* module stores */
	const uint64_t *in_buf, unsigned int in_entries,	/* module loads */
	bool exit_debug);

/* Register dump / restore using the extracted official modules. */
int loongarch_pracc_read_regs(struct loongarch_ejtag *ej,
	uint64_t *regs, unsigned int reg_count);
int loongarch_pracc_write_regs(struct loongarch_ejtag *ej,
	const uint64_t *regs, unsigned int reg_count);

int loongarch_pracc_read_csr(struct loongarch_ejtag *ej,
	uint32_t csr, uint64_t *value);
int loongarch_pracc_write_csr(struct loongarch_ejtag *ej,
	uint32_t csr, uint64_t value);
int loongarch_pracc_read_cpucfg(struct loongarch_ejtag *ej,
	uint32_t index, uint64_t *value);
int loongarch_pracc_read_iocsr(struct loongarch_ejtag *ej,
	uint64_t address, unsigned int size, uint64_t *value);
int loongarch_pracc_write_iocsr(struct loongarch_ejtag *ej,
	uint64_t address, unsigned int size, uint64_t value);
int loongarch_pracc_read_fpr(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t *value);
int loongarch_pracc_write_fpr(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t value);
int loongarch_pracc_read_fcsr(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t *value);
int loongarch_pracc_write_fcsr(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t value);
int loongarch_pracc_read_fcc(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t *value);
int loongarch_pracc_write_fcc(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t value);

/* Block memory access through synthesized LoongArch modules.
 * @words 64-bit words, <= LA_PRACC_MAX_WORDS per call. */
int loongarch_pracc_read_mem(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int words, uint64_t *out);
int loongarch_pracc_write_mem(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int words, const uint64_t *in);
int loongarch_pracc_read_mem_width(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int size, unsigned int count, uint64_t *out);
int loongarch_pracc_write_mem_width(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int size, unsigned int count, const uint64_t *in);

/* Experimental IR=FASTDATA transfer.  A temporary handler is installed at
 * @handler_addr and executed through dmseg.  The handler, save area, and
 * scratch registers are restored before this function returns. */
int loongarch_pracc_fastdata_read64(struct loongarch_ejtag *ej,
	uint64_t handler_addr, uint64_t addr, unsigned int count, uint64_t *out);
int loongarch_pracc_fastdata_write64(struct loongarch_ejtag *ej,
	uint64_t handler_addr, uint64_t addr, unsigned int count,
	const uint64_t *in);
int loongarch_pracc_fastdata_finish(struct loongarch_ejtag *ej);

#endif /* OPENOCD_TARGET_LOONGARCH_PRACC_H */
