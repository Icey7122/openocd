/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 * LoongArch processor-access (PrAcc) engine.
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <inttypes.h>

#include <helper/time_support.h>
#include <target/target.h>

#include "loongarch_pracc.h"

/* The ADDRESS register may present a dmseg access either as the full
 * 64-bit address (0xDB000000000FFFxx) or as the low 20-bit offset
 * (0xFFExx).  Normalize both to the slot offset below the mailbox base
 * and return true when the access hits one of the mailbox slots. */
static bool loongarch_mailbox_slot(uint64_t addr, uint64_t *slot)
{
	uint64_t rel = addr & 0xFFFFF;

	if (rel >= 0xFFE00)
		rel -= 0xFFE00;
	else if (addr <= 0x1F8)
		rel = addr;
	else
		return false;

	/* A 64-bit dmseg access can be reported as either its first byte or
	 * its final byte.  The 2K300 reports loads from 0x1e0/0x1e8 as
	 * 0x1e7/0x1ef, so classify the containing mailbox word. */
	rel &= ~0x7ull;
	if (rel >= 0x1E0 && rel <= 0x1F8) {
		*slot = rel;
		return true;
	}
	return false;
}

/* The 2K300 fetches one 32-bit instruction from a 64-bit DATA register.
 * ADDRESS bit 2 selects the low or high half, therefore both fetches in an
 * aligned instruction pair must receive the same DATA value. */
static bool loongarch_pracc_instruction_pair(const uint32_t *code,
	unsigned int code_len, uint64_t addr, uint64_t *data,
	unsigned int *pair_index)
{
	uint64_t segment = addr & ~(LA_DMSEG_SIZE - 1);
	uint64_t offset = addr & (LA_DMSEG_SIZE - 1);

	if ((segment != 0 && segment != LA_DMSEG_BASE_ADDR) ||
		(offset & 3) != 0)
		return false;

	uint64_t requested_index = offset / sizeof(uint32_t);
	if (requested_index >= code_len)
		return false;

	unsigned int index = (unsigned int)requested_index & ~1u;
	uint32_t high = index + 1 < code_len ? code[index + 1] : LA_ANDI_ZERO;

	*data = code[index] | ((uint64_t)high << 32);
	*pair_index = index;
	return true;
}

static int loongarch_pracc_capture_access(struct loongarch_ejtag *ej,
	uint32_t *ctrl, uint64_t *data, uint64_t *addr)
{
	if (ej->use_all) {
		struct loongarch_ejtag_all all = {
			.control = ej->ejtag_ctrl,
			.data = 0,
			.address = 0,
		};
		int retval = loongarch_ejtag_scan_all(ej, &all);

		if (retval != ERROR_OK)
			return retval;
		*ctrl = all.control;
		*data = all.data;
		*addr = all.address;
		return ERROR_OK;
	}

	/* The known-good fallback selects CONTROL for every poll.  Do not rely
	 * on OpenOCD's cached IR value across a completed PrAcc. */
	loongarch_ejtag_set_instr_force(ej, ej->ir_control);
	int retval = loongarch_ejtag_drscan_32(ej, ctrl);
	if (retval != ERROR_OK || !(*ctrl & LA_CTRL_PRACC))
		return retval;

	loongarch_ejtag_set_instr(ej, ej->ir_address);
	retval = loongarch_ejtag_drscan_64(ej, addr);
	if (retval != ERROR_OK)
		return retval;

	/* A CPU store exposes its value in DATA.  For a CPU load, the caller
	 * supplies DATA after classifying the address. */
	if (*ctrl & LA_CTRL_PRNW) {
		loongarch_ejtag_set_instr(ej, ej->ir_data);
		retval = loongarch_ejtag_drscan_64(ej, data);
	}
	return retval;
}

/* Wait for the CPU to post the next processor access (PrAcc set) or to
 * leave debug mode (BRKST clear).  IR=ALL captures CONTROL, DATA and ADDRESS
 * in one official-style transaction; the original separate scans remain as
 * a runtime-selectable fallback. */
static int loongarch_pracc_wait_access(struct loongarch_ejtag *ej,
	uint32_t *ctrl_out, uint64_t *data_out, uint64_t *addr_out,
	bool *left_debug)
{
	int64_t then = timeval_ms();
	int retval;
	uint64_t data = 0;
	uint64_t addr = 0;

	while (1) {
		uint32_t ctrl = ej->ejtag_ctrl;
		data = 0;
		addr = 0;

		retval = loongarch_pracc_capture_access(ej, &ctrl, &data, &addr);
		if (retval != ERROR_OK)
			return retval;

		*left_debug = !(ctrl & LA_CTRL_BRKST);
		*ctrl_out = ctrl;
		if (*left_debug)
			break;

		if (ctrl & LA_CTRL_PRACC)
			break;

		if (timeval_ms() - then > 2000) {
			LOG_DEBUG("loongarch pracc: no access pending (ctrl=0x%08"
				  PRIx32 ")", ctrl);
			return ERROR_TARGET_TIMEOUT;
		}
		keep_alive();
	}

	*addr_out = addr;
	*data_out = data;
	LOG_DEBUG("loongarch pracc poll: ctrl=0x%08" PRIx32
		  " data=0x%016" PRIx64 " addr=0x%016" PRIx64,
		  *ctrl_out, *data_out, *addr_out);
	return ERROR_OK;
}

/* Queue the official output-only PrAcc=0 completion (CONTROL 0xc000). */
static uint32_t loongarch_pracc_release_control(struct loongarch_ejtag *ej)
{
	return LA_CTRL_PROBEN |
		(ej->ejtag_ctrl & LA_CTRL_PROBEVEC);
}

static void loongarch_pracc_queue_release(struct loongarch_ejtag *ej)
{
	uint32_t ctrl = loongarch_pracc_release_control(ej);

	/* The official gdbproxy emits this CONTROL word as output-only.  Keeping
	 * the capture buffer out of the transaction is significant for the
	 * 2K300 debug window: it distinguishes a release from a readback poll. */
	LOG_DEBUG("loongarch pracc: release control=0x%08" PRIx32, ctrl);
	loongarch_ejtag_set_instr(ej, ej->ir_control);
	loongarch_ejtag_drscan_32_out(ej, ctrl);
}

/* Complete the pending access by writing PrAcc=0 (CONTROL 0xc000). */
static int loongarch_pracc_release(struct loongarch_ejtag *ej)
{
	loongarch_pracc_queue_release(ej);
	return jtag_execute_queue();
}

static int loongarch_pracc_feed_instruction(struct loongarch_ejtag *ej,
	uint64_t addr, uint64_t instruction_data, unsigned int pair_index)
{
	LOG_DEBUG("loongarch pracc: feed[%u:%u] addr=0x%016" PRIx64
		  " data=0x%016" PRIx64, pair_index, pair_index + 1, addr,
		  instruction_data);
	loongarch_ejtag_set_instr(ej, ej->ir_data);
	loongarch_ejtag_drscan_64_out(ej, instruction_data);
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	return loongarch_pracc_release(ej);
}

static int loongarch_pracc_error(struct loongarch_ejtag *ej, int retval)
{
	(void)ej;
	LOG_ERROR("loongarch: PRACC failed (%d)", retval);
	return retval;
}

/* Service a pending dmseg mailbox access: PRnW=1 (CPU store) uses the DATA
 * captured after reading ADDRESS; PRnW=0 (CPU load) writes DATA from in_buf.
 * The FIFO slot streams values in access order; other slots are indexed by
 * (slot / 8). */
static int loongarch_pracc_service_mailbox(struct loongarch_ejtag *ej,
	uint32_t ctrl, uint64_t slot, uint64_t captured_data,
	uint64_t *out_buf, unsigned int out_entries, unsigned int *out_seq,
	unsigned int *out_events,
	const uint64_t *in_buf, unsigned int in_entries, unsigned int *in_seq)
{
	uint64_t data = 0;
	bool cpu_store = (ctrl & LA_CTRL_PRNW) != 0;
	bool completed = false;

	if (!cpu_store) {
		if (slot == LA_PRACC_SLOT_FIFO) {
			data = (in_buf && *in_seq < in_entries) ?
				in_buf[*in_seq] : 0;
		} else {
			uint64_t idx = slot / 8;
			data = (in_buf && idx < in_entries) ? in_buf[idx] : 0;
		}
	}

	/* FASTDATA combines DATA transfer and PrAcc completion.  Capture the
	 * incoming SPrAcc bit so an unsupported address can safely fall back to
	 * the ordinary DATA + CONTROL path without losing the pending access. */
	if (ej->use_fastdata) {
		struct loongarch_ejtag_fastdata fastdata = {
			.spracc = false,
			.data = data,
		};
		int retval = loongarch_ejtag_scan_fastdata(ej, &fastdata);
		if (retval != ERROR_OK)
			return retval;
		if (fastdata.spracc) {
			completed = true;
			if (cpu_store)
				data = fastdata.data;
			else if (slot == LA_PRACC_SLOT_FIFO)
				(*in_seq)++;
			LOG_DEBUG("loongarch pracc: FASTDATA slot=0x%03" PRIx64
				" direction=%s data=0x%016" PRIx64,
				slot, cpu_store ? "target-to-host" : "host-to-target",
				data);
		} else {
			LOG_DEBUG("loongarch pracc: FASTDATA unavailable for slot=0x%03"
				PRIx64 "; using ordinary completion", slot);
		}
	}

	if (cpu_store) {
		if (!completed)
			data = captured_data;
		if (out_events)
			(*out_events)++;
		if (slot == LA_PRACC_SLOT_FIFO) {
			if (out_buf && *out_seq < out_entries)
				out_buf[(*out_seq)++] = data;
		} else {
			uint64_t idx = slot / 8;
			if (out_buf && idx < out_entries)
				out_buf[idx] = data;
		}
		LOG_DEBUG("loongarch pracc: store slot=0x%03" PRIx64
			 " data=0x%016" PRIx64, slot, data);
	} else if (!completed) {
		if (slot == LA_PRACC_SLOT_FIFO)
			(*in_seq)++;
		loongarch_ejtag_set_instr(ej, ej->ir_data);
		int retval = loongarch_ejtag_drscan_64(ej, &data);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("loongarch pracc: load slot=0x%03" PRIx64
			 " data=0x%016" PRIx64, slot, data);
	}
	return completed ? ERROR_OK : loongarch_pracc_release(ej);
}

static int loongarch_pracc_exec_raw(struct loongarch_ejtag *ej,
	unsigned int code_len, const uint32_t *code,
	uint64_t *out_buf, unsigned int out_entries, unsigned int *out_count,
	unsigned int *out_events,
	const uint64_t *in_buf, unsigned int in_entries,
	bool exit_debug)
{
	unsigned int out_seq = 0;
	unsigned int store_events = 0;
	unsigned int in_seq = 0;
	bool tail_pair_seen = false;

	if (!code || code_len == 0)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	if (out_count)
		*out_count = 0;
	if (out_events)
		*out_events = 0;

	while (1) {
		bool left_debug = false;
		uint32_t ctrl = 0;
		uint64_t data = 0;
		uint64_t addr = 0;
		int retval = loongarch_pracc_wait_access(ej, &ctrl, &data, &addr,
			&left_debug);
		if (retval != ERROR_OK)
			return loongarch_pracc_error(ej, retval);

		if (left_debug) {
			/* CPU executed DERET and left debug mode; on LS2K0300 it
			 * fetches one more word from dmseg afterwards - release
			 * that residual access too (the LS2K300 extra-fetch behavior). */
			retval = loongarch_pracc_release(ej);
			if (retval != ERROR_OK)
				return loongarch_pracc_error(ej, retval);
			return exit_debug ? ERROR_OK : ERROR_TARGET_TIMEOUT;
		}

		uint64_t slot = 0;
		if ((ctrl & LA_CTRL_PRACC) && loongarch_mailbox_slot(addr, &slot)) {
			retval = loongarch_pracc_service_mailbox(ej, ctrl, slot, data,
				out_buf, out_entries, &out_seq,
				&store_events,
				in_buf, in_entries, &in_seq);
			if (retval != ERROR_OK)
				return loongarch_pracc_error(ej, retval);
			continue;
		}

		/* After ERTN has been supplied, release any residual dmseg fetches
		 * without replacing it with another instruction. */
		if (exit_debug && tail_pair_seen) {
			/* DERET already fed; the CPU is leaving debug mode but on
			 * 2K300 it posts one or more residual fetches first.
			 * Complete them with 0xc000 only, never feeding a word
			 * (feeding garbage here is what kept the system dead
			 * after resume). */
			retval = loongarch_pracc_release(ej);
			if (retval != ERROR_OK)
				return loongarch_pracc_error(ej, retval);
			continue;
		}

		uint64_t instruction_data = 0;
		unsigned int pair_index = 0;
		if (!loongarch_pracc_instruction_pair(code, code_len, addr,
				&instruction_data, &pair_index)) {
			LOG_ERROR("loongarch pracc: unexpected instruction fetch "
				"addr=0x%016" PRIx64 " code_words=%u", addr, code_len);
			return loongarch_pracc_error(ej, ERROR_FAIL);
		}

		/* Every normal module ends by branching to word zero.  Leave that
		 * access pending so the next operation can replace the module without
		 * an unnecessary debug exit/re-entry cycle. */
		if (!exit_debug && tail_pair_seen && pair_index == 0) {
			if (out_count)
				*out_count = out_seq;
			if (out_events)
				*out_events = store_events;
			return ERROR_OK;
		}

		/* Official feed_acc() uses Data0(code, NULL), i.e. an output-only
		 * 64-bit DATA scan.  A capture-capable scan is a different transport
		 * operation and must not be substituted here. */
		retval = loongarch_pracc_feed_instruction(ej, addr,
			instruction_data, pair_index);
		if (retval != ERROR_OK)
			return loongarch_pracc_error(ej, retval);

		/* Normal modules end in branch-to-entry followed by padding.  LoongArch
		 * has no branch delay slot, so the padding word is normally never fetched;
		 * seeing the pair that contains the branch is the completion watermark. */
		unsigned int tail_pair_index = code_len > 1 ?
			(code_len - 2) & ~1u : 0;
		if (pair_index == tail_pair_index)
			tail_pair_seen = true;
	}
}

int loongarch_pracc_exec(struct loongarch_ejtag *ej,
	unsigned int code_len, const uint32_t *code,
	uint64_t *out_buf, unsigned int out_entries, unsigned int *out_count,
	unsigned int *out_events,
	const uint64_t *in_buf, unsigned int in_entries,
	bool exit_debug)
{
	int retval = loongarch_pracc_fastdata_finish(ej);
	if (retval != ERROR_OK)
		return retval;
	return loongarch_pracc_exec_raw(ej, code_len, code,
		out_buf, out_entries, out_count, out_events,
		in_buf, in_entries, exit_debug);
}

int loongarch_pracc_read_regs(struct loongarch_ejtag *ej,
	uint64_t *regs, unsigned int reg_count)
{
	uint64_t mailbox[0x80] = { 0 };
	unsigned int count = reg_count < 33 ? reg_count : 33;
	unsigned int out_count = 0;

	/* The extracted official module emits r0..r31 and CSR 0x501 (PC)
	 * through FIFO, while replaying r2/r3 from the two spill slots. */
	int retval = loongarch_pracc_exec(ej,
		ARRAY_SIZE(loongarch_read_registers_code),
		loongarch_read_registers_code,
		mailbox, ARRAY_SIZE(mailbox),
		&out_count, NULL,
		mailbox, ARRAY_SIZE(mailbox), false);
	if (retval != ERROR_OK)
		return retval;
	if (out_count < count) {
		loongarch_pracc_error(ej, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for (unsigned int i = 0; i < count; i++)
		regs[i] = mailbox[i];
	return ERROR_OK;
}

int loongarch_pracc_write_regs(struct loongarch_ejtag *ej,
	const uint64_t *regs, unsigned int reg_count)
{
	uint64_t mailbox[0x80] = { 0 };

	if (reg_count < 33)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	/* The official write module consumes two copies of r1, then r2..r31,
	 * followed by the PC.  It also reloads r31 from the 0x1e8 spill slot
	 * after writing CSR 0x501, so seed that indexed mailbox entry too. */
	mailbox[0] = regs[1];
	mailbox[1] = regs[1];
	for (unsigned int i = 2; i <= 31; i++)
		mailbox[i] = regs[i];
	mailbox[32] = regs[32];
	mailbox[LA_PRACC_SLOT_SAVE / 8] = regs[31];

	return loongarch_pracc_exec(ej,
		ARRAY_SIZE(loongarch_write_registers_code),
		loongarch_write_registers_code,
		mailbox, ARRAY_SIZE(mailbox),
		NULL, NULL,
		mailbox, ARRAY_SIZE(mailbox), false);
}

static unsigned int loongarch_csr_module_build(uint32_t *code,
	uint32_t csr, bool write_t)
{
	unsigned int n = 0;

	/* Preserve r15 in DEBUG.SAVE, use it as the mailbox base, and preserve
	 * r1 in the indexed save slot while the CSR operation runs. */
	code[n++] = LA_CSRWR(15, LA_CSR_DEBUG_SAVE);
	code[n++] = LA_LU12I_W(15, 0xFF);
	code[n++] = LA_ORI(15, 15, 0xE00);
	code[n++] = LA_LU32I_D(15, 0);
	code[n++] = LA_LU52I_D(15, 15, 0xDB0);
	code[n++] = LA_ST_D(1, 15, LA_PRACC_SLOT_SAVE);

	if (write_t)
		code[n++] = LA_LD_D(1, 15, LA_PRACC_SLOT_ADDR);
	else
		code[n++] = LA_CSRRD(1, csr);

	if (write_t)
		code[n++] = LA_CSRWR(1, csr);
	else
		code[n++] = LA_ST_D(1, 15, LA_PRACC_SLOT_ADDR);

	code[n++] = LA_LD_D(1, 15, LA_PRACC_SLOT_SAVE);
	code[n++] = LA_CSRRD(15, LA_CSR_DEBUG_SAVE);
	code[n++] = LA_DBAR;
	unsigned int branch_offset = n;
	code[n++] = LA_B(-(int)branch_offset);
	code[n++] = LA_ANDI_ZERO;
	return n;
}

static int loongarch_pracc_csr_rw(struct loongarch_ejtag *ej,
	uint32_t csr, uint64_t *value, bool write_t)
{
	uint32_t code[32];
	uint64_t mailbox[0x80] = { 0 };
	unsigned int code_len;
	unsigned int out_events = 0;

	if (!value || csr > 0x3FFFu)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	mailbox[LA_PRACC_SLOT_ADDR / 8] = *value;
	code_len = loongarch_csr_module_build(code, csr, write_t);
	int retval = loongarch_pracc_exec(ej, code_len, code,
		mailbox, ARRAY_SIZE(mailbox),
		NULL, &out_events,
		mailbox, ARRAY_SIZE(mailbox), false);
	if (retval != ERROR_OK)
		return retval;
	if (!write_t && out_events == 0) {
		loongarch_pracc_error(ej, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (!write_t)
		*value = mailbox[LA_PRACC_SLOT_ADDR / 8];
	return ERROR_OK;
}

int loongarch_pracc_read_csr(struct loongarch_ejtag *ej,
	uint32_t csr, uint64_t *value)
{
	return loongarch_pracc_csr_rw(ej, csr, value, false);
}

int loongarch_pracc_write_csr(struct loongarch_ejtag *ej,
	uint32_t csr, uint64_t value)
{
	return loongarch_pracc_csr_rw(ej, csr, &value, true);
}

int loongarch_pracc_read_cpucfg(struct loongarch_ejtag *ej,
	uint32_t index, uint64_t *value)
{
	uint32_t code[16];
	uint64_t mailbox[0x80] = { 0 };
	unsigned int out_events = 0;
	unsigned int n = 0;

	if (!value)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	mailbox[LA_PRACC_SLOT_ADDR / 8] = index;
	code[n++] = LA_CSRWR(15, LA_CSR_DEBUG_SAVE);
	code[n++] = LA_LU12I_W(15, 0xFF);
	code[n++] = LA_ORI(15, 15, 0xE00);
	code[n++] = LA_LU32I_D(15, 0);
	code[n++] = LA_LU52I_D(15, 15, 0xDB0);
	code[n++] = LA_ST_D(1, 15, LA_PRACC_SLOT_SAVE);
	code[n++] = LA_ST_D(2, 15, LA_PRACC_SLOT_SAVE2);
	code[n++] = LA_LD_D(1, 15, LA_PRACC_SLOT_ADDR);
	code[n++] = LA_CPUCFG(2, 1);
	code[n++] = LA_ST_D(2, 15, LA_PRACC_SLOT_ADDR);
	code[n++] = LA_LD_D(1, 15, LA_PRACC_SLOT_SAVE);
	code[n++] = LA_LD_D(2, 15, LA_PRACC_SLOT_SAVE2);
	code[n++] = LA_CSRRD(15, LA_CSR_DEBUG_SAVE);
	code[n++] = LA_DBAR;
	unsigned int branch_offset = n;
	code[n++] = LA_B(-(int)branch_offset);
	code[n++] = LA_ANDI_ZERO;

	int retval = loongarch_pracc_exec(ej, n, code,
		mailbox, ARRAY_SIZE(mailbox), NULL, &out_events,
		mailbox, ARRAY_SIZE(mailbox), false);
	if (retval != ERROR_OK)
		return retval;
	if (out_events == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	*value = mailbox[LA_PRACC_SLOT_ADDR / 8];
	return ERROR_OK;
}

static int loongarch_iocsr_opcode(unsigned int size, bool write_t,
	uint32_t *opcode)
{
	uint32_t base;

	if (!opcode)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	switch (size) {
	case 1:
		base = write_t ? 0x06481000u : 0x06480000u;
		break;
	case 2:
		base = write_t ? 0x06481400u : 0x06480400u;
		break;
	case 4:
		base = write_t ? 0x06481800u : 0x06480800u;
		break;
	case 8:
		base = write_t ? 0x06481C00u : 0x06480C00u;
		break;
	default:
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	*opcode = LA_IOCSR_2R(base, 2, 1);
	return ERROR_OK;
}

static int loongarch_pracc_iocsr_rw(struct loongarch_ejtag *ej,
	uint64_t address, unsigned int size, uint64_t *value, bool write_t)
{
	uint32_t code[20];
	uint64_t mailbox[0x80] = { 0 };
	uint32_t access_instruction;
	unsigned int n = 0;
	int retval;

	if (!value)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	retval = loongarch_iocsr_opcode(size, write_t, &access_instruction);
	if (retval != ERROR_OK)
		return retval;

	mailbox[LA_PRACC_SLOT_ADDR / 8] = address;
	if (write_t)
		mailbox[0] = *value;
	code[n++] = LA_CSRWR(15, LA_CSR_DEBUG_SAVE);
	code[n++] = LA_LU12I_W(15, 0xFF);
	code[n++] = LA_ORI(15, 15, 0xE00);
	code[n++] = LA_LU32I_D(15, 0);
	code[n++] = LA_LU52I_D(15, 15, 0xDB0);
	code[n++] = LA_ST_D(1, 15, LA_PRACC_SLOT_SAVE);
	code[n++] = LA_ST_D(2, 15, LA_PRACC_SLOT_SAVE2);
	code[n++] = LA_LD_D(1, 15, LA_PRACC_SLOT_ADDR);
	if (write_t)
		code[n++] = LA_LD_D(2, 15, LA_PRACC_SLOT_FIFO);
	code[n++] = access_instruction;
	if (!write_t)
		code[n++] = LA_ST_D(2, 15, LA_PRACC_SLOT_ADDR);
	code[n++] = LA_LD_D(1, 15, LA_PRACC_SLOT_SAVE);
	code[n++] = LA_LD_D(2, 15, LA_PRACC_SLOT_SAVE2);
	code[n++] = LA_CSRRD(15, LA_CSR_DEBUG_SAVE);
	code[n++] = LA_DBAR;
	unsigned int branch_offset = n;
	code[n++] = LA_B(-(int)branch_offset);
	code[n++] = LA_ANDI_ZERO;

	retval = loongarch_pracc_exec(ej, n, code,
		mailbox, ARRAY_SIZE(mailbox), NULL, NULL,
		mailbox, ARRAY_SIZE(mailbox), false);
	if (retval != ERROR_OK)
		return retval;

	if (!write_t)
		*value = mailbox[LA_PRACC_SLOT_ADDR / 8];
	return ERROR_OK;
}

int loongarch_pracc_read_iocsr(struct loongarch_ejtag *ej,
	uint64_t address, unsigned int size, uint64_t *value)
{
	return loongarch_pracc_iocsr_rw(ej, address, size, value, false);
}

int loongarch_pracc_write_iocsr(struct loongarch_ejtag *ej,
	uint64_t address, unsigned int size, uint64_t value)
{
	return loongarch_pracc_iocsr_rw(ej, address, size, &value, true);
}

static int loongarch_pracc_fpu_exec(struct loongarch_ejtag *ej,
	const uint32_t *code, unsigned int code_len, uint64_t *value,
	bool write_t)
{
	uint64_t mailbox[0x80] = { 0 };
	unsigned int out_events = 0;

	if (!value)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	mailbox[LA_PRACC_SLOT_ADDR / 8] = *value;
	int retval = loongarch_pracc_exec(ej, code_len, code,
		mailbox, ARRAY_SIZE(mailbox), NULL, &out_events,
		mailbox, ARRAY_SIZE(mailbox), false);
	if (retval != ERROR_OK)
		return retval;
	if (!write_t && out_events == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (!write_t)
		*value = mailbox[LA_PRACC_SLOT_ADDR / 8];
	return ERROR_OK;
}

static unsigned int loongarch_fpr_module_build(uint32_t *code,
	unsigned int reg, bool write_t)
{
	static const uint32_t template[] = {
		0x0414082F, 0x14001FEF, 0x03B801EF, 0x0336C1EF,
		0, 0x0414080F, 0x38720000, 0x53FFE7FF, 0x03400000,
	};

	memcpy(code, template, sizeof(template));
	/* fld.d/fst.d fN, r15, 0x1e0: exchange the FPR through mailbox 0x1e0. */
	code[4] = (write_t ? 0x2B8781E0u : 0x2BC781E0u) | reg;
	return ARRAY_SIZE(template);
}

static unsigned int loongarch_fcsr_module_build(uint32_t *code,
	unsigned int reg, bool write_t)
{
	static const uint32_t read_template[] = {
		0x0414082F, 0x14001FEF, 0x03B801EF, 0x0336C1EF,
		0x29C7A1E2, 0, 0x29C781E2, 0x28C7A1E2,
		0x0414080F, 0x38720000, 0x53FFDBFF, 0x03400000,
	};
	static const uint32_t write_template[] = {
		0x0414082F, 0x14001FEF, 0x03B801EF, 0x0336C1EF,
		0x29C7A1E2, 0x28C781E2, 0, 0x28C7A1E2,
		0x0414080F, 0x38720000, 0x53FFDBFF, 0x03400000,
	};
	const uint32_t *template = write_t ? write_template : read_template;

	memcpy(code, template, sizeof(read_template));
	/* movgr2fcsr encodes FCSR in bits 0..4 and r2 in bits 5..9;
	 * movfcsr2gr encodes r2 in bits 0..4 and FCSR in bits 5..9.
	 * The official client uses 0x0114c002 for writes, which selects r0 and
	 * FCSR2 and therefore always writes zero. */
	code[write_t ? 6 : 5] = write_t ?
		(0x0114C040u | reg) : (0x0114C802u | (reg << 5));
	return ARRAY_SIZE(read_template);
}

static unsigned int loongarch_fcc_module_build(uint32_t *code,
	unsigned int reg, bool write_t)
{
	static const uint32_t read_template[] = {
		0x0414082F, 0x14001FEF, 0x03B801EF, 0x0336C1EF,
		0x29C7A1E2, 0, 0x29C781E2, 0x28C7A1E2,
		0x0414080F, 0x38720000, 0x53FFDBFF, 0x03400000,
	};
	static const uint32_t write_template[] = {
		0x0414082F, 0x14001FEF, 0x03B801EF, 0x0336C1EF,
		0x29C7A1E2, 0x28C781E2, 0, 0x28C7A1E2,
		0x0414080F, 0x38720000, 0x53FFDBFF, 0x03400000,
	};
	const uint32_t *template = write_t ? write_template : read_template;

	memcpy(code, template, sizeof(read_template));
	code[write_t ? 6 : 5] = write_t ?
		(0x0114D840u | reg) : (0x0114DC02u | (reg << 5));
	return ARRAY_SIZE(read_template);
}

int loongarch_pracc_read_fpr(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t *value)
{
	uint32_t code[12];

	if (reg >= 32)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	unsigned int code_len = loongarch_fpr_module_build(code, reg, false);
	return loongarch_pracc_fpu_exec(ej, code, code_len, value, false);
}

int loongarch_pracc_write_fpr(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t value)
{
	uint32_t code[12];

	if (reg >= 32)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	unsigned int code_len = loongarch_fpr_module_build(code, reg, true);
	return loongarch_pracc_fpu_exec(ej, code, code_len, &value, true);
}

int loongarch_pracc_read_fcsr(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t *value)
{
	uint32_t code[12];

	if (reg >= 4)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	unsigned int code_len = loongarch_fcsr_module_build(code, reg, false);
	return loongarch_pracc_fpu_exec(ej, code, code_len, value, false);
}

int loongarch_pracc_write_fcsr(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t value)
{
	uint32_t code[12];

	if (reg >= 4)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	unsigned int code_len = loongarch_fcsr_module_build(code, reg, true);
	return loongarch_pracc_fpu_exec(ej, code, code_len, &value, true);
}

int loongarch_pracc_read_fcc(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t *value)
{
	uint32_t code[12];

	if (reg >= 8)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	unsigned int code_len = loongarch_fcc_module_build(code, reg, false);
	return loongarch_pracc_fpu_exec(ej, code, code_len, value, false);
}

int loongarch_pracc_write_fcc(struct loongarch_ejtag *ej,
	unsigned int reg, uint64_t value)
{
	uint32_t code[12];

	if (reg >= 8 || value > 1)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	unsigned int code_len = loongarch_fcc_module_build(code, reg, true);
	return loongarch_pracc_fpu_exec(ej, code, code_len, &value, true);
}

/*
 * Memory access module.  The target address is embedded directly in the
 * instruction stream (lu12i/ori/lu32i/lu52i); data streams through the
 * FIFO slot 0x1F0 in access order.  Scratch r1/r2/r3/r15 preserved
 * (r1/r15 via CSR 0x502, r2/r3 via 0x1E8/0x1F8 spills).
 */
static uint32_t loongarch_mem_load(unsigned int size, unsigned int rd,
	unsigned int rj, unsigned int imm12)
{
	switch (size) {
	case 1:
		return LA_LD_B(rd, rj, imm12);
	case 2:
		return LA_LD_H(rd, rj, imm12);
	case 4:
		return LA_LD_W(rd, rj, imm12);
	case 8:
		return LA_LD_D(rd, rj, imm12);
	default:
		return 0;
	}
}

static uint32_t loongarch_mem_store(unsigned int size, unsigned int rd,
	unsigned int rj, unsigned int imm12)
{
	switch (size) {
	case 1:
		return LA_ST_B(rd, rj, imm12);
	case 2:
		return LA_ST_H(rd, rj, imm12);
	case 4:
		return LA_ST_W(rd, rj, imm12);
	case 8:
		return LA_ST_D(rd, rj, imm12);
	default:
		return 0;
	}
}

static unsigned int loongarch_mem_module_build(uint32_t *code,
	uint64_t addr, unsigned int size, unsigned int words, bool write_t)
{
	unsigned int n = 0;

	/* r15 = mailbox base; spill r2/r3 through it */
	code[n++] = LA_CSRWR(15, LA_CSR_DEBUG_SAVE);
	code[n++] = LA_LU12I_W(15, 0xFF);
	code[n++] = LA_ORI(15, 15, 0xE00);
	code[n++] = LA_LU32I_D(15, 0);
	code[n++] = LA_LU52I_D(15, 15, 0xDB0);	/* r15 = mailbox base */
	code[n++] = LA_ST_D(2, 15, LA_PRACC_SLOT_SAVE);
	code[n++] = LA_ST_D(3, 15, LA_PRACC_SLOT_SAVE2);
	code[n++] = LA_CSRRD(15, LA_CSR_DEBUG_SAVE);	/* restore r15 */

	/* r1 = target address */
	code[n++] = LA_CSRWR(1, LA_CSR_DEBUG_SAVE);	/* save r1 */
	code[n++] = LA_LU12I_W(1, (addr >> 12) & 0xFFFFF);
	code[n++] = LA_ORI(1, 1, addr & 0xFFF);
	code[n++] = LA_LU32I_D(1, (addr >> 32) & 0xFFFFF);
	code[n++] = LA_LU52I_D(1, 1, (addr >> 52) & 0xFFF);	/* r1 = addr */

	/* r3 = mailbox base (r3 was spilled above) */
	code[n++] = LA_LU12I_W(3, 0xFF);
	code[n++] = LA_ORI(3, 3, 0xE00);
	code[n++] = LA_LU32I_D(3, 0);
	code[n++] = LA_LU52I_D(3, 3, 0xDB0);

	for (unsigned int i = 0; i < words; i++) {
		if (write_t) {
			code[n++] = LA_LD_D(2, 3, LA_PRACC_SLOT_FIFO);	/* data in */
			code[n++] = loongarch_mem_store(size, 2, 1, i * size);
		} else {
			code[n++] = loongarch_mem_load(size, 2, 1, i * size);
			code[n++] = LA_ST_D(2, 3, LA_PRACC_SLOT_FIFO);	/* data out */
		}
	}

	code[n++] = LA_LD_D(2, 3, LA_PRACC_SLOT_SAVE);	/* restore r2 */
	code[n++] = LA_LD_D(3, 3, LA_PRACC_SLOT_SAVE2);	/* restore r3 */
	code[n++] = LA_CSRRD(1, LA_CSR_DEBUG_SAVE);	/* restore r1 */
	code[n++] = LA_DBAR;
	unsigned int branch_offset = n;
	code[n++] = LA_B(-(int)branch_offset);	/* branch back to the entry */
	code[n++] = LA_ANDI_ZERO;	/* nop padding */
	return n;
}

static int loongarch_pracc_mem_rw(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int size, unsigned int words, uint64_t *data,
	bool write_t)
{
	uint32_t code[64];
	uint64_t mailbox[0x80] = { 0 };
	unsigned int n;
	unsigned int out_count = 0;

	if ((size != 1 && size != 2 && size != 4 && size != 8) ||
		words == 0 || words > LA_PRACC_MAX_WORDS)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	n = loongarch_mem_module_build(code, addr, size, words, write_t);

	if (write_t) {
		for (unsigned int i = 0; i < words; i++)
			mailbox[i] = data[i];
	}

	/* in_buf == out_buf: the module's spill slots are replayed back to
	 * it on restore, keeping r2/r3 intact. */
	int retval = loongarch_pracc_exec(ej, n, code,
		mailbox, ARRAY_SIZE(mailbox),
		&out_count, NULL,
		mailbox, ARRAY_SIZE(mailbox), false);
	if (retval != ERROR_OK)
		return retval;
	if (!write_t && out_count < words) {
		loongarch_pracc_error(ej, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (!write_t) {
		for (unsigned int i = 0; i < words; i++)
			data[i] = mailbox[i];
	}
	return ERROR_OK;
}

int loongarch_pracc_read_mem(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int words, uint64_t *out)
{
	return loongarch_pracc_mem_rw(ej, addr, 8, words, out, false);
}

int loongarch_pracc_write_mem(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int words, const uint64_t *in)
{
	uint64_t data[LA_PRACC_MAX_WORDS];

	for (unsigned int i = 0; i < words; i++)
		data[i] = in[i];
	return loongarch_pracc_mem_rw(ej, addr, 8, words, data, true);
}

int loongarch_pracc_read_mem_width(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int size, unsigned int count, uint64_t *out)
{
	if (!out)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	while (count != 0) {
		unsigned int words = count > LA_PRACC_MAX_WORDS ?
			LA_PRACC_MAX_WORDS : count;
		int retval = loongarch_pracc_mem_rw(ej, addr, size, words, out,
			false);
		if (retval != ERROR_OK)
			return retval;
		addr += (uint64_t)size * words;
		out += words;
		count -= words;
	}
	return ERROR_OK;
}

int loongarch_pracc_write_mem_width(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int size, unsigned int count, const uint64_t *in)
{
	if (!in)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	while (count != 0) {
		unsigned int words = count > LA_PRACC_MAX_WORDS ?
			LA_PRACC_MAX_WORDS : count;
		uint64_t values[LA_PRACC_MAX_WORDS];
		memcpy(values, in, words * sizeof(values[0]));
		int retval = loongarch_pracc_mem_rw(ej, addr, size, words,
			values, true);
		if (retval != ERROR_OK)
			return retval;
		addr += (uint64_t)size * words;
		in += words;
		count -= words;
	}
	return ERROR_OK;
}

#define LA_FASTDATA_HANDLER_WORDS	56u
#define LA_FASTDATA_HANDLER_AREA_WORDS	32u
#define LA_FASTDATA_SAVED_REG_OFFSET	0xE0u
#define LA_FASTDATA_WRITE_ENTRY_WORD	24u

struct loongarch_fastdata_session {
	uint64_t handler_addr;
	uint64_t saved_r21;
	uint64_t backup[LA_FASTDATA_HANDLER_AREA_WORDS];
};

static unsigned int loongarch_emit_address(uint32_t *code,
	unsigned int n, unsigned int reg, uint64_t addr)
{
	code[n++] = LA_LU12I_W(reg, (addr >> 12) & 0xFFFFF);
	code[n++] = LA_ORI(reg, reg, addr & 0xFFF);
	code[n++] = LA_LU32I_D(reg, (addr >> 32) & 0xFFFFF);
	code[n++] = LA_LU52I_D(reg, reg, (addr >> 52) & 0xFFF);
	return n;
}

static void loongarch_fastdata_handler_build(uint32_t *code)
{
	unsigned int n = 0;
	unsigned int read_done_branch;
	unsigned int read_exit_branch;

	/* Read entry.  r21 contains the handler base; preserve every scratch
	 * register in the 32-byte save area immediately following the code. */
	code[n++] = LA_ST_D(1, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x00);
	code[n++] = LA_ST_D(2, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x08);
	code[n++] = LA_ST_D(3, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x10);
	code[n++] = LA_ST_D(4, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x18);
	n = loongarch_emit_address(code, n, 3, LA_FASTDATA_ADDR);
	code[n++] = LA_LD_D(1, 3, 0); /* start address from IR=FASTDATA */
	code[n++] = LA_LD_D(2, 3, 0); /* transfer count */
	/* Preload the first word before posting the marker.  Once the host observes
	 * the marker, the following FASTDATA access is data. */
	code[n++] = LA_LD_D(4, 1, 0);
	code[n++] = LA_ST_D(3, 3, 0);
	unsigned int read_loop = n;
	code[n++] = LA_ADDI_D(2, 2, -1);
	code[n++] = LA_ST_D(4, 3, 0);
	code[n++] = LA_ST_D(2, 3, 0); /* remaining-count acknowledgement */
	code[n++] = LA_ADDI_D(1, 1, 8);
	read_done_branch = n;
	code[n++] = LA_BEQ(2, 0, 0); /* patched to the common exit */
	code[n++] = LA_ANDI_ZERO;
	code[n++] = LA_ANDI_ZERO;
	code[n++] = LA_ANDI_ZERO;
	code[n++] = LA_ANDI_ZERO;
	code[n++] = LA_LD_D(4, 1, 0);
	int read_loop_branch = (int)read_loop - (int)n;
	code[n++] = LA_B(read_loop_branch);
	read_exit_branch = n;
	code[n++] = LA_B(0); /* patched to the common exit */
	code[read_done_branch] = LA_BEQ(2, 0,
		(int)read_exit_branch - (int)read_done_branch);

	/* Write entry.  Keeping both entries in one immutable handler avoids a
	 * stale instruction-cache line selecting the previous transfer mode. */
	assert(n == LA_FASTDATA_WRITE_ENTRY_WORD);
	code[n++] = LA_ST_D(1, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x00);
	code[n++] = LA_ST_D(2, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x08);
	code[n++] = LA_ST_D(3, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x10);
	code[n++] = LA_ST_D(4, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x18);
	n = loongarch_emit_address(code, n, 3, LA_FASTDATA_ADDR);
	code[n++] = LA_LD_D(1, 3, 0); /* start address from IR=FASTDATA */
	code[n++] = LA_LD_D(2, 3, 0); /* transfer count */
	unsigned int write_loop = n;
	code[n++] = LA_LD_D(4, 3, 0);
	code[n++] = LA_ADDI_D(2, 2, -1);
	code[n++] = LA_ST_D(4, 1, 0);
	code[n++] = LA_ADDI_D(1, 1, 8);
	code[n++] = LA_ANDI_ZERO;
	int write_loop_branch = (int)write_loop - (int)n;
	code[n++] = LA_BNE(2, 0, write_loop_branch);
	code[n++] = LA_ANDI_ZERO;
	code[n++] = LA_ANDI_ZERO;
	code[n++] = LA_ANDI_ZERO;
	code[n++] = LA_ANDI_ZERO;

	code[read_exit_branch] = LA_B((int)n - (int)read_exit_branch);
	code[n++] = LA_LD_D(1, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x00);
	code[n++] = LA_LD_D(2, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x08);
	code[n++] = LA_LD_D(3, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x10);
	code[n++] = LA_LD_D(4, 21, LA_FASTDATA_SAVED_REG_OFFSET + 0x18);
	n = loongarch_emit_address(code, n, 21, LA_DMSEG_BASE_ADDR);
	code[n++] = LA_JIRL(0, 21, 0);
	code[n++] = LA_ANDI_ZERO;
	while (n < LA_FASTDATA_HANDLER_WORDS)
		code[n++] = LA_ANDI_ZERO;

	assert(n == LA_FASTDATA_HANDLER_WORDS);
}

static bool loongarch_fastdata_access(uint64_t addr)
{
	uint64_t segment = addr & ~(LA_DMSEG_SIZE - 1);
	uint64_t offset = addr & (LA_DMSEG_SIZE - 1);

	return (segment == 0 || segment == LA_DMSEG_BASE_ADDR) &&
		(offset & ~0x7ull) == (LA_FASTDATA_ADDR & (LA_DMSEG_SIZE - 1));
}

static bool loongarch_fastdata_handler_instruction_pair(uint64_t handler_addr,
	const uint32_t *handler, unsigned int handler_len, uint64_t addr,
	uint64_t *data, unsigned int *pair_index)
{
	uint64_t segment = addr & ~(LA_DMSEG_SIZE - 1);
	uint64_t request_offset = addr & (LA_DMSEG_SIZE - 1);
	uint64_t handler_offset = handler_addr & (LA_DMSEG_SIZE - 1);

	if ((segment != 0 && segment != LA_DMSEG_BASE_ADDR) ||
		request_offset < handler_offset)
		return false;

	uint64_t relative_addr = request_offset - handler_offset;
	return loongarch_pracc_instruction_pair(handler, handler_len,
		relative_addr, data, pair_index);
}

static unsigned int loongarch_fastdata_jump_build(uint32_t *code,
	uint64_t handler_addr, uint64_t entry_addr)
{
	unsigned int n = 0;

	/* Preserve r15 while using it to post the original r21 through the
	 * ordinary mailbox.  The cleanup module restores r21 after the handler
	 * has returned to dmseg. */
	code[n++] = LA_CSRWR(15, LA_CSR_DEBUG_SAVE);
	n = loongarch_emit_address(code, n, 15, LA_PRACC_BASE_ADDR);
	code[n++] = LA_ST_D(21, 15, LA_PRACC_SLOT_SAVE);
	code[n++] = LA_CSRRD(15, LA_CSR_DEBUG_SAVE);
	n = loongarch_emit_address(code, n, 21, handler_addr);
	/* The handler was just written as data.  Complete those stores and
	 * discard any stale instruction fetch before entering the RAM code. */
	code[n++] = LA_DBAR;
	code[n++] = LA_IBAR;
	code[n++] = LA_JIRL(0, 21,
		(entry_addr - handler_addr) / sizeof(uint32_t));
	code[n++] = LA_ANDI_ZERO;
	return n;
}

static unsigned int loongarch_fastdata_cleanup_build(uint32_t *code)
{
	unsigned int n = 0;

	code[n++] = LA_CSRWR(15, LA_CSR_DEBUG_SAVE);
	n = loongarch_emit_address(code, n, 15, LA_PRACC_BASE_ADDR);
	code[n++] = LA_LD_D(21, 15, LA_PRACC_SLOT_SAVE);
	code[n++] = LA_CSRRD(15, LA_CSR_DEBUG_SAVE);
	code[n++] = LA_DBAR;
	unsigned int branch_offset = n;
	code[n++] = LA_B(-(int)branch_offset);
	code[n++] = LA_ANDI_ZERO;
	return n;
}

static int loongarch_pracc_enter_fastdata_handler(struct loongarch_ejtag *ej,
	uint64_t handler_addr, uint64_t entry_addr, const uint32_t *handler,
	unsigned int handler_len, uint64_t *saved_r21)
{
	uint32_t code[16];
	uint64_t mailbox[0x80] = { 0 };
	unsigned int code_len = loongarch_fastdata_jump_build(code, handler_addr,
		entry_addr);
	unsigned int out_seq = 0;
	unsigned int out_events = 0;
	unsigned int in_seq = 0;

	while (1) {
		bool left_debug = false;
		uint32_t ctrl = 0;
		uint64_t data = 0;
		uint64_t addr = 0;
		int retval = loongarch_pracc_wait_access(ej, &ctrl, &data, &addr,
			&left_debug);
		if (retval != ERROR_OK)
			return retval;
		if (left_debug)
			return ERROR_TARGET_NOT_HALTED;
		if ((ctrl & LA_CTRL_PRACC) && loongarch_fastdata_access(addr)) {
			*saved_r21 = mailbox[LA_PRACC_SLOT_SAVE / 8];
			return ERROR_OK;
		}

		uint64_t slot = 0;
		if ((ctrl & LA_CTRL_PRACC) && loongarch_mailbox_slot(addr, &slot)) {
			retval = loongarch_pracc_service_mailbox(ej, ctrl, slot, data,
				mailbox, ARRAY_SIZE(mailbox), &out_seq, &out_events,
				mailbox, ARRAY_SIZE(mailbox), &in_seq);
			if (retval != ERROR_OK)
				return retval;
			continue;
		}

		uint64_t instruction_data = 0;
		unsigned int pair_index = 0;
		if (!loongarch_pracc_instruction_pair(code, code_len, addr,
				&instruction_data, &pair_index) &&
			!loongarch_fastdata_handler_instruction_pair(handler_addr,
				handler, handler_len, addr, &instruction_data, &pair_index))
			return ERROR_FAIL;
		retval = loongarch_pracc_feed_instruction(ej, addr,
			instruction_data, pair_index);
		if (retval != ERROR_OK)
			return retval;

		/* The 2K300 can request the safe tail word after JIRL before the
		 * handler's first FASTDATA access becomes visible.  Keep servicing
		 * those instruction fetches; switch to IR=FASTDATA only after the
		 * address register reports the FASTDATA mailbox above. */
	}
}

static int loongarch_fastdata_exchange(struct loongarch_ejtag *ej,
	uint64_t outbound, uint64_t *inbound)
{
	struct loongarch_ejtag_fastdata fastdata;
	fastdata.spracc = false;
	fastdata.data = outbound;
	int retval = loongarch_ejtag_scan_fastdata(ej, &fastdata);
	if (retval != ERROR_OK)
		return retval;
	if (!fastdata.spracc)
		return ERROR_TARGET_TIMEOUT;
	if (inbound)
		*inbound = fastdata.data;
	LOG_DEBUG("loongarch: continuous FASTDATA out=0x%016" PRIx64
		" in=0x%016" PRIx64, outbound, fastdata.data);
	return ERROR_OK;
}

static int loongarch_fastdata_wait_marker(struct loongarch_ejtag *ej,
	uint64_t expected, uint64_t outbound)
{
	for (unsigned int attempt = 0; attempt < 8; attempt++) {
		uint64_t marker = 0;
		int retval = loongarch_fastdata_exchange(ej, outbound, &marker);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("loongarch: FASTDATA sync slot %u = 0x%016" PRIx64,
			attempt, marker);
		if (marker == expected)
			return ERROR_OK;
	}

	LOG_ERROR("loongarch: FASTDATA marker 0x%016" PRIx64
		" not observed", expected);
	return ERROR_TARGET_TIMEOUT;
}

static int loongarch_fastdata_write_values(struct loongarch_ejtag *ej,
	const uint64_t *values, unsigned int count)
{
	/* The target consumes the host DATA payload one scan later.  Send every
	 * value once, then repeat the final value to retire the last pending load. */
	for (unsigned int i = 0; i < count; i++) {
		int retval = loongarch_fastdata_exchange(ej, values[i], NULL);
		if (retval != ERROR_OK)
			return retval;
	}

	return loongarch_fastdata_exchange(ej, values[count - 1], NULL);
}

static int loongarch_fastdata_transfer_once(struct loongarch_ejtag *ej,
	uint64_t addr, unsigned int count, const uint64_t *in, uint64_t *out,
	bool write_t)
{
	uint64_t descriptor = count;
	int retval = loongarch_fastdata_exchange(ej, addr, NULL);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_fastdata_exchange(ej, descriptor, NULL);
	if (retval != ERROR_OK)
		return retval;
	if (!write_t) {
		retval = loongarch_fastdata_wait_marker(ej, LA_FASTDATA_ADDR, 0);
		if (retval != ERROR_OK)
			return retval;
		for (unsigned int i = 0; i < count; i++) {
			retval = loongarch_fastdata_exchange(ej, 0, &out[i]);
			if (retval != ERROR_OK)
				return retval;
			uint64_t remaining = 0;
			retval = loongarch_fastdata_exchange(ej, 0, &remaining);
			if (retval != ERROR_OK)
				return retval;
			uint64_t expected = count - i - 1;
			if (remaining != expected) {
				LOG_ERROR("loongarch: FASTDATA read acknowledgement is "
					"0x%016" PRIx64 ", expected 0x%016" PRIx64,
					remaining, expected);
				return ERROR_FAIL;
			}
		}
		return ERROR_OK;
	}

	retval = loongarch_fastdata_write_values(ej, in, count);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

int loongarch_pracc_fastdata_finish(struct loongarch_ejtag *ej)
{
	struct loongarch_fastdata_session *session = ej->fastdata_session;
	if (!session)
		return ERROR_OK;

	uint32_t cleanup[12];
	unsigned int cleanup_len = loongarch_fastdata_cleanup_build(cleanup);
	uint64_t mailbox[0x80] = { 0 };
	mailbox[LA_PRACC_SLOT_SAVE / 8] = session->saved_r21;
	int retval = loongarch_pracc_exec_raw(ej, cleanup_len, cleanup,
		mailbox, ARRAY_SIZE(mailbox), NULL, NULL,
		mailbox, ARRAY_SIZE(mailbox), false);
	if (retval != ERROR_OK)
		return retval;

	/* The processor has returned to dmseg, so normal PrAcc can safely restore
	 * the temporary RAM area without recursively finishing this session. */
	ej->fastdata_session = NULL;
	retval = loongarch_pracc_write_mem_width(ej, session->handler_addr, 8,
		ARRAY_SIZE(session->backup), session->backup);
	if (retval == ERROR_OK) {
		uint64_t restored[LA_FASTDATA_HANDLER_AREA_WORDS];
		retval = loongarch_pracc_read_mem_width(ej, session->handler_addr, 8,
			ARRAY_SIZE(restored), restored);
		if (retval == ERROR_OK &&
				memcmp(session->backup, restored, sizeof(restored)) != 0) {
			LOG_ERROR("loongarch: FASTDATA handler area restore verification failed");
			retval = ERROR_FAIL;
		}
	}
	free(session);
	return retval;
}

static int loongarch_pracc_fastdata_transfer64(struct loongarch_ejtag *ej,
	uint64_t handler_addr, uint64_t addr, unsigned int count,
	const uint64_t *in, uint64_t *out, bool write_t)
{
	uint64_t handler_values[LA_FASTDATA_HANDLER_WORDS];
	uint32_t handler[LA_FASTDATA_HANDLER_WORDS];

	if ((write_t ? !in : !out) || count == 0 || count > 256 ||
		(handler_addr & 7) != 0 ||
		(addr & 7) != 0 || addr + (uint64_t)count * 8 < addr)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if (!ej->fastdata_session) {
		struct loongarch_fastdata_session *session = calloc(1,
			sizeof(*session));
		if (!session)
			return ERROR_FAIL;
		session->handler_addr = handler_addr;

		int retval = loongarch_pracc_read_mem_width(ej, handler_addr, 8,
			ARRAY_SIZE(session->backup), session->backup);
		if (retval != ERROR_OK) {
			free(session);
			return retval;
		}

		loongarch_fastdata_handler_build(handler);
		for (unsigned int i = 0; i < ARRAY_SIZE(handler); i++)
			handler_values[i] = handler[i];
		retval = loongarch_pracc_write_mem_width(ej, handler_addr, 4,
			ARRAY_SIZE(handler_values), handler_values);
		if (retval != ERROR_OK) {
			free(session);
			return retval;
		}

		uint64_t entry_addr = handler_addr + (write_t ?
			LA_FASTDATA_WRITE_ENTRY_WORD * sizeof(uint32_t) : 0);
		retval = loongarch_pracc_enter_fastdata_handler(ej, handler_addr,
			entry_addr, handler, ARRAY_SIZE(handler), &session->saved_r21);
		if (retval != ERROR_OK) {
			free(session);
			LOG_ERROR("loongarch: FASTDATA handler entry failed; target may require reset");
			return retval;
		}
		ej->fastdata_session = session;
	} else if (ej->fastdata_session->handler_addr != handler_addr) {
		LOG_ERROR("loongarch: active FASTDATA session uses handler 0x%016" PRIx64,
			ej->fastdata_session->handler_addr);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	int retval = loongarch_fastdata_transfer_once(ej, addr, count, in, out,
		write_t);
	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch: FASTDATA transfer failed; target may require reset");
		return retval;
	}

	return loongarch_pracc_fastdata_finish(ej);
}

int loongarch_pracc_fastdata_read64(struct loongarch_ejtag *ej,
	uint64_t handler_addr, uint64_t addr, unsigned int count, uint64_t *out)
{
	return loongarch_pracc_fastdata_transfer64(ej, handler_addr, addr,
		count, NULL, out, false);
}

int loongarch_pracc_fastdata_write64(struct loongarch_ejtag *ej,
	uint64_t handler_addr, uint64_t addr, unsigned int count,
	const uint64_t *in)
{
	return loongarch_pracc_fastdata_transfer64(ej, handler_addr, addr,
		count, in, NULL, true);
}
