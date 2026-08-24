/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 * LoongArch EJTAG probe primitives.
 *
 * Mirrors OpenOCD's mips_ejtag.c but with a parameterized instruction
 * register (LoongArch TAPs use 4-bit IR codes configured via probe
 * registers instead of fixed 5-bit MIPS codes).
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <inttypes.h>

#include <helper/time_support.h>
#include <target/target.h>

#include "loongarch_ejtag.h"
#include "loongarch_pracc.h"

void loongarch_ejtag_set_instr(struct loongarch_ejtag *ej, uint32_t instr)
{
	assert(ej->tap);
	struct jtag_tap *tap = ej->tap;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != instr) {
		struct scan_field field;
		uint8_t t[4] = { 0 };

		field.num_bits = ej->ir_len;
		field.out_value = t;
		field.in_value = NULL;
		buf_set_u32(t, 0, field.num_bits, instr);

		jtag_add_ir_scan(tap, &field, TAP_IDLE);
	}
}

void loongarch_ejtag_set_instr_force(struct loongarch_ejtag *ej, uint32_t instr)
{
	assert(ej->tap);
	struct scan_field field;
	uint8_t value[4] = { 0 };

	field.num_bits = ej->ir_len;
	field.out_value = value;
	field.in_value = NULL;
	buf_set_u32(value, 0, field.num_bits, instr);
	jtag_add_ir_scan_noverify(ej->tap, &field, TAP_IDLE);
}

static void loongarch_ejtag_drscan_32_queued(struct loongarch_ejtag *ej,
		uint32_t data_out, uint8_t *data_in)
{
	struct scan_field field;
	uint8_t scan_out[4] = { 0 };

	field.num_bits = 32;
	field.out_value = scan_out;
	field.in_value = data_in;
	buf_set_u32(scan_out, 0, 32, data_out);

	jtag_add_dr_scan(ej->tap, 1, &field, TAP_IDLE);
	keep_alive();
}

int loongarch_ejtag_drscan_32(struct loongarch_ejtag *ej, uint32_t *data)
{
	uint8_t scan_in[4];

	loongarch_ejtag_drscan_32_queued(ej, *data, scan_in);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch ejtag register read failed");
		return retval;
	}

	*data = buf_get_u32(scan_in, 0, 32);
	return ERROR_OK;
}

void loongarch_ejtag_drscan_32_out(struct loongarch_ejtag *ej, uint32_t data)
{
	loongarch_ejtag_drscan_32_queued(ej, data, NULL);
}

int loongarch_ejtag_drscan_64(struct loongarch_ejtag *ej, uint64_t *data)
{
	struct scan_field field;
	uint8_t t[8] = { 0 }, r[8];

	field.num_bits = 64;
	field.out_value = t;
	field.in_value = r;
	buf_set_u64(t, 0, 64, *data);

	jtag_add_dr_scan(ej->tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch ejtag register read failed");
		return retval;
	}

	*data = buf_get_u64(r, 0, 64);
	keep_alive();

	return ERROR_OK;
}

void loongarch_ejtag_drscan_64_out(struct loongarch_ejtag *ej, uint64_t data)
{
	struct scan_field field;
	uint8_t scan_out[8] = { 0 };

	field.num_bits = 64;
	field.out_value = scan_out;
	field.in_value = NULL;
	buf_set_u64(scan_out, 0, 64, data);

	jtag_add_dr_scan(ej->tap, 1, &field, TAP_IDLE);
	keep_alive();
}

int loongarch_ejtag_scan_all(struct loongarch_ejtag *ej,
	struct loongarch_ejtag_all *all)
{
	if (!all)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	struct scan_field fields[3];
	uint8_t control_out[4] = { 0 };
	uint8_t control_in[4] = { 0 };
	uint8_t data_out[8] = { 0 };
	uint8_t data_in[8] = { 0 };
	uint8_t address_out[8] = { 0 };
	uint8_t address_in[8] = { 0 };

	buf_set_u32(control_out, 0, 32, all->control);
	buf_set_u64(data_out, 0, 64, all->data);
	buf_set_u64(address_out, 0, 64, all->address);

	fields[0].num_bits = 32;
	fields[0].out_value = control_out;
	fields[0].in_value = control_in;
	fields[1].num_bits = 64;
	fields[1].out_value = data_out;
	fields[1].in_value = data_in;
	fields[2].num_bits = 64;
	fields[2].out_value = address_out;
	fields[2].in_value = address_in;

	/* A completed PrAcc can invalidate the hardware-selected IR without
	 * updating OpenOCD's TAP cache, so every ALL poll re-selects IR=7. */
	loongarch_ejtag_set_instr_force(ej, ej->ir_all);
	jtag_add_dr_scan(ej->tap, ARRAY_SIZE(fields), fields, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch EJTAG ALL scan failed");
		return retval;
	}

	all->control = buf_get_u32(control_in, 0, 32);
	all->data = buf_get_u64(data_in, 0, 64);
	all->address = buf_get_u64(address_in, 0, 64);
	keep_alive();
	return ERROR_OK;
}

int loongarch_ejtag_scan_fastdata(struct loongarch_ejtag *ej,
	struct loongarch_ejtag_fastdata *fastdata)
{
	if (!fastdata)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	struct scan_field field;
	uint8_t out[9] = { 0 };
	uint8_t in[9] = { 0 };

	buf_set_u32(out, 0, 1, fastdata->spracc ? 1 : 0);
	buf_set_u64(out, 1, 64, fastdata->data);

	field.num_bits = 65;
	field.out_value = out;
	field.in_value = in;

	/* Completing a processor access can invalidate the hardware-selected IR
	 * without updating OpenOCD's cache, so re-select FASTDATA for every word.
	 * The target also needs to observe Update-IR before Capture-DR.  Keep the
	 * two operations in separate adapter transactions, matching the official
	 * MPSSE implementation's flush between them. */
	loongarch_ejtag_set_instr_force(ej, ej->ir_fastdata);
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch EJTAG FASTDATA instruction select failed");
		return retval;
	}

	jtag_add_dr_scan(ej->tap, 1, &field, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch EJTAG FASTDATA scan failed");
		return retval;
	}

	fastdata->spracc = buf_get_u32(in, 0, 1) != 0;
	fastdata->data = buf_get_u64(in, 1, 64);
	keep_alive();
	return ERROR_OK;
}

static int loongarch_ejtag_drscan_pcsample65(struct loongarch_ejtag *ej,
	uint64_t *low, uint32_t *high)
{
	struct scan_field field;
	uint8_t out[9] = { 0 };
	uint8_t in[9] = { 0 };

	field.num_bits = 65;
	field.out_value = out;
	field.in_value = in;

	jtag_add_dr_scan(ej->tap, 1, &field, TAP_IDLE);
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch PCSAMPLE read failed");
		return retval;
	}

	*low = buf_get_u64(in, 0, 64);
	*high = buf_get_u32(in, 64, 1);
	return ERROR_OK;
}

int loongarch_ejtag_read_pcsample(struct loongarch_ejtag *ej,
	uint64_t *pc)
{
	uint64_t raw65;
	uint32_t high65;
	uint64_t raw64;
	int retval;

	if (!pc)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	loongarch_ejtag_set_instr(ej, ej->ir_pcsample);
	retval = loongarch_ejtag_drscan_pcsample65(ej, &raw65, &high65);
	if (retval != ERROR_OK)
		return retval;

	/* Some 2K300 samples contain a stale high scan bit.  A second 64-bit
	 * read is stable; use the 65-bit value when both scans agree on the
	 * low 63 PC bits, otherwise apply the documented high-bit fallback. */
	retval = loongarch_ejtag_drscan_64(ej, &raw64);
	if (retval != ERROR_OK)
		return retval;

	if ((raw65 >> 1) == (raw64 >> 1))
		*pc = (raw65 >> 1) | ((uint64_t)high65 << 63);
	else
		*pc = (raw64 >> 1) | ((raw64 >> 32) ? (1ull << 63) : 0);

	return ERROR_OK;
}

int loongarch_ejtag_get_impcode(struct loongarch_ejtag *ej)
{
	loongarch_ejtag_set_instr(ej, ej->ir_impcode);

	ej->impcode = 0;
	return loongarch_ejtag_drscan_32(ej, &ej->impcode);
}

int loongarch_ejtag_get_idcode(struct loongarch_ejtag *ej)
{
	loongarch_ejtag_set_instr(ej, ej->ir_idcode);

	ej->idcode = 0;
	return loongarch_ejtag_drscan_32(ej, &ej->idcode);
}

int loongarch_ejtag_enter_debug(struct loongarch_ejtag *ej)
{
	const uint32_t vector = ej->ejtag_ctrl & LA_CTRL_PROBEVEC;
	const uint32_t steady = LA_CTRL_PRACC | LA_CTRL_PROBEN | vector;
	const uint32_t request = steady | LA_CTRL_JTAGBRK;
	int64_t then = timeval_ms();
	int retval;

	loongarch_ejtag_set_instr(ej, ej->ir_control);

	/* A previous ERTN can post one delayed dmseg fetch after the resume
	 * routine has returned.  Service that real PrAcc before asserting a new
	 * JTAG break; otherwise LS2K300 keeps JTAGBRK pending at 0x4d000 and
	 * never raises BRKST for the second halt. */
	uint32_t pending = steady;
	retval = loongarch_ejtag_drscan_32(ej, &pending);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("loongarch: halt preflight control=0x%08" PRIx32,
		pending);
	if (!(pending & LA_CTRL_BRKST) && (pending & LA_CTRL_PRACC)) {
		uint64_t data = 0;

		/* This is the delayed instruction fetch after ERTN.  The hardware
		 * completes it with CONTROL=0xc000 directly;
		 * do not insert an ADDRESS read here, which changes the pending
		 * fetch state on LS2K300. */
		if (pending & LA_CTRL_PRNW) {
			loongarch_ejtag_set_instr(ej, ej->ir_data);
			retval = loongarch_ejtag_drscan_64(ej, &data);
			if (retval != ERROR_OK)
				return retval;
		}
		loongarch_ejtag_set_instr(ej, ej->ir_control);
		pending = LA_CTRL_PROBEN | vector;
		retval = loongarch_ejtag_drscan_32(ej, &pending);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("loongarch: halt preflight release=0x%08" PRIx32,
			pending);
	}

	while (timeval_ms() - then <= 5000) {
		/* Poll with the steady CONTROL value before issuing the one-shot
		 * break request. */
		uint32_t ctrl = steady;
		retval = loongarch_ejtag_drscan_32(ej, &ctrl);
		if (retval != ERROR_OK)
			return retval;
		if (ctrl & LA_CTRL_BRKST)
			return ERROR_OK;

		/* Do not capture the old CONTROL value while issuing JTAGBRK. */
		loongarch_ejtag_drscan_32_out(ej, request);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		/* Restore the steady CONTROL word and capture the resulting state. */
		ctrl = steady;
		retval = loongarch_ejtag_drscan_32(ej, &ctrl);
		if (retval != ERROR_OK)
			return retval;
		if (ctrl & LA_CTRL_BRKST)
			return ERROR_OK;

		keep_alive();
	}

	LOG_ERROR("Failed to enter debug mode (ctrl=0x%8.8" PRIx32 ")", request);
	return ERROR_FAIL;
}

int loongarch_ejtag_exit_debug(struct loongarch_ejtag *ej)
{
	/* execute the LoongArch DERET (ertn): after it is fed, every
	 * residual fetch is completed with 0xc000 only (no instruction),
	 * matching the observed LS2K300 extra-fetch behavior. */
	const uint32_t code[] = { LA_DERET };
	int retval = loongarch_pracc_exec(ej, ARRAY_SIZE(code), code,
					 NULL, 0, NULL, NULL, NULL, 0, true);
	if (retval != ERROR_OK)
		return retval;

	/* LS2K0300 may clear BRKST after ERTN while leaving a delayed
	 * instruction-fetch transaction.  Do not stop at the first clean
	 * CONTROL sample because another fetch can become visible on the
	 * following scan.  Emit the complete bounded
	 * release tail every time. */
	const uint32_t release = LA_CTRL_PROBEN |
		(ej->ejtag_ctrl & LA_CTRL_PROBEVEC);
	const uint32_t steady = LA_CTRL_PRACC | release;
	uint32_t ctrl = release;
	uint64_t address;
	uint64_t data;
	loongarch_ejtag_set_instr(ej, ej->ir_control);
	for (unsigned int pass = 0; pass < 5; pass++) {
		/* A delayed LS2K300 fetch is a real PrAcc transaction.  Reading
		 * CONTROL alone is insufficient: consume ADDRESS (and DATA for a
		 * posted store) before issuing the completion word. */
		ctrl = steady;
		retval = loongarch_ejtag_drscan_32(ej, &ctrl);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("loongarch: exit tail pass=%u poll=0x%08" PRIx32,
			pass, ctrl);
		/* loongarch_pracc_exec() returned only after observing BRKST clear.
		 * If it is set again now, the core has already entered a new debug
		 * exception (for example, an instruction breakpoint immediately after
		 * ERTN).  Leave that exception pending for loongarch_poll(); releasing
		 * it here would consume the new stop as if it were an ERTN tail. */
		if (ctrl & LA_CTRL_BRKST)
			return ERROR_OK;
		if (ctrl & LA_CTRL_PRACC) {
			loongarch_ejtag_set_instr(ej, ej->ir_address);
			address = 0;
			retval = loongarch_ejtag_drscan_64(ej, &address);
			if (retval != ERROR_OK)
				return retval;
			LOG_DEBUG("loongarch: exit tail pass=%u address=0x%016" PRIx64,
				pass, address);
			if (ctrl & LA_CTRL_PRNW) {
				loongarch_ejtag_set_instr(ej, ej->ir_data);
				data = 0;
				retval = loongarch_ejtag_drscan_64(ej, &data);
				if (retval != ERROR_OK)
					return retval;
			}
			loongarch_ejtag_set_instr(ej, ej->ir_control);
		}
		ctrl = release;
		retval = loongarch_ejtag_drscan_32(ej, &ctrl);
		if (retval != ERROR_OK)
			return retval;
	}

	/* The final release itself starts the core again.  One more delayed
	 * CONTROL poll is therefore required after that write; this is the
	 * extra dmseg fetch documented for LS2K300. */
	ctrl = steady;
	retval = loongarch_ejtag_drscan_32(ej, &ctrl);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("loongarch: exit tail final poll=0x%08" PRIx32, ctrl);
	if (ctrl & LA_CTRL_BRKST)
		return ERROR_OK;
	if (ctrl & LA_CTRL_PRACC) {
		loongarch_ejtag_set_instr(ej, ej->ir_address);
		address = 0;
		retval = loongarch_ejtag_drscan_64(ej, &address);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("loongarch: exit tail final address=0x%016" PRIx64,
			address);
		if (ctrl & LA_CTRL_PRNW) {
			loongarch_ejtag_set_instr(ej, ej->ir_data);
			data = 0;
			retval = loongarch_ejtag_drscan_64(ej, &data);
			if (retval != ERROR_OK)
				return retval;
		}
		loongarch_ejtag_set_instr(ej, ej->ir_control);
	}
	ctrl = release;
	retval = loongarch_ejtag_drscan_32(ej, &ctrl);
	if (retval != ERROR_OK)
		return retval;
	if (ctrl & LA_CTRL_PRACC) {
		LOG_ERROR("loongarch: residual debug access after ERTN "
			"(control=0x%08" PRIx32 ")", ctrl);
		return ERROR_TARGET_TIMEOUT;
	}
	return ERROR_OK;
}

int loongarch_ejtag_init(struct loongarch_ejtag *ej)
{
	ej->ejtag_ctrl = LA_CTRL_PRACC | LA_CTRL_PROBEN | LA_CTRL_PROBEVEC;

	int retval = loongarch_ejtag_get_impcode(ej);
	if (retval != ERROR_OK) {
		LOG_ERROR("impcode read failed");
		return retval;
	}

	retval = loongarch_ejtag_get_idcode(ej);
	if (retval != ERROR_OK) {
		LOG_ERROR("idcode read failed");
		return retval;
	}

	return ERROR_OK;
}

void loongarch_ejtag_set_probe_vector(struct loongarch_ejtag *ej, bool dmseg)
{
	if (dmseg)
		ej->ejtag_ctrl |= LA_CTRL_PROBEVEC;
	else
		ej->ejtag_ctrl &= ~LA_CTRL_PROBEVEC;
}
