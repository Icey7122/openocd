/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 * LoongArch (Loongson 2K series) EJTAG target for OpenOCD.
 *
 * Derived from the official la_dbg_tool_usb binary, its LS2K300 target
 * configuration, and measurements on real LS2K300 hardware.
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <inttypes.h>

#include <helper/command.h>
#include <helper/log.h>
#include <helper/time_support.h>
#include <target/breakpoints.h>
#include <target/target.h>
#include <target/register.h>
#include <target/target_type.h>
#include <jtag/interface.h>

#include "loongarch.h"
#include "loongarch_pracc.h"

static struct loongarch_common *target_to_loongarch(struct target *target)
{
	return target->arch_info;
}

static void loongarch_invalidate_register_cache(struct loongarch_common *la)
{
	la->regs_valid = false;
	la->fpu_enabled_valid = false;
	for (unsigned int i = 0; la->regs && i < la->reg_count; i++) {
		la->regs[i].valid = false;
		la->regs[i].dirty = false;
	}
}

static int loongarch_add_breakpoint(struct target *target,
	struct breakpoint *breakpoint);
static int loongarch_remove_breakpoint(struct target *target,
	struct breakpoint *breakpoint);
static int loongarch_hit_watchpoint(struct target *target,
	struct watchpoint **hit_watchpoint);
static int loongarch_step(struct target *target, bool current,
	target_addr_t address, bool handle_breakpoints);
static int loongarch_add_hardware_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint64_t address_mask);

#define LOONGARCH_SOFTWARE_BREAK_INSTRUCTION 0x002A8000u
#define LOONGARCH_DEBUG_DIB (1ull << 10)
#define LOONGARCH_DEBUG_DMW (1ull << 11)
#define LOONGARCH_BREAKPOINT_STATUS_MASK 0x1FFFFu
#define LOONGARCH_CSR_EUEN 0x2u
#define LOONGARCH_CSR_DEPC 0x501u
#define LOONGARCH_EUEN_FPE (1ull << 0)
#define LOONGARCH_PCSAMPLE_MAX_SAMPLES 100000u
#define LOONGARCH_PCSAMPLE_MAX_DURATION_MS 3600000u
#define LOONGARCH_PCSAMPLE_MAX_INTERVAL_MS 60000u

enum loongarch_reg_number {
	LOONGARCH_REG_R0 = 0,
	LOONGARCH_REG_PC = 32,
	LOONGARCH_REG_BADV,
	LOONGARCH_REG_F0,
	LOONGARCH_REG_F31 = LOONGARCH_REG_F0 + 31,
	LOONGARCH_REG_FCC,
	LOONGARCH_REG_FCSR,
	LOONGARCH_REG_COUNT,
};

#define LOONGARCH_CORE_REG_COUNT (LOONGARCH_REG_PC + 1)
#define LOONGARCH_CSR_BADV 0x7u

static uint64_t loongarch_sign_extend(uint64_t value, unsigned int bits)
{
	uint64_t sign_bit = 1ull << (bits - 1);

	return (value ^ sign_bit) - sign_bit;
}

static uint64_t loongarch_branch_offset(uint32_t immediate,
	unsigned int bits)
{
	return loongarch_sign_extend(immediate, bits) << 2;
}

static int loongarch_next_pc(struct target *target, uint64_t pc,
	uint64_t *next_pc)
{
	struct loongarch_common *la = target_to_loongarch(target);
	uint64_t regs[33];
	uint32_t instruction;
	uint32_t masked;
	uint32_t immediate;
	int retval;

	if (!next_pc)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	retval = target_read_u32(target, pc, &instruction);
	if (retval != ERROR_OK)
		return retval;

	masked = instruction & 0xF8000000u;
	if (masked == 0x40000000u ||
		(instruction & 0xFC000200u) == 0x48000000u) {
		immediate = ((instruction & 0x1Fu) << 16) |
			((instruction >> 10) & 0xFFFFu);
		*next_pc = pc + loongarch_branch_offset(immediate, 21);
		return ERROR_OK;
	}

	masked = instruction & 0xFC0003E0u;
	if (masked == 0x48000200u || masked == 0x48000300u) {
		LOG_TARGET_ERROR(target,
			"single-step does not yet expose the official SCR0/SCR1 branch base");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((instruction & 0xFC000000u) == 0x4C000000u) {
		unsigned int source_register = (instruction >> 5) & 0x1Fu;

		retval = loongarch_pracc_read_regs(&la->ejtag_info, regs,
			ARRAY_SIZE(regs));
		if (retval != ERROR_OK)
			return retval;
		immediate = (instruction >> 10) & 0xFFFFu;
		*next_pc = regs[source_register] +
			loongarch_branch_offset(immediate, 16);
		return ERROR_OK;
	}

	if ((instruction & 0xF8000000u) == 0x50000000u) {
		immediate = ((instruction & 0x3FFu) << 16) |
			((instruction >> 10) & 0xFFFFu);
		*next_pc = pc + loongarch_branch_offset(immediate, 26);
		return ERROR_OK;
	}

	if ((instruction & 0xF0000000u) == 0x60000000u ||
		(instruction & 0xF8000000u) == 0x58000000u) {
		immediate = (instruction >> 10) & 0xFFFFu;
		*next_pc = pc + loongarch_branch_offset(immediate, 16);
		return ERROR_OK;
	}

	*next_pc = pc + sizeof(uint32_t);
	return ERROR_OK;
}

static int loongarch_add_step_breakpoint(struct target *target,
	uint64_t address, struct breakpoint *breakpoint, uint8_t *original)
{
	struct breakpoint *existing = breakpoint_find(target, address);

	if (existing && existing->is_set)
		return ERROR_OK;

	memset(breakpoint, 0, sizeof(*breakpoint));
	breakpoint->address = address;
	breakpoint->length = sizeof(uint32_t);
	breakpoint->type = BKPT_SOFT;
	breakpoint->orig_instr = original;
	return loongarch_add_breakpoint(target, breakpoint);
}

static int loongarch_arch_state(struct target *target)
{
	return ERROR_OK;
}

static const char *loongarch_get_gdb_arch(const struct target *target)
{
	(void)target;
	return "loongarch64";
}

static unsigned int loongarch_target_width(struct target *target)
{
	(void)target;
	return 64;
}

static int loongarch_poll(struct target *target)
{
	struct loongarch_common *la = target_to_loongarch(target);
	struct loongarch_ejtag *ej = &la->ejtag_info;
	uint32_t ctrl;

	loongarch_ejtag_set_instr(ej, ej->ir_control);
	ctrl = ej->ejtag_ctrl;
	int retval = loongarch_ejtag_drscan_32(ej, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & LA_CTRL_BRKST) {
		uint64_t debug;
		bool classified = false;

		/* loongarch_halt() has already established a deliberate DBGRQ
		 * stop.  Do not immediately launch a PrAcc CSR module just to
		 * classify that same stop: on LS2K300 the processor-access data
		 * path may be unavailable, while CONTROL/PCSAMPLE are still
		 * usable.  Keep the stop and defer CSR classification to an
		 * asynchronous breakpoint/watchpoint poll. */
		if (target->state == TARGET_HALTED &&
			target->debug_reason == DBG_REASON_DBGRQ)
			goto mark_halted;

		retval = loongarch_pracc_read_csr(&la->ejtag_info, 0x500, &debug);
		if (retval != ERROR_OK)
			return retval;

		if (debug & LOONGARCH_DEBUG_DMW) {
			struct watchpoint *hit_watchpoint = NULL;
			int hit_retval = loongarch_hit_watchpoint(target,
				&hit_watchpoint);
			if (hit_retval == ERROR_OK)
				classified = true;
			else if (hit_retval != ERROR_FAIL &&
				hit_retval != ERROR_WATCHPOINT_NOT_FOUND)
				return hit_retval;
		}

		if (!classified && (debug & LOONGARCH_DEBUG_DIB)) {
			retval = loongarch_pracc_write_csr(&la->ejtag_info, 0x381,
				LOONGARCH_BREAKPOINT_STATUS_MASK);
			if (retval != ERROR_OK)
				return retval;
			if (target->debug_reason != DBG_REASON_SINGLESTEP)
				target->debug_reason = DBG_REASON_BREAKPOINT;
			classified = true;
		}

	mark_halted:
		if (target->state != TARGET_HALTED) {
			loongarch_invalidate_register_cache(la);
			target->state = TARGET_HALTED;
			if (!classified && target->debug_reason == DBG_REASON_NOTHALTED)
				target->debug_reason = DBG_REASON_DBGRQ;
			LOG_INFO("loongarch: target halted");
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	} else if (target->state == TARGET_HALTED) {
		target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}

static int loongarch_halt(struct target *target)
{
	struct loongarch_common *la = target_to_loongarch(target);
	int retval = loongarch_ejtag_enter_debug(&la->ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	loongarch_invalidate_register_cache(la);
	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

static int loongarch_resume(struct target *target, bool current,
	target_addr_t address, bool handle_breakpoints, bool debug_execution)
{
	struct loongarch_common *la = target_to_loongarch(target);
	struct breakpoint *skip_breakpoint = NULL;
	uint64_t resume_pc;
	bool breakpoint_removed = false;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (current) {
		retval = loongarch_pracc_read_csr(&la->ejtag_info,
			LOONGARCH_CSR_DEPC, &resume_pc);
		if (retval != ERROR_OK)
			return retval;
	} else {
		resume_pc = address;
	}

	if (handle_breakpoints) {
		for (struct breakpoint *breakpoint = target->breakpoints;
			breakpoint; breakpoint = breakpoint->next) {
			if (breakpoint->is_set && breakpoint->address == resume_pc) {
				skip_breakpoint = breakpoint;
				break;
			}
		}

		if (skip_breakpoint) {
			retval = loongarch_remove_breakpoint(target, skip_breakpoint);
			if (retval != ERROR_OK)
				return retval;
			breakpoint_removed = true;

			/* Execute the instruction hidden by the breakpoint while the
			 * target is halted, then restore the original breakpoint. */
			retval = loongarch_step(target, true, 0, false);
			if (retval != ERROR_OK)
				goto restore_breakpoint;

			retval = loongarch_add_breakpoint(target, skip_breakpoint);
			if (retval != ERROR_OK)
				return retval;
			breakpoint_removed = false;
		}
	}

	if (!current) {
		/* DERA is the architected debug exception return address.  The
		 * official client writes it before DERET when resuming elsewhere. */
		retval = loongarch_pracc_write_csr(&la->ejtag_info, 0x501, address);
		if (retval != ERROR_OK)
			goto restore_breakpoint;
	}
	(void)debug_execution;

	retval = loongarch_ejtag_exit_debug(&la->ejtag_info);
	if (retval != ERROR_OK)
		goto restore_breakpoint;

	loongarch_invalidate_register_cache(la);
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	return ERROR_OK;

restore_breakpoint:
	if (breakpoint_removed && target->state == TARGET_HALTED) {
		int restore_retval = loongarch_add_breakpoint(target, skip_breakpoint);
		if (retval == ERROR_OK)
			retval = restore_retval;
	}
	return retval;
}

static int loongarch_step(struct target *target, bool current,
	target_addr_t address, bool handle_breakpoints)
{
	struct loongarch_common *la = target_to_loongarch(target);
	struct breakpoint target_breakpoint = { 0 };
	struct breakpoint fallthrough_breakpoint = { 0 };
	struct breakpoint *step_over_breakpoint = NULL;
	uint8_t target_original[sizeof(uint32_t)];
	uint8_t fallthrough_original[sizeof(uint32_t)];
	uint64_t next_pc;
	uint64_t fallthrough_pc;
	uint64_t pc;
	bool breakpoint_removed = false;
	int64_t timeout;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (current) {
		retval = loongarch_pracc_read_csr(&la->ejtag_info,
			LOONGARCH_CSR_DEPC, &pc);
	} else {
		pc = address;
		retval = loongarch_pracc_write_csr(&la->ejtag_info,
			LOONGARCH_CSR_DEPC, pc);
	}
	if (retval != ERROR_OK)
		return retval;

	if (handle_breakpoints) {
		for (struct breakpoint *breakpoint = target->breakpoints;
			breakpoint; breakpoint = breakpoint->next) {
			if (breakpoint->is_set && breakpoint->address == pc) {
				step_over_breakpoint = breakpoint;
				break;
			}
		}

		if (step_over_breakpoint) {
			retval = loongarch_remove_breakpoint(target,
				step_over_breakpoint);
			if (retval != ERROR_OK)
				return retval;
			breakpoint_removed = true;
		}
	}

	/* The official si_s path places temporary software breakpoints at both
	 * possible successors.  This avoids the LS2K300 match-any-fetch path,
	 * which can re-enter debug before the current instruction retires. */
	retval = loongarch_next_pc(target, pc, &next_pc);
	if (retval != ERROR_OK)
		goto cleanup;
	fallthrough_pc = pc + sizeof(uint32_t);

	retval = loongarch_add_step_breakpoint(target, next_pc,
		&target_breakpoint, target_original);
	if (retval != ERROR_OK)
		goto cleanup;
	if (fallthrough_pc != next_pc) {
		retval = loongarch_add_step_breakpoint(target, fallthrough_pc,
			&fallthrough_breakpoint, fallthrough_original);
		if (retval != ERROR_OK)
			goto cleanup;
	}

	retval = loongarch_resume(target, true, 0, false, false);
	if (retval != ERROR_OK)
		goto cleanup;
	target->debug_reason = DBG_REASON_SINGLESTEP;

	timeout = timeval_ms() + 2000;
	while (target->state != TARGET_HALTED) {
		retval = loongarch_poll(target);
		if (retval != ERROR_OK)
			goto cleanup;
		if (timeval_ms() >= timeout) {
			LOG_ERROR("loongarch: single-step timeout; target is still running");
			retval = ERROR_TARGET_TIMEOUT;
			goto cleanup;
		}
		keep_alive();
	}

cleanup:
	if (target->state == TARGET_HALTED && fallthrough_breakpoint.is_set) {
		int cleanup_retval = loongarch_remove_breakpoint(target,
			&fallthrough_breakpoint);
		if (retval == ERROR_OK)
			retval = cleanup_retval;
	}
	if (target->state == TARGET_HALTED && target_breakpoint.is_set) {
		int cleanup_retval = loongarch_remove_breakpoint(target,
			&target_breakpoint);
		if (retval == ERROR_OK)
			retval = cleanup_retval;
	}
	if (breakpoint_removed && target->state == TARGET_HALTED) {
		int restore_retval = loongarch_add_breakpoint(target,
			step_over_breakpoint);
		if (retval == ERROR_OK)
			retval = restore_retval;
	}
	return retval;
}

static int loongarch_assert_reset(struct target *target)
{
	struct loongarch_common *la = target_to_loongarch(target);
	int retval = adapter_assert_reset();
	if (retval != ERROR_OK)
		return retval;

	loongarch_invalidate_register_cache(la);
	target->state = TARGET_RESET;
	return ERROR_OK;
}

static int loongarch_deassert_reset(struct target *target)
{
	int retval = adapter_deassert_reset();
	if (retval != ERROR_OK)
		return retval;

	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

static int loongarch_refresh_register_cache(struct loongarch_common *la)
{
	uint64_t regs[LOONGARCH_CORE_REG_COUNT];

	int retval = loongarch_pracc_read_regs(&la->ejtag_info, regs,
					      ARRAY_SIZE(regs));
	if (retval != ERROR_OK)
		return retval;

	memcpy(la->reg_cache, regs, sizeof(regs));
	for (unsigned int i = 0; i < LOONGARCH_CORE_REG_COUNT; i++) {
		la->regs[i].valid = true;
		la->regs[i].dirty = false;
	}
	la->regs_valid = true;
	return ERROR_OK;
}

static int loongarch_commit_register_cache(struct loongarch_common *la)
{
	int retval = loongarch_pracc_write_regs(&la->ejtag_info,
					       la->reg_cache,
					       LOONGARCH_CORE_REG_COUNT);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < LOONGARCH_CORE_REG_COUNT; i++)
		la->regs[i].dirty = false;
	la->regs_valid = true;
	return ERROR_OK;
}

static int loongarch_fpu_enabled(struct loongarch_common *la, bool *enabled)
{
	uint64_t euen;

	if (!enabled)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	if (la->fpu_enabled_valid) {
		*enabled = la->fpu_enabled;
		return ERROR_OK;
	}
	int retval = loongarch_pracc_read_csr(&la->ejtag_info,
		LOONGARCH_CSR_EUEN, &euen);
	if (retval != ERROR_OK)
		return retval;

	la->fpu_enabled = euen & LOONGARCH_EUEN_FPE;
	la->fpu_enabled_valid = true;
	*enabled = la->fpu_enabled;
	return ERROR_OK;
}

static int loongarch_read_fcc_bundle(struct loongarch_common *la,
	uint64_t *value)
{
	uint64_t bundle = 0;

	if (!value)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	for (unsigned int i = 0; i < 8; i++) {
		uint64_t flag = 0;
		int retval = loongarch_pracc_read_fcc(&la->ejtag_info, i, &flag);
		if (retval != ERROR_OK)
			return retval;
		bundle |= (flag & 1) << i;
	}
	*value = bundle;
	return ERROR_OK;
}

static int loongarch_write_fcc_bundle(struct loongarch_common *la,
	uint64_t value)
{
	for (unsigned int i = 0; i < 8; i++) {
		int retval = loongarch_pracc_write_fcc(&la->ejtag_info, i,
			(value >> i) & 1);
		if (retval != ERROR_OK)
			return retval;
	}
	return ERROR_OK;
}

static int loongarch_get_register(struct reg *reg)
{
	struct loongarch_common *la = reg->arch_info;
	uint64_t value = 0;
	int retval;

	if (!la || !la->reg_cache || !la->regs)
		return ERROR_FAIL;

	/* PCSAMPLE observes the current dmseg fetch while the core is halted.
	 * The architected stop PC is DEPC, which is also what the official
	 * register module exports as register 32. */
	if (reg->number == LOONGARCH_REG_PC) {
		retval = loongarch_pracc_read_csr(&la->ejtag_info,
			LOONGARCH_CSR_DEPC, &value);
		if (retval != ERROR_OK)
			return retval;
	} else if (reg->number == LOONGARCH_REG_BADV) {
		retval = loongarch_pracc_read_csr(&la->ejtag_info,
			LOONGARCH_CSR_BADV, &value);
		if (retval != ERROR_OK)
			return retval;
	} else if (reg->number >= LOONGARCH_REG_F0) {
		bool enabled;

		retval = loongarch_fpu_enabled(la, &enabled);
		if (retval != ERROR_OK)
			return retval;
		if (enabled) {
			if (reg->number <= LOONGARCH_REG_F31)
				retval = loongarch_pracc_read_fpr(&la->ejtag_info,
					reg->number - LOONGARCH_REG_F0, &value);
			else if (reg->number == LOONGARCH_REG_FCC)
				retval = loongarch_read_fcc_bundle(la, &value);
			else
				retval = loongarch_pracc_read_fcsr(&la->ejtag_info, 0,
					&value);
			if (retval != ERROR_OK)
				return retval;
		}
	} else {
		if (!la->regs_valid || !reg->valid)
			return loongarch_refresh_register_cache(la);
		return ERROR_OK;
	}

	la->reg_cache[reg->number] = value;
	reg->valid = true;
	reg->dirty = false;
	return ERROR_OK;
}

static int loongarch_set_register(struct reg *reg, uint8_t *buf)
{
	struct loongarch_common *la = reg->arch_info;
	uint64_t value;
	int retval;

	if (!la || !la->reg_cache || !la->regs)
		return ERROR_FAIL;
	value = buf_get_u64(buf, 0, reg->size);
	if (reg->number <= LOONGARCH_REG_PC && !la->regs_valid) {
		retval = loongarch_refresh_register_cache(la);
		if (retval != ERROR_OK)
			return retval;
	}

	if (reg->number <= LOONGARCH_REG_PC) {
		la->reg_cache[reg->number] = value;
		retval = loongarch_commit_register_cache(la);
	} else if (reg->number == LOONGARCH_REG_BADV) {
		retval = loongarch_pracc_write_csr(&la->ejtag_info,
			LOONGARCH_CSR_BADV, value);
	} else {
		bool enabled;

		retval = loongarch_fpu_enabled(la, &enabled);
		if (retval == ERROR_OK && !enabled)
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		if (retval == ERROR_OK && reg->number <= LOONGARCH_REG_F31)
			retval = loongarch_pracc_write_fpr(&la->ejtag_info,
				reg->number - LOONGARCH_REG_F0, value);
		else if (retval == ERROR_OK && reg->number == LOONGARCH_REG_FCC)
			retval = loongarch_write_fcc_bundle(la, value);
		else if (retval == ERROR_OK)
			retval = loongarch_pracc_write_fcsr(&la->ejtag_info, 0,
				value & UINT32_MAX);
	}
	if (retval != ERROR_OK)
		return retval;

	la->reg_cache[reg->number] = value;
	reg->valid = true;
	reg->dirty = false;
	return ERROR_OK;
}

static const struct reg_arch_type loongarch_reg_type = {
	.get = loongarch_get_register,
	.set = loongarch_set_register,
};

static struct reg_feature loongarch_base_feature = {
	.name = "org.gnu.gdb.loongarch.base",
};

static struct reg_feature loongarch_fpu_feature = {
	.name = "org.gnu.gdb.loongarch.fpu",
};

static struct reg_data_type loongarch_uint64_type = {
	.type = REG_TYPE_UINT64,
};

static struct reg_data_type loongarch_uint32_type = {
	.type = REG_TYPE_UINT32,
};

static struct reg_data_type loongarch_ieee_single_type = {
	.type = REG_TYPE_IEEE_SINGLE,
	.id = "ieee_single",
};

static struct reg_data_type loongarch_ieee_double_type = {
	.type = REG_TYPE_IEEE_DOUBLE,
	.id = "ieee_double",
};

static struct reg_data_type_union_field loongarch_fpu_fields[] = {
	{ "f", &loongarch_ieee_single_type, loongarch_fpu_fields + 1 },
	{ "d", &loongarch_ieee_double_type, NULL },
};

static struct reg_data_type_union loongarch_fpu_union = {
	.fields = loongarch_fpu_fields,
};

static struct reg_data_type loongarch_fpu_type = {
	.type = REG_TYPE_ARCH_DEFINED,
	.id = "fputype",
	.type_class = REG_TYPE_CLASS_UNION,
	{ .reg_type_union = &loongarch_fpu_union },
};

static struct reg_data_type loongarch_code_pointer_type = {
	.type = REG_TYPE_CODE_PTR,
};

/*
 * Generic memory access through width-aware PrAcc modules.
 *
 * Aligned requests use the same access width as the OpenOCD caller.  This is
 * important for MMIO: forcing an ld.d/st.d pair for an 8-bit UART write can
 * read or acknowledge unrelated registers.  Unaligned requests retain the
 * 64-bit-window fallback below, which avoids issuing an architecturally
 * invalid unaligned target access.  Little-endian targets only (TODO:
 * byte-swap for BE).
 */
static int loongarch_mem_rw(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer, bool write_t)
{
	struct loongarch_common *la = target_to_loongarch(target);
	struct loongarch_ejtag *ej = &la->ejtag_info;
	uint64_t transfer_size;
	uint64_t start;
	uint64_t end;
	uint64_t cur;

	if (count == 0)
		return ERROR_OK;
	if (size == 0 || !buffer)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	transfer_size = (uint64_t)size * count;
	if (transfer_size / size != count || address > UINT64_MAX - transfer_size)
		return ERROR_COMMAND_ARGUMENT_INVALID;
	if (size != 1 && size != 2 && size != 4 && size != 8)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if ((address & (size - 1)) == 0) {
		uint32_t processed = 0;

		while (processed < count) {
			unsigned int words = count - processed;
			uint64_t values[LA_PRACC_MAX_WORDS] = { 0 };
			if (words > LA_PRACC_MAX_WORDS)
				words = LA_PRACC_MAX_WORDS;

			if (write_t) {
				for (unsigned int i = 0; i < words; i++)
					memcpy(&values[i], buffer +
						(size_t)(processed + i) * size, size);
				int retval = loongarch_pracc_write_mem_width(ej,
					address + (uint64_t)processed * size, size, words,
					values);
				if (retval != ERROR_OK)
					return retval;
			} else {
				int retval = loongarch_pracc_read_mem_width(ej,
					address + (uint64_t)processed * size, size, words,
					values);
				if (retval != ERROR_OK)
					return retval;
				for (unsigned int i = 0; i < words; i++)
					memcpy(buffer + (size_t)(processed + i) * size,
						&values[i], size);
			}
			processed += words;
		}
		return ERROR_OK;
	}

	start = address & ~7ull;
	end = address + transfer_size;
	cur = start;

	while (cur < end) {
		unsigned int words = 0;
		uint64_t windows[LA_PRACC_MAX_WORDS];
		uint64_t data[LA_PRACC_MAX_WORDS];

		for (; words < LA_PRACC_MAX_WORDS && cur + words * 8 < end; words++)
			windows[words] = cur + words * 8;

		int retval = loongarch_pracc_read_mem(ej, cur, words, data);
		if (retval != ERROR_OK)
			return retval;

		for (unsigned int w = 0; w < words; w++) {
			uint64_t wstart = windows[w];
			uint64_t wend = wstart + 8;
			uint64_t overlap_start = wstart > address ? wstart : address;
			uint64_t overlap_end = wend < end ? wend : end;
			size_t word_offset = (size_t)(overlap_start - wstart);
			size_t buffer_offset = (size_t)(overlap_start - address);
			size_t overlap_length = (size_t)(overlap_end - overlap_start);

			if (write_t) {
				memcpy((uint8_t *)&data[w] + word_offset,
				       buffer + buffer_offset, overlap_length);
			} else {
				memcpy(buffer + buffer_offset,
				       (uint8_t *)&data[w] + word_offset, overlap_length);
			}
		}

		if (write_t) {
			retval = loongarch_pracc_write_mem(ej, cur, words, data);
			if (retval != ERROR_OK)
				return retval;
		}

		cur += words * 8;
	}

	return ERROR_OK;
}

static int loongarch_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	return loongarch_mem_rw(target, address, size, count, buffer, false);
}

static int loongarch_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	return loongarch_mem_rw(target, address, size, count, (uint8_t *)buffer, true);
}

static int loongarch_build_reg_cache(struct loongarch_common *la)
{
	static const char *const gdb_names[] = {
		"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
		"r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
		"r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
		"r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31",
		"pc", "badv",
		"f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",
		"f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",
		"f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
		"f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
		"fcc", "fcsr",
	};
	unsigned int count = ARRAY_SIZE(gdb_names);
	if (count != LOONGARCH_REG_COUNT)
		return ERROR_FAIL;

	la->reg_count = count;
	la->reg_cache = calloc(count, sizeof(uint64_t));
	la->regs = calloc(count, sizeof(struct reg));
	if (!la->reg_cache || !la->regs)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < count; i++) {
		la->regs[i].name = gdb_names[i];
		la->regs[i].value = (uint8_t *)&la->reg_cache[i];
		la->regs[i].size = i == LOONGARCH_REG_FCSR ? 32 : 64;
		la->regs[i].feature = i >= LOONGARCH_REG_F0 ?
			&loongarch_fpu_feature : &loongarch_base_feature;
		la->regs[i].group = i >= LOONGARCH_REG_F0 ? "float" : "general";
		if (i == LOONGARCH_REG_PC)
			la->regs[i].reg_data_type = &loongarch_code_pointer_type;
		else if (i >= LOONGARCH_REG_F0 && i <= LOONGARCH_REG_FCC)
			la->regs[i].reg_data_type = &loongarch_fpu_type;
		else if (i == LOONGARCH_REG_FCSR)
			la->regs[i].reg_data_type = &loongarch_uint32_type;
		else
			la->regs[i].reg_data_type = &loongarch_uint64_type;
		la->regs[i].arch_info = la;
		la->regs[i].type = &loongarch_reg_type;
		la->regs[i].number = i;
		la->regs[i].exist = true;
		la->regs[i].valid = false;
		la->regs[i].dirty = false;
	}
	return ERROR_OK;
}

static int loongarch_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class)
{
	struct loongarch_common *la = target_to_loongarch(target);
	struct reg_cache *cache;

	if (la->regs == NULL) {
		int retval = loongarch_build_reg_cache(la);
		if (retval != ERROR_OK)
			return retval;
	}

	if (target->reg_cache == NULL) {
		cache = calloc(1, sizeof(*cache));
		if (!cache)
			return ERROR_FAIL;
		cache->name = "loongarch";
		cache->reg_list = la->regs;
		cache->num_regs = la->reg_count;
		target->reg_cache = cache;
	}

	*reg_list = malloc(sizeof(struct reg *) * la->reg_count);
	if (!*reg_list)
		return ERROR_FAIL;
	for (unsigned int i = 0; i < la->reg_count; i++)
		(*reg_list)[i] = &la->regs[i];
	*reg_list_size = la->reg_count;
	(void)reg_class;
	return ERROR_OK;
}

static int loongarch_add_hardware_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint64_t address_mask)
{
	struct loongarch_common *la = target_to_loongarch(target);
	uint64_t max_breakpoints;
	uint64_t control;
	unsigned int slot;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	if (breakpoint->is_set)
		return ERROR_OK;
	if (breakpoint->type != BKPT_HARD)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if (!la->max_inst_breakpoints_valid) {
		retval = loongarch_pracc_read_csr(&la->ejtag_info, 0x380,
			&max_breakpoints);
		if (retval != ERROR_OK)
			return retval;
		la->max_inst_breakpoints = max_breakpoints & 0x3F;
		if (la->max_inst_breakpoints > 32)
			la->max_inst_breakpoints = 32;
		la->max_inst_breakpoints_valid = true;
	}

	if (la->max_inst_breakpoints == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	for (slot = 0; slot < la->max_inst_breakpoints; slot++) {
		if (!(la->inst_breakpoint_mask & (1u << slot)))
			break;
	}
	if (slot == la->max_inst_breakpoints)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = loongarch_pracc_write_csr(&la->ejtag_info,
		0x390 + slot * 8, breakpoint->address);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_pracc_write_csr(&la->ejtag_info,
		0x391 + slot * 8, address_mask);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_pracc_write_csr(&la->ejtag_info,
		0x393 + slot * 8, breakpoint->asid);
	if (retval != ERROR_OK)
		return retval;
	control = breakpoint->asid ? 0x9F : 0x1F;
	retval = loongarch_pracc_write_csr(&la->ejtag_info,
		0x392 + slot * 8, control);
	if (retval != ERROR_OK)
		return retval;

	la->inst_breakpoint_mask |= 1u << slot;
	breakpoint_hw_set(breakpoint, slot);
	return ERROR_OK;
}

static int loongarch_add_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	if (breakpoint->is_set)
		return ERROR_OK;

	if (breakpoint->type == BKPT_SOFT) {
		uint32_t verify;

		if (breakpoint->length != sizeof(uint32_t) ||
			(breakpoint->address & (sizeof(uint32_t) - 1)) != 0)
			return ERROR_COMMAND_ARGUMENT_INVALID;

		retval = target_read_memory(target, breakpoint->address,
			sizeof(uint32_t), 1, breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, breakpoint->address,
			LOONGARCH_SOFTWARE_BREAK_INSTRUCTION);
		if (retval != ERROR_OK)
			return retval;

		retval = target_read_u32(target, breakpoint->address, &verify);
		if (retval != ERROR_OK) {
			target_write_memory(target, breakpoint->address,
				sizeof(uint32_t), 1, breakpoint->orig_instr);
			return retval;
		}
		if (verify != LOONGARCH_SOFTWARE_BREAK_INSTRUCTION) {
			LOG_TARGET_ERROR(target,
				"unable to verify software breakpoint at 0x%" TARGET_PRIxADDR,
				breakpoint->address);
			target_write_memory(target, breakpoint->address,
				sizeof(uint32_t), 1, breakpoint->orig_instr);
			return ERROR_FAIL;
		}

		breakpoint->is_set = true;
		return ERROR_OK;
	}

	/* Exact matching uses a zero address mask.  The official client reserves
	 * an all-ones mask for its match-any-fetch single-step breakpoint.  It
	 * adds bit 7 only when ASID comparison is requested; a normal address
	 * breakpoint uses 0x1f. */
	return loongarch_add_hardware_breakpoint(target, breakpoint, 0);
}

static int loongarch_remove_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct loongarch_common *la = target_to_loongarch(target);
	int retval;

	if (!breakpoint->is_set)
		return ERROR_OK;

	if (breakpoint->type == BKPT_SOFT) {
		if (breakpoint->length != sizeof(uint32_t) ||
			(breakpoint->address & (sizeof(uint32_t) - 1)) != 0)
			return ERROR_COMMAND_ARGUMENT_INVALID;

		retval = target_write_memory(target, breakpoint->address,
			sizeof(uint32_t), 1, breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;

		breakpoint->is_set = false;
		return ERROR_OK;
	}

	if (breakpoint->type != BKPT_HARD)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	unsigned int slot = breakpoint->number;
	if (slot >= 32 || !(la->inst_breakpoint_mask & (1u << slot)))
		return ERROR_FAIL;

	retval = loongarch_pracc_write_csr(&la->ejtag_info,
		0x390 + slot * 8, 0);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_pracc_write_csr(&la->ejtag_info,
		0x392 + slot * 8, 0);
	if (retval != ERROR_OK)
		return retval;

	la->inst_breakpoint_mask &= ~(1u << slot);
	breakpoint->is_set = false;
	return ERROR_OK;
}

static int loongarch_watchpoint_control(const struct watchpoint *watchpoint,
	uint64_t *control)
{
	switch (watchpoint->length) {
	case 1:
		*control = 3u << 10;
		break;
	case 2:
		*control = 2u << 10;
		break;
	case 4:
		*control = 1u << 10;
		break;
	case 8:
		*control = 0;
		break;
	default:
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	switch (watchpoint->rw) {
	case WPT_READ:
		*control |= 1u << 8;
		break;
	case WPT_WRITE:
		*control |= 1u << 9;
		break;
	case WPT_ACCESS:
		*control |= 3u << 8;
		break;
	default:
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	*control |= 0x1F; /* enable plus PLV0..PLV3; DMOnly stays clear. */

	return ERROR_OK;
}

static int loongarch_add_watchpoint(struct target *target,
	struct watchpoint *watchpoint)
{
	struct loongarch_common *la = target_to_loongarch(target);
	uint64_t max_watchpoints;
	uint64_t control;
	uint32_t base;
	unsigned int slot;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = loongarch_watchpoint_control(watchpoint, &control);
	if (retval != ERROR_OK)
		return retval;

	if (!la->max_data_breakpoints_valid) {
		retval = loongarch_pracc_read_csr(&la->ejtag_info, 0x300,
			&max_watchpoints);
		if (retval != ERROR_OK)
			return retval;
		la->max_data_breakpoints = max_watchpoints & 0x3F;
		if (la->max_data_breakpoints > 32)
			la->max_data_breakpoints = 32;
		la->max_data_breakpoints_valid = true;
	}

	if (la->max_data_breakpoints == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	for (slot = 0; slot < la->max_data_breakpoints; slot++) {
		if (!(la->data_breakpoint_mask & (1u << slot)))
			break;
	}
	if (slot == la->max_data_breakpoints)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* MWPC/MWPnCFG1-4 are laid out like FWPC/FWPnCFG1-4.  The
	 * address-mask and ASID comparison remain caller-independent, matching
	 * the official debugger's default data-breakpoint setup. */
	base = 0x310 + slot * 8;
	retval = loongarch_pracc_write_csr(&la->ejtag_info, base,
		watchpoint->address);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_pracc_write_csr(&la->ejtag_info, base + 1,
		0);
	if (retval != ERROR_OK)
		return retval;
	/* The official client leaves the compare-data slot at zero and uses
	 * these units as address watchpoints.  Accept OpenOCD's value argument
	 * for compatibility, but do not invent a compare-data control mode that
	 * the documented 2K300 flow does not enable. */
	retval = loongarch_pracc_write_csr(&la->ejtag_info, base + 3, 0);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_pracc_write_csr(&la->ejtag_info, base + 2, control);
	if (retval != ERROR_OK)
		return retval;

	la->data_breakpoint_mask |= 1u << slot;
	watchpoint_set(watchpoint, slot);
	return ERROR_OK;
}

static int loongarch_remove_watchpoint(struct target *target,
	struct watchpoint *watchpoint)
{
	struct loongarch_common *la = target_to_loongarch(target);
	unsigned int slot = watchpoint->number;
	int retval;

	if (!watchpoint->is_set)
		return ERROR_OK;
	if (slot >= 32 || !(la->data_breakpoint_mask & (1u << slot)))
		return ERROR_WATCHPOINT_NOT_FOUND;

	retval = loongarch_pracc_write_csr(&la->ejtag_info,
		0x310 + slot * 8 + 2, 0);
	if (retval != ERROR_OK)
		return retval;

	la->data_breakpoint_mask &= ~(1u << slot);
	watchpoint->is_set = false;
	return ERROR_OK;
}

static int loongarch_hit_watchpoint(struct target *target,
	struct watchpoint **hit_watchpoint)
{
	struct loongarch_common *la = target_to_loongarch(target);
	uint64_t debug;
	uint64_t status;
	uint32_t hit_mask;
	int retval;

	if (!hit_watchpoint)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	retval = loongarch_pracc_read_csr(&la->ejtag_info, 0x500, &debug);
	if (retval != ERROR_OK)
		return retval;
	if (!(debug & (1u << 11))) /* DBG.DMW */
		return ERROR_FAIL;

	retval = loongarch_pracc_read_csr(&la->ejtag_info, 0x301, &status);
	if (retval != ERROR_OK)
		return retval;
	hit_mask = (uint32_t)status & la->data_breakpoint_mask;
	if (!hit_mask)
		return ERROR_FAIL;

	for (struct watchpoint *watchpoint = target->watchpoints;
		watchpoint; watchpoint = watchpoint->next) {
		if (watchpoint->is_set && (hit_mask & (1u << watchpoint->number))) {
			retval = loongarch_pracc_write_csr(&la->ejtag_info, 0x301,
				LOONGARCH_BREAKPOINT_STATUS_MASK);
			if (retval != ERROR_OK)
				return retval;
			target->debug_reason = DBG_REASON_WATCHPOINT;
			*hit_watchpoint = watchpoint;
			return ERROR_OK;
		}
	}

	return ERROR_WATCHPOINT_NOT_FOUND;
}

static int loongarch_examine(struct target *target)
{
	struct loongarch_common *la = target_to_loongarch(target);
	struct loongarch_ejtag *ej = &la->ejtag_info;

	if (target->tap == NULL) {
		LOG_ERROR("loongarch: target has no JTAG tap");
		return ERROR_FAIL;
	}

	ej->tap = target->tap;
	int retval = loongarch_ejtag_init(ej);
	if (retval != ERROR_OK)
		return retval;

	if (la->regs == NULL) {
		retval = loongarch_build_reg_cache(la);
		if (retval != ERROR_OK)
			return retval;
	}

	if (!target->reg_cache) {
		target->reg_cache = calloc(1, sizeof(*target->reg_cache));
		if (!target->reg_cache)
			return ERROR_FAIL;
		target->reg_cache->name = "loongarch";
		target->reg_cache->reg_list = la->regs;
		target->reg_cache->num_regs = la->reg_count;
	}

	target->state = TARGET_UNKNOWN;
	return ERROR_OK;
}

static int loongarch_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	(void)cmd_ctx;
	(void)target;
	return ERROR_OK;
}

static int loongarch_target_create(struct target *target)
{
	struct loongarch_common *la = calloc(1, sizeof(*la));
	if (!la)
		return ERROR_FAIL;

	/* Loongson 2K300 IR map measured on real hardware. */
	la->ejtag_info.ir_len = 4;
	la->ejtag_info.ir_impcode = 1;	/* 0x5a5a5a5a marker register */
	la->ejtag_info.ir_idcode = 2;	/* 0x60400001 on LS2K300 */
	la->ejtag_info.ir_address = 3;
	la->ejtag_info.ir_data = 4;
	la->ejtag_info.ir_control = 5;
	la->ejtag_info.ir_pcsample = 6;
	la->ejtag_info.ir_all = 7;	/* 160-bit CONTROL + DATA + ADDRESS */
	la->ejtag_info.ir_fastdata = 8;	/* 65-bit SPrAcc + DATA */
	la->ejtag_info.ir_bypass = 0;
	la->ejtag_info.use_all = true;	/* official LS2K300 default */
	la->ejtag_info.use_fastdata = false;	/* official LS2K300 default */

	target->arch_info = la;
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	struct loongarch_ejtag *ej = &la->ejtag_info;

	command_print(CMD, "loongarch: ir_len=%u idcode=0x%" PRIx32
			  " address=0x%" PRIx32
			  " data=0x%" PRIx32 " control=0x%" PRIx32
			  " pcsample=0x%" PRIx32 " all=0x%" PRIx32
			  " fastdata=0x%" PRIx32
			  " bypass=0x%" PRIx32,
			  ej->ir_len, ej->ir_idcode, ej->ir_address,
			  ej->ir_data, ej->ir_control, ej->ir_pcsample,
			  ej->ir_all, ej->ir_fastdata, ej->ir_bypass);
	command_print(CMD, "loongarch: impcode=0x%08" PRIx32, ej->impcode);
	command_print(CMD, "loongarch: idcode=0x%08" PRIx32, ej->idcode);
	command_print(CMD, "loongarch: use_all=%s", ej->use_all ? "on" : "off");
	command_print(CMD, "loongarch: use_fastdata=%s",
		ej->use_fastdata ? "on" : "off");
	command_print(CMD, "loongarch: probe_vector=%s",
		      (ej->ejtag_ctrl & LA_CTRL_PROBEVEC) ? "dmseg" : "bios");
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_vector_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	struct loongarch_ejtag *ej = &la->ejtag_info;
	bool dmseg;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!strcmp(CMD_ARGV[0], "dmseg"))
		dmseg = true;
	else if (!strcmp(CMD_ARGV[0], "bios"))
		dmseg = false;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	loongarch_ejtag_set_probe_vector(ej, dmseg);
	command_print(CMD, "loongarch: probe vector set to %s",
		      dmseg ? "dmseg" : "bios");
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_ir_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	struct loongarch_ejtag *ej = &la->ejtag_info;
	uint32_t code;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], code);
	if (!strcmp(CMD_ARGV[0], "idcode"))
		ej->ir_idcode = code;
	else if (!strcmp(CMD_ARGV[0], "address"))
		ej->ir_address = code;
	else if (!strcmp(CMD_ARGV[0], "data"))
		ej->ir_data = code;
	else if (!strcmp(CMD_ARGV[0], "control"))
		ej->ir_control = code;
	else if (!strcmp(CMD_ARGV[0], "pcsample"))
		ej->ir_pcsample = code;
	else if (!strcmp(CMD_ARGV[0], "all"))
		ej->ir_all = code;
	else if (!strcmp(CMD_ARGV[0], "fastdata"))
		ej->ir_fastdata = code;
	else if (!strcmp(CMD_ARGV[0], "bypass"))
		ej->ir_bypass = code;
	else if (!strcmp(CMD_ARGV[0], "impcode"))
		ej->ir_impcode = code;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_idcode_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	int retval;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = loongarch_ejtag_get_idcode(&la->ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%08" PRIx32, la->ejtag_info.idcode);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_all_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	struct loongarch_ejtag_all all = {
		.control = la->ejtag_info.ejtag_ctrl,
		.data = 0,
		.address = 0,
	};
	int retval;

	if (CMD_ARGC != 0 && CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (target->state == TARGET_HALTED)
		LOG_WARNING("raw ALL scan captures but does not service the pending PrAcc");

	if (CMD_ARGC == 3) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], all.control);
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], all.data);
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[2], all.address);
	}

	retval = loongarch_ejtag_scan_all(&la->ejtag_info, &all);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "control=0x%08" PRIx32
		" data=0x%016" PRIx64 " address=0x%016" PRIx64,
		all.control, all.data, all.address);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_use_all_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);

	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1) {
		if (!strcmp(CMD_ARGV[0], "on"))
			la->ejtag_info.use_all = true;
		else if (!strcmp(CMD_ARGV[0], "off"))
			la->ejtag_info.use_all = false;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "%s", la->ejtag_info.use_all ? "on" : "off");
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_fastdata_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	struct loongarch_ejtag_fastdata fastdata = {
		.spracc = false,
		.data = 0,
	};

	if (CMD_ARGC != 0 && CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 2) {
		uint32_t spracc;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], spracc);
		if (spracc > 1)
			return ERROR_COMMAND_ARGUMENT_INVALID;
		fastdata.spracc = spracc != 0;
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], fastdata.data);
	}

	int retval = loongarch_ejtag_scan_fastdata(&la->ejtag_info, &fastdata);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "spracc=%u data=0x%016" PRIx64,
		fastdata.spracc ? 1 : 0, fastdata.data);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_use_fastdata_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);

	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1) {
		if (!strcmp(CMD_ARGV[0], "on"))
			la->ejtag_info.use_fastdata = true;
		else if (!strcmp(CMD_ARGV[0], "off"))
			la->ejtag_info.use_fastdata = false;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "%s", la->ejtag_info.use_fastdata ? "on" : "off");
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_fastdata_bulk_read_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint64_t handler_addr;
	uint64_t address;
	uint32_t count;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], handler_addr);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], count);
	if (count == 0 || count > 256)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	uint64_t *values = calloc(count, sizeof(*values));
	if (!values)
		return ERROR_FAIL;
	int retval = loongarch_pracc_fastdata_read64(&la->ejtag_info,
		handler_addr, address, count, values);
	if (retval == ERROR_OK) {
		for (unsigned int i = 0; i < count; i++)
			command_print(CMD, "0x%016" PRIx64 ": 0x%016" PRIx64,
				address + (uint64_t)i * 8, values[i]);
	}
	free(values);
	return retval;
}

COMMAND_HANDLER(loongarch_handle_fastdata_bulk_write_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint64_t handler_addr;
	uint64_t address;
	uint64_t values[256];

	if (CMD_ARGC < 3 || CMD_ARGC > 258)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], handler_addr);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], address);
	unsigned int count = CMD_ARGC - 2;
	for (unsigned int i = 0; i < count; i++)
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[i + 2], values[i]);

	return loongarch_pracc_fastdata_write64(&la->ejtag_info,
		handler_addr, address, count, values);
}

COMMAND_HANDLER(loongarch_handle_csr_read_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t csr;
	uint64_t value;
	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], csr);
	retval = loongarch_pracc_read_csr(&la->ejtag_info, csr, &value);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%016" PRIx64, value);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_csr_write_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t csr;
	uint64_t value;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], csr);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], value);
	int retval = loongarch_pracc_write_csr(&la->ejtag_info, csr, value);
	if (retval == ERROR_OK && csr == LOONGARCH_CSR_EUEN) {
		la->fpu_enabled = value & LOONGARCH_EUEN_FPE;
		la->fpu_enabled_valid = true;
	}
	return retval;
}

COMMAND_HANDLER(loongarch_handle_cpucfg_read_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t index;
	uint64_t value;
	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], index);
	retval = loongarch_pracc_read_cpucfg(&la->ejtag_info, index, &value);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%016" PRIx64, value);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_iocsr_read_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint64_t address;
	uint64_t value;
	uint32_t size;
	int retval;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);
	retval = loongarch_pracc_read_iocsr(&la->ejtag_info, address, size, &value);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%016" PRIx64, value);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_iocsr_write_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint64_t address;
	uint64_t value;
	uint32_t size;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[2], value);
	return loongarch_pracc_write_iocsr(&la->ejtag_info, address, size, value);
}

static int loongarch_compare_pcsample(const void *left, const void *right)
{
	uint64_t left_pc = *(const uint64_t *)left;
	uint64_t right_pc = *(const uint64_t *)right;

	return (left_pc > right_pc) - (left_pc < right_pc);
}

static int loongarch_collect_pcsamples(struct loongarch_ejtag *ej,
	uint64_t *samples, uint32_t capacity, uint32_t duration_ms,
	uint32_t interval_ms, uint32_t *sample_count)
{
	int64_t deadline = timeval_ms() + duration_ms;
	uint32_t count = 0;

	do {
		int retval = loongarch_ejtag_read_pcsample(ej, &samples[count]);
		if (retval != ERROR_OK)
			return retval;
		count++;

		if (count >= capacity || timeval_ms() >= deadline)
			break;
		if (interval_ms)
			alive_sleep(interval_ms);
	} while (timeval_ms() < deadline);

	*sample_count = count;
	return ERROR_OK;
}

static void loongarch_warn_halted_pcsample(struct target *target)
{
	if (target->state == TARGET_HALTED)
		LOG_TARGET_WARNING(target, "PCSAMPLE reports the current dmseg debug PC; "
			"use 'reg pc' for the architectural resume PC");
}

COMMAND_HANDLER(loongarch_handle_pcsample_read_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t count = 1;
	uint32_t interval_ms = 0;

	if (CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC >= 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], count);
	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], interval_ms);
	if (count == 0 || count > LOONGARCH_PCSAMPLE_MAX_SAMPLES ||
		interval_ms > LOONGARCH_PCSAMPLE_MAX_INTERVAL_MS)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	loongarch_warn_halted_pcsample(target);
	for (uint32_t i = 0; i < count; i++) {
		uint64_t pc;
		int retval = loongarch_ejtag_read_pcsample(&la->ejtag_info, &pc);
		if (retval != ERROR_OK)
			return retval;

		if (count == 1)
			command_print(CMD, "0x%016" PRIx64, pc);
		else
			command_print(CMD, "%" PRIu32 " 0x%016" PRIx64, i, pc);
		if (interval_ms && i + 1 < count)
			alive_sleep(interval_ms);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_pcsample_profile_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t duration_ms;
	uint32_t interval_ms = 0;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], duration_ms);
	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], interval_ms);
	if (duration_ms == 0 || duration_ms > LOONGARCH_PCSAMPLE_MAX_DURATION_MS ||
		interval_ms > LOONGARCH_PCSAMPLE_MAX_INTERVAL_MS)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	uint64_t *samples = malloc(sizeof(*samples) * LOONGARCH_PCSAMPLE_MAX_SAMPLES);
	if (!samples)
		return ERROR_FAIL;

	loongarch_warn_halted_pcsample(target);
	int64_t started = timeval_ms();
	uint32_t sample_count = 0;
	int retval = loongarch_collect_pcsamples(&la->ejtag_info, samples,
		LOONGARCH_PCSAMPLE_MAX_SAMPLES, duration_ms, interval_ms,
		&sample_count);
	int64_t elapsed_ms = timeval_ms() - started;
	if (retval != ERROR_OK) {
		free(samples);
		return retval;
	}

	qsort(samples, sample_count, sizeof(*samples), loongarch_compare_pcsample);
	command_print(CMD, "samples=%" PRIu32 " elapsed_ms=%" PRId64
		" rate=%.2f/s%s", sample_count, elapsed_ms,
		(double)sample_count * 1000.0 / (elapsed_ms > 0 ? elapsed_ms : 1),
		sample_count == LOONGARCH_PCSAMPLE_MAX_SAMPLES ? " capped" : "");

	for (uint32_t first = 0; first < sample_count;) {
		uint32_t next = first + 1;
		while (next < sample_count && samples[next] == samples[first])
			next++;
		uint32_t occurrences = next - first;
		command_print(CMD, "0x%016" PRIx64 " %" PRIu32 " %.2f%%",
			samples[first], occurrences,
			(double)occurrences * 100.0 / sample_count);
		first = next;
	}

	free(samples);
	return ERROR_OK;
}

static int loongarch_require_fpu(struct target *target,
	struct loongarch_common *la)
{
	bool enabled;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	int retval = loongarch_fpu_enabled(la, &enabled);
	if (retval != ERROR_OK)
		return retval;
	if (!enabled) {
		LOG_TARGET_ERROR(target, "floating-point unit is disabled in EUEN");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_fpr_read_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t reg;
	uint64_t value = 0;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg);

	int retval = loongarch_require_fpu(target, la);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_pracc_read_fpr(&la->ejtag_info, reg, &value);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%016" PRIx64, value);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_fpr_write_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t reg;
	uint64_t value;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], value);

	int retval = loongarch_require_fpu(target, la);
	if (retval != ERROR_OK)
		return retval;
	return loongarch_pracc_write_fpr(&la->ejtag_info, reg, value);
}

COMMAND_HANDLER(loongarch_handle_fcsr_read_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t reg;
	uint64_t value = 0;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg);

	int retval = loongarch_require_fpu(target, la);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_pracc_read_fcsr(&la->ejtag_info, reg, &value);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%08" PRIx64, value & UINT32_MAX);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_fcsr_write_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t reg;
	uint64_t value;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], value);
	if (value > UINT32_MAX)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	int retval = loongarch_require_fpu(target, la);
	if (retval != ERROR_OK)
		return retval;
	return loongarch_pracc_write_fcsr(&la->ejtag_info, reg, value);
}

COMMAND_HANDLER(loongarch_handle_fcc_read_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t reg;
	uint64_t value = 0;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg);

	int retval = loongarch_require_fpu(target, la);
	if (retval != ERROR_OK)
		return retval;
	retval = loongarch_pracc_read_fcc(&la->ejtag_info, reg, &value);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%01" PRIx64, value & 1);
	return ERROR_OK;
}

COMMAND_HANDLER(loongarch_handle_fcc_write_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct loongarch_common *la = target_to_loongarch(target);
	uint32_t reg;
	uint64_t value;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], value);

	int retval = loongarch_require_fpu(target, la);
	if (retval != ERROR_OK)
		return retval;
	return loongarch_pracc_write_fcc(&la->ejtag_info, reg, value);
}

static const struct command_registration loongarch_exec_command_handlers[] = {
	{
		.name = "info",
		.handler = loongarch_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "show loongarch EJTAG configuration",
	},
	{
		.name = "vector",
		.handler = loongarch_handle_vector_command,
		.mode = COMMAND_EXEC,
		.usage = "<dmseg|bios>",
		.help = "select the EJTAG debug exception vector",
	},
	{
		.name = "ir",
		.handler = loongarch_handle_ir_command,
		.mode = COMMAND_EXEC,
		.usage = "<idcode|address|data|control|pcsample|all|fastdata|bypass|impcode> <code>",
		.help = "set a LoongArch EJTAG IR code",
	},
	{
		.name = "idcode",
		.handler = loongarch_handle_idcode_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "read the 32-bit LoongArch EJTAG IDCODE register",
	},
	{
		.name = "all",
		.handler = loongarch_handle_all_command,
		.mode = COMMAND_EXEC,
		.usage = "[<control> <data> <address>]",
		.help = "raw-scan the 160-bit CONTROL + DATA + ADDRESS register",
	},
	{
		.name = "use_all",
		.handler = loongarch_handle_use_all_command,
		.mode = COMMAND_EXEC,
		.usage = "[on|off]",
		.help = "select IR=ALL or separate-register PrAcc polling",
	},
	{
		.name = "fastdata",
		.handler = loongarch_handle_fastdata_command,
		.mode = COMMAND_EXEC,
		.usage = "[<spracc-out:0|1> <data-out>]",
		.help = "raw-scan the 65-bit SPrAcc + DATA register",
	},
	{
		.name = "use_fastdata",
		.handler = loongarch_handle_use_fastdata_command,
		.mode = COMMAND_EXEC,
		.usage = "[on|off]",
		.help = "complete mailbox accesses through IR=FASTDATA when available",
	},
	{
		.name = "fastdata_bulk_read",
		.handler = loongarch_handle_fastdata_bulk_read_command,
		.mode = COMMAND_EXEC,
		.usage = "<handler_addr> <address> <64-bit count>",
		.help = "diagnose continuous IR=FASTDATA reads using a temporary RAM handler",
	},
	{
		.name = "fastdata_bulk_write",
		.handler = loongarch_handle_fastdata_bulk_write_command,
		.mode = COMMAND_EXEC,
		.usage = "<handler_addr> <address> <value> [value ...]",
		.help = "diagnose continuous IR=FASTDATA writes using a temporary RAM handler",
	},
	{
		.name = "csr_read",
		.handler = loongarch_handle_csr_read_command,
		.mode = COMMAND_EXEC,
		.usage = "<csr>",
		.help = "read a LoongArch control and status register",
	},
	{
		.name = "csr_write",
		.handler = loongarch_handle_csr_write_command,
		.mode = COMMAND_EXEC,
		.usage = "<csr> <value>",
		.help = "write a LoongArch control and status register",
	},
	{
		.name = "cpucfg_read",
		.handler = loongarch_handle_cpucfg_read_command,
		.mode = COMMAND_EXEC,
		.usage = "<index>",
		.help = "read a LoongArch CPUCFG word",
	},
	{
		.name = "iocsr_read",
		.handler = loongarch_handle_iocsr_read_command,
		.mode = COMMAND_EXEC,
		.usage = "<address> <1|2|4|8>",
		.help = "read a LoongArch IOCSR register",
	},
	{
		.name = "iocsr_write",
		.handler = loongarch_handle_iocsr_write_command,
		.mode = COMMAND_EXEC,
		.usage = "<address> <1|2|4|8> <value>",
		.help = "write a LoongArch IOCSR register",
	},
	{
		.name = "pcsample_read",
		.handler = loongarch_handle_pcsample_read_command,
		.mode = COMMAND_EXEC,
		.usage = "[<count> [<interval_ms>]]",
		.help = "read one or more full-width PC samples without halting the target",
	},
	{
		.name = "pcsample_profile",
		.handler = loongarch_handle_pcsample_profile_command,
		.mode = COMMAND_EXEC,
		.usage = "<duration_ms> [<interval_ms>]",
		.help = "collect and summarize full-width IR=PCSAMPLE values without halting the target",
	},
	{
		.name = "fpr_read",
		.handler = loongarch_handle_fpr_read_command,
		.mode = COMMAND_EXEC,
		.usage = "<0..31>",
		.help = "read a LoongArch floating-point register",
	},
	{
		.name = "fpr_write",
		.handler = loongarch_handle_fpr_write_command,
		.mode = COMMAND_EXEC,
		.usage = "<0..31> <value>",
		.help = "write a LoongArch floating-point register",
	},
	{
		.name = "fcsr_read",
		.handler = loongarch_handle_fcsr_read_command,
		.mode = COMMAND_EXEC,
		.usage = "<0..3>",
		.help = "read a LoongArch floating-point control/status register",
	},
	{
		.name = "fcsr_write",
		.handler = loongarch_handle_fcsr_write_command,
		.mode = COMMAND_EXEC,
		.usage = "<0..3> <value>",
		.help = "write a LoongArch floating-point control/status register",
	},
	{
		.name = "fcc_read",
		.handler = loongarch_handle_fcc_read_command,
		.mode = COMMAND_EXEC,
		.usage = "<0..7>",
		.help = "read a LoongArch floating-point condition flag",
	},
	{
		.name = "fcc_write",
		.handler = loongarch_handle_fcc_write_command,
		.mode = COMMAND_EXEC,
		.usage = "<0..7> <0|1>",
		.help = "write a LoongArch floating-point condition flag",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration loongarch_command_handlers[] = {
	{
		.name = "loongarch",
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "loongarch target commands",
		.chain = loongarch_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type loongarch_target = {
	.name = "loongarch",

	.poll = loongarch_poll,
	.arch_state = loongarch_arch_state,

	.halt = loongarch_halt,
	.resume = loongarch_resume,
	.step = loongarch_step,

	.assert_reset = loongarch_assert_reset,
	.deassert_reset = loongarch_deassert_reset,

	.get_gdb_arch = loongarch_get_gdb_arch,
	.get_gdb_reg_list = loongarch_get_gdb_reg_list,

	.read_memory = loongarch_read_memory,
	.write_memory = loongarch_write_memory,

	.add_breakpoint = loongarch_add_breakpoint,
	.remove_breakpoint = loongarch_remove_breakpoint,
	.add_watchpoint = loongarch_add_watchpoint,
	.remove_watchpoint = loongarch_remove_watchpoint,
	.hit_watchpoint = loongarch_hit_watchpoint,

	.commands = loongarch_command_handlers,
	.target_create = loongarch_target_create,
	.init_target = loongarch_init_target,
	.examine = loongarch_examine,

	.address_bits = loongarch_target_width,
	.data_bits = loongarch_target_width,
};
