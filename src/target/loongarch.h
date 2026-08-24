/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_LOONGARCH_H
#define OPENOCD_TARGET_LOONGARCH_H

#include <target/target.h>
#include <target/register.h>
#include "loongarch_ejtag.h"

struct loongarch_common {
	struct loongarch_ejtag ejtag_info;
	uint64_t *reg_cache;
	struct reg *regs;
	unsigned int reg_count;
	bool regs_valid;
	bool fpu_enabled;
	bool fpu_enabled_valid;
	uint32_t inst_breakpoint_mask;
	uint32_t max_inst_breakpoints;
	bool max_inst_breakpoints_valid;
	uint32_t data_breakpoint_mask;
	uint32_t max_data_breakpoints;
	bool max_data_breakpoints_valid;
};

#endif /* OPENOCD_TARGET_LOONGARCH_H */
