/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2024 Rivos, Inc
 */

#ifndef _ASM_VENDOR_EXTENSIONS_H
#define _ASM_VENDOR_EXTENSIONS_H

#include <asm/cpufeature.h>

#include <linux/array_size.h>
#include <linux/types.h>

/*
 * The extension keys of each vendor must be strictly less than this value.
 */
#define RISCV_ISA_VENDOR_EXT_MAX 32

struct riscv_isavendorinfo {
	DECLARE_BITMAP(isa, RISCV_ISA_VENDOR_EXT_MAX);
};

struct riscv_isa_vendor_ext_data_list {
	const size_t ext_data_count;
	const struct riscv_isa_ext_data *ext_data;
	struct riscv_isavendorinfo per_hart_isa_bitmap[NR_CPUS];
	struct riscv_isavendorinfo all_harts_isa_bitmap;
};

extern struct riscv_isa_vendor_ext_data_list *riscv_isa_vendor_ext_list[];

extern const size_t riscv_isa_vendor_ext_list_size;

#endif /* _ASM_VENDOR_EXTENSIONS_H */
