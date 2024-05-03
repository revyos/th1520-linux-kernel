// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 Rivos, Inc
 */

#include <asm/vendor_extensions.h>
#include <asm/vendor_extensions/thead.h>

#include <linux/array_size.h>
#include <linux/types.h>

struct riscv_isa_vendor_ext_data_list *riscv_isa_vendor_ext_list[] = {
#ifdef CONFIG_RISCV_ISA_VENDOR_EXT_THEAD
	&riscv_isa_vendor_ext_list_thead,
#endif
};

const size_t riscv_isa_vendor_ext_list_size = ARRAY_SIZE(riscv_isa_vendor_ext_list);
