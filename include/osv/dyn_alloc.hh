/*
 * Copyright (C) 2013 Cloudius Systems, Ltd.
 *
 * This work is open source software, licensed under the terms of the
 * BSD license as described in the LICENSE file in the top-level directory.
 */

#ifndef DYN_ALLOC_HH
#define DYN_ALLOC_HH

#include <osv/types.h>
#include <functional>
#include <osv/addr_range.hh>
#include <vector>

/**
 * MMU namespace
 */
namespace mmu {
constexpr uintptr_t lower_vma_limit = 0x0;
constexpr uintptr_t upper_vma_limit = 0x400000000000;

addr_range find_free(ulong size);
void add_free(addr_range range);
std::vector<addr_range> aquire_fixed(uintptr_t start, uintptr_t end);
}

#endif /* DYN_ALLOC_HH */
