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

/**
 * MMU namespace
 */
namespace mmu {

constexpr uintptr_t lower_vma_limit = 0x0;
constexpr uintptr_t upper_vma_limit = 0x400000000000;
constexpr uintptr_t fixed_alloc_border = 0x200000000000;

struct split_result {
    bool splitted;
    std::size_t left_ref;
    int left_count;
    std::size_t right_ref;
    int right_count;
};

std::size_t size_n(std::size_t n);
uintptr_t starting_n(std::size_t n);

std::pair<uintptr_t, std::size_t> nballoc(std::size_t size, uintptr_t hint = fixed_alloc_border);

std::pair<int, std::size_t> nballoc_fixed(uintptr_t start, std::size_t size);

split_result nbsplit(std::size_t ref, uintptr_t edge);
split_result nbsplit(std::size_t ref, int buddy_count, uintptr_t edge);

void nbfree(std::size_t i);

}

#endif /* DYN_ALLOC_HH */
