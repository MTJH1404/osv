#include <osv/dyn_alloc.hh>
#include <memory>
#include <osv/align.hh>
#include <osv/rwlock.h>
#include <osv/prio.hh>
#include <osv/mmu-defs.hh>
//#include <osv/trace.hh>
#include <stdint.h>
#include <cmath>
#include <boost/intrusive/set.hpp>
#include <osv/error.h>

namespace mmu {

namespace bi = boost::intrusive;

class free_block : public bi::set_base_hook<bi::optimize_size<true>>
{
   public:
   ulong _size;
   addr_range _range;
   //This is a member hook
   bi::set_member_hook<> _member_hook;

   free_block(addr_range range)
      :  _size(range.end()-range.start()), _range(range)
      {}
   friend bool operator< (const free_block &a, const free_block &b)
      {  return a._range.start() < b._range.start();  }
   friend bool operator> (const free_block &a, const free_block &b)
      {  return a._range.start() > b._range.start();  }
   friend bool operator== (const free_block &a, const free_block &b)
      {  return a._range.start() == b._range.start();  }
};

struct _size_is_key
{
   typedef ulong type;

   const type & operator()(const free_block& v) const
   {  return v._size;  }
};

class addr_compare {
public:
    bool operator()(const free_block& x, uintptr_t y) const { return x._range.start() < y; }
    bool operator()(uintptr_t x, const free_block& y) const { return x < y._range.start(); }
};

class addr_size_compare {
public:
    bool operator()(const free_block& x, ulong y) const { return x._range.end()-x._range.start() < y; }
    bool operator()(ulong x, const free_block& y) const { return x < y._range.end()-y._range.start(); }
};

//Define an multiset using the member hook
typedef bi::member_hook<free_block, bi::set_member_hook<>, &free_block::_member_hook> MemberOption;
typedef bi::multiset<free_block, MemberOption> vma_free_multiset_base;

struct vma_free_multiset_type : vma_free_multiset_base 
{
    vma_free_multiset_type() {
        auto* free = new free_block(addr_range(lower_vma_limit+1, upper_vma_limit-1));
        //debug_early(("insert init start: " + std::to_string(free->_range.start()) + " end: " + std::to_string(free->_range.end()) + " \n").c_str());
        insert(*free);
    }
};

__attribute__((init_priority((int)init_prio::vma_free_list)))
vma_free_multiset_type vma_free_list;
rwlock_t vma_free_list_mutex;

void print_free_blocks() 
{
    for(auto i = vma_free_list.begin(); i!= vma_free_list.end(); i++) {
        debug_early(("free block start: " + std::to_string(i->_range.start()) + " end: " + std::to_string(i->_range.end()) + " \n").c_str());
    }
}

addr_range find_free(ulong size)
{
    //debug_early(("find free " + std::to_string(size) + " \n").c_str());
    bool small = size < huge_page_size;
    //print_free_blocks();
    WITH_LOCK(vma_free_list_mutex.for_write()) {
        auto free = vma_free_list.lower_bound(size, addr_size_compare());
        auto start = free->_range.start();
        if (!small) {
            start = align_up(free->_range.start(), huge_page_size);
            while (free->_range.end()-start < size) {
                free++;
                start = align_up(free->_range.start(), huge_page_size);
            }
        } else {
            start = align_up(free->_range.start(), page_size);
            while (free->_range.end()-start < size) {
                free++;
                start = align_up(free->_range.start(), page_size);
            }
        }
        auto free_range = free->_range;
        auto free_size = free->_size;
        if (free == vma_free_list.end()) {
            throw make_error(ENOMEM);
        }
        
        vma_free_list.erase(free);
        
        //delete &free;
        if (free_size > size) {
            if (start == free_range.start()) {
                auto* unused = new free_block(addr_range(free_range.start()+size, free_range.end()));
                //debug_early(("insert back ff start: " + std::to_string(unused->_range.start()) + " end: " + std::to_string(unused->_range.end()) + " \n").c_str());
                vma_free_list.insert(*unused);
            } else {
                auto* unused = new free_block(addr_range(free_range.start(), start));
                //debug_early(("insert back ff start: " + std::to_string(unused->_range.start()) + " end: " + std::to_string(unused->_range.end()) + " \n").c_str());
                vma_free_list.insert(*unused);
                if (start+size != free_range.end()) {
                    unused = new free_block(addr_range(start + size, free_range.end()));
                    //debug_early(("insert back ff start: " + std::to_string(unused->_range.start()) + " end: " + std::to_string(unused->_range.end()) + " \n").c_str());
                    vma_free_list.insert(*unused);
                }
            }
        }
        //debug_early(("return start: " + std::to_string(start) + " end: " + std::to_string(start+size) + " \n").c_str());
        //debug_early("finished find free \n");
        //print_free_blocks();
        return addr_range(start,start+size);
    }
}

void add_free(addr_range range) 
{
    //debug_early("add free \n");
    auto start = range.start();
    auto end = range.end();
    //debug_early(("add free start: " + std::to_string(start) + " end: " + std::to_string(end) + " \n").c_str());
    assert(start < upper_vma_limit && end <= upper_vma_limit);
    //print_free_blocks();
    WITH_LOCK(vma_free_list_mutex.for_write()) {
        auto free = vma_free_list.lower_bound(range.start(), addr_compare());
        if (free != vma_free_list.begin()) {
            free--;
            if (free->_range.end() == range.start()) {
                start = free->_range.start();
                auto next = vma_free_list.erase(free);
                //delete &free;
                free = next;
            }
        }
        if (free->_range.start() == range.end()) {
            end = free->_range.end();
            vma_free_list.erase(free);
            //delete &free;
        }
        auto* fb = new free_block(addr_range(start, end));
        //debug_early(("insert addf start: " + std::to_string(fb->_range.start()) + " end: " + std::to_string(fb->_range.end()) + " \n").c_str());
        vma_free_list.insert(*fb);
    }
    //debug_early("finished add free \n");
    //print_free_blocks();
}

/**
 * @brief guarantes that start and end lay in the aquired memory region. 
*         caller needs to ensure, that the memory area is free
 * 
 * @param start start of the requested memory region
 * @param end end of the requested memory region
 * @return ** addr_range
 */
addr_range aquire_fixed(uintptr_t start, uintptr_t end) 
{
    //debug_early("aquire fixed \n");
    auto size = end-start;
    //print_free_blocks();
    assert(start < upper_vma_limit && end <= upper_vma_limit);
    WITH_LOCK(vma_free_list_mutex.for_write()) {
        auto free = vma_free_list.lower_bound(start, addr_compare());
        if (free == vma_free_list.end() || end <= free->_range.start()) --free;
        auto free_range = free->_range;
        //debug_early(("aquire fixed try start: " + std::to_string(start) + " end: " + std::to_string(end) + " \n").c_str());
        //debug_early(("aquire fixed found start: " + std::to_string(free_range.start()) + " end: " + std::to_string(free_range.end()) + " \n").c_str());
        assert(free_range.start() <= start && end <= free_range.end());
        auto free_size = free->_size;
        assert(free_size >= size);
        vma_free_list.erase(free);
        //delete &free;
        if (free_range.start() < start) {
            auto* unused = new free_block(addr_range(free_range.start(), start));
            //debug_early(("insert back af start: " + std::to_string(unused->_range.start()) + " end: " + std::to_string(unused->_range.end()) + " \n").c_str());
            vma_free_list.insert(*unused);
        }
        if (end < free_range.end()) {
            auto* unused = new free_block(addr_range(end, free_range.end()));
            //debug_early(("insert back af start: " + std::to_string(unused->_range.start()) + " end: " + std::to_string(unused->_range.end()) + " \n").c_str());
            vma_free_list.insert(*unused);
        }
    }
    //debug_early(("return start: " + std::to_string(start) + " end: " + std::to_string(end) + " \n").c_str());
    //debug_early("finished aquire fixed \n");
    return addr_range(start, end);
}
}