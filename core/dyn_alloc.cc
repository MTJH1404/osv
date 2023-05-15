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
   uint _size;
   addr_range _range;
   //This is a member hook
   bi::set_member_hook<> _member_hook;

   free_block(addr_range range)
      :  _size(std::ceil(std::log(static_cast<float>(range.end()-range.start())) / std::log(static_cast<float>(2)))), _range(range)
      {}
   friend bool operator< (const free_block &a, const free_block &b)
      {  return a._range.start() < b._range.start();  }
   friend bool operator> (const free_block &a, const free_block &b)
      {  return a._range.start() > b._range.start();  }
   friend bool operator== (const free_block &a, const free_block &b)
      {  return a._range.start() == b._range.start();  }
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

struct n_uint_is_key
{
   typedef uint type;

   const type & operator()(const free_block& v) const
   {  return v._size;  }
};

const auto max_powers = static_cast<uint>(std::ceil(std::log(static_cast<float>(upper_vma_limit)) / std::log(static_cast<float>(2))));

//Define an multiset using the member hook
typedef bi::member_hook<free_block, bi::set_member_hook<>, &free_block::_member_hook> MemberOption;
typedef bi::multiset<free_block, MemberOption, bi::key_of_value<n_uint_is_key>> vma_free_multiset_base;

struct vma_free_multiset_type : vma_free_multiset_base 
{
    vma_free_multiset_type() {
        auto* b = new free_block(addr_range(lower_vma_limit, upper_vma_limit));
        insert(*b);
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
    // Calculate index in free list
    // to search for block if available
    auto n = std::ceil(std::log(static_cast<float>(size)) / std::log(static_cast<float>(2)));
    
    // Block available
    if (vma_free_list.count(n) > 0)
    {
        auto free = vma_free_list.find(n);
        //debug_early("found block\n");
        //debug_early(("found block start: " + std::to_string(free->_range.start()) + " end: " + std::to_string(free->_range.end()) + "\n ").c_str());

        // Remove block from free list
        vma_free_list.erase(free);
 
        return free->_range;
    }
    else
    {
        ulong i;
         
        // If not, search for a larger block
        for(i = n + 1; i <= max_powers; i++)
        {
             
            // Find block size greater
            // than request
            if (vma_free_list.count(i) != 0)
                break;
        }
 
        // If no such block is found
        // i.e., no memory block available
        if (i == size) {
            throw make_error(ENOMEM);
        } else {
            // If found
            auto temp = vma_free_list.find(i);
 
            // Remove first block to split
            // it into halves
            vma_free_list.erase(temp);
            i--;
             
            for(;i >= n; i--) {
                //debug_early(("i " + std::to_string(i) + "\n ").c_str());
                auto start = temp->_range.start();
                auto end = temp->_range.end();
                auto half = (end-start)/2;
                auto split = align_up(start+half, mmu::huge_page_size);
                if (split >= end) {
                    split = align_up(start+half, mmu::page_size);
                }
                //debug_early(("start " + std::to_string(start) + "\n ").c_str());
                //debug_early(("end " + std::to_string(end) + "\n ").c_str());
                //debug_early(("split " + std::to_string(split) + "\n ").c_str());

                // Divide block into two halves
                auto* pair1 = new free_block(addr_range(start, split));
                auto* pair2 = new free_block(addr_range(split, end));
                vma_free_list.insert(*pair2);
                vma_free_list.insert(*pair1);

                temp = vma_free_list.find(i);
 
                // Remove first free block to
                // further split
                vma_free_list.erase(temp);
            }       
            //debug_early(("find return start: " + std::to_string(temp->_range.start()) + " end: " + std::to_string(temp->_range.end()) + " \n").c_str());           
            return temp->_range;
        }
    }
}

void add_free(addr_range range)
{     
    auto id = range.start();
    auto size = range.end()-range.start();
    // Size of block to be searched
    int n = std::ceil(std::log(static_cast<float>(size)) / std::log(static_cast<float>(2)));
     
    uintptr_t buddyNumber, buddyAddress;
 
    // Add the block in free list
    auto* free = new free_block(range);
    /*arr[n].push_back(make_pair(id,
                               id + pow(2, n) - 1));
    cout << "Memory block from " << id
         << " to "<< id + pow(2, n) - 1
         << " freed\n";*/
 
    // Calculate buddy number
    buddyNumber = id / size;
 
    if (buddyNumber % 2 != 0)
        buddyAddress = id - std::pow(2, n);
    else
        buddyAddress = id + std::pow(2, n);
         
    // Search in free list to find it's buddy
    auto equal_range = vma_free_list.equal_range(n);
    for(auto i = equal_range.first; i != equal_range.second; i++)
    {
         
        // If buddy found and is also free
        if (i->_range.start() == buddyAddress)
        {
             
            // Now merge the buddies to make
            // them one large free memory block
            if (buddyNumber % 2 == 0) {
                free = new free_block(addr_range(id, id + 2 * std::pow(2, n) - 1));
            } else {
                free = new free_block(addr_range(buddyAddress, buddyAddress + 2 * std::pow(2, n) - 1));
            }
            break;
        }
    }
 
    vma_free_list.insert(*free);
}

std::vector<addr_range> crop_front (addr_range range, uintptr_t start) 
{
    auto half = (range.end()-range.start())/2;
    auto split = align_up(range.start()+half, mmu::huge_page_size);
    if (split >= range.end()) {
        split = align_up(range.start()+half, mmu::page_size);
    }
    std::vector<addr_range> buddies;
    while (split > mmu::page_size && start - range.start() < mmu::page_size) {
        if (split < start) {
            auto* nbuddy = new free_block(addr_range(range.start(), split));
            vma_free_list.insert(*nbuddy);
            range = addr_range(split, range.end());
        } else {
            addr_range back_range(split, range.end());
            buddies.push_back(back_range);
            range = addr_range(range.start(), split);
        }
    }
    buddies.push_back(range);
    return buddies;
}

std::vector<addr_range> crop_back (addr_range range, uintptr_t end) 
{
    auto half = (range.end()-range.start())/2;
    auto split = align_up(range.start()+half, mmu::huge_page_size);
    if (split >= range.end()) {
        split = align_up(range.start()+half, mmu::page_size);
    }
    std::vector<addr_range> buddies;
    while (split > mmu::page_size && range.end() - end < mmu::page_size) {
        if (end < split) {
            auto* nbuddy = new free_block(addr_range(split, range.end()));
            vma_free_list.insert(*nbuddy);
            range = addr_range(range.start(), split);
        } else {
            addr_range back_range(range.start(), split);
            buddies.push_back(back_range);
            range = addr_range(split, range.end());
        }
    }
    buddies.push_back(range);
    return buddies;
}

std::vector<addr_range> aquire_fixed(uintptr_t start, uintptr_t end)
{
    //debug_early("aquire fixed \n");
    assert(start < upper_vma_limit && end <= upper_vma_limit);
    std::vector<addr_range> buddies;
    bool memory_aquired = false;
    uintptr_t reserved_start = start;
    uintptr_t reserved_end = end;
    WITH_LOCK(vma_free_list_mutex.for_write()) {
        for (auto i = max_powers; i > 0; i--) {
            //print_free_blocks();
            auto equal_range = vma_free_list.equal_range(i);
            for (auto j = equal_range.first; j != equal_range.second; ) {
                addr_range range = j->_range;
                if (range.start() <= start && end <= range.end()) {
                    j = vma_free_list.erase(j);
                    auto half = (range.end()-range.start())/2;
                    auto split = align_up(range.start()+half, mmu::huge_page_size);
                    if (split >= range.end()) {
                        split = align_up(range.start()+half, mmu::page_size);
                    }
                    while (split < start || end < split) {
                        addr_range not_required (0,0);
                        if (split < start) {
                            not_required = addr_range(range.start(), split);
                            range = addr_range(split, range.end());
                        } else {
                            not_required = addr_range(split, range.end());
                            range = addr_range(range.start(), split);
                        }
                        auto* nbuddy = new free_block(not_required);
                        vma_free_list.insert(*nbuddy);
                        half = (range.end()-range.start())/2;
                        split = align_up(range.start()+half, mmu::huge_page_size);
                        if (split >= range.end()) {
                            split = align_up(range.start()+half, mmu::page_size);
                        }
                        // crop unnecassary memory from the front and back
                        addr_range front_range(range.start(), split);
                        addr_range back_range(split, range.end());
                        auto front_buddies = crop_front(front_range, start);
                        auto back_buddies = crop_back(back_range, end);
                        reserved_start = front_buddies.end()->start();
                        reserved_end = back_buddies.end()->end();
                        if (reserved_start == range.start() && reserved_end == range.end()) {
                            // nothing was cropped from the range
                            buddies.push_back(range);
                        } else {
                            buddies.insert(buddies.end(), front_buddies.begin(), front_buddies.end());
                            buddies.insert(buddies.end(), back_buddies.begin(), back_buddies.end());
                        }
                        memory_aquired = true;
                        break;
                    }
                } else if (start < range.start() && range.end() < end) { // buddy within the requested memory region
                    j = vma_free_list.erase(j);
                    buddies.push_back(range);
                }  else if (range.start() <= start && start < range.end() ) { // buddy containing the start address
                    j = vma_free_list.erase(j);
                    auto cropped = crop_front(range, start);
                    reserved_start = cropped.end()->start();
                    buddies.insert(buddies.end(), cropped.begin(), cropped.end());
                } else if (range.start() < end && end <= range.end()) { // buddy containing the end address 
                    j = vma_free_list.erase(j);
                    auto cropped = crop_front(range, start);
                    reserved_end = cropped.end()->end();
                    buddies.insert(buddies.end(), cropped.begin(), cropped.end());
                } else { // buddy outside of the requested area
                    ++j;
                }
            }
            if (memory_aquired) break;
        }
    }
    return buddies;
}
}