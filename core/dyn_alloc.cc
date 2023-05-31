#include <osv/dyn_alloc.hh>
#include <memory>
#include <osv/align.hh>
#include <osv/prio.hh>
//#include <osv/trace.hh>
#include <stdint.h>
#include <osv/error.h>
#include <osv/rwlock.h>
#include <osv/mmu-defs.hh>
#include <cmath>
#include <atomic>
#include <osv/debug.hh>
#include <boost/intrusive/set.hpp>
//#include <vector>

namespace mmu {

#define OCC_RIGHT   1
#define OCC_LEFT    2
#define COAL_RIGHT  4
#define COAL_LEFT   8
#define OCC         16
#define BUSY        (OCC | OCC_LEFT | OCC_RIGHT)

constexpr size_t max_depth = 46; //static_cast<unsigned int>(std::ceil(std::log(static_cast<float>(upper_vma_limit-lower_vma_limit)) / std::log(static_cast<float>(2))));
constexpr size_t depth = 34; //max_depth - static_cast<unsigned int>(std::ceil(std::log(static_cast<float>(page_size)) / std::log(static_cast<float>(2))));
constexpr size_t tree_size = 17179869184; //static_cast<unsigned int>(std::pow(2,depth+1)); // we start at index one leaving 0 empty for simplicity
constexpr size_t _max_level = 4;

namespace bi = boost::intrusive;

class buddy : public bi::set_base_hook<>
{
public:
    buddy(size_t pos, uintptr_t mem_start, size_t mem_size) : _pos(pos), _mem_start(mem_start), _mem_size(mem_size) {}
    buddy(buddy &b) : _pos(b._pos), _mem_start(b._mem_start), _mem_size(b._mem_size) {
        _val.exchange(b._val.load());
    }
    size_t _pos;
    uintptr_t _mem_start;
    size_t _mem_size;
    std::atomic<unsigned char> _val {(unsigned char) 0}; //~OCC
    bi::set_member_hook<> _member_hook;
    friend bool operator< (const buddy &a, const buddy &b)
        {  return a._pos < b._pos;  }
    friend bool operator> (const buddy &a, const buddy &b)
        {  return a._pos > b._pos;  }
    friend bool operator== (const buddy &a, const buddy &b)
        {  return a._pos == b._pos;  }
};

struct _pos_int_is_key
{
   typedef size_t type;

   const type & operator()(const buddy& v) const
   {  return v._pos;  }
};

size_t level_n(size_t n) 
{
    return std::log2(n);
}

size_t size_n(size_t n) 
{
    return upper_vma_limit/std::pow(2, level_n(n));
}

uintptr_t starting_n(size_t n)
{
    return lower_vma_limit + ((n-std::pow(2, level_n(n))) * size_n(n));
}

typedef bi::set<buddy,  bi::key_of_value<_pos_int_is_key>,
                        bi::member_hook<buddy, 
                        bi::set_member_hook<>, 
                        &buddy::_member_hook>, 
                        bi::optimize_size<true>> 
                        buddy_tree_base;

struct buddy_tree_type : buddy_tree_base {
    buddy_tree_type() {
       /*for (size_t i=std::pow(2, max_level-1); i<tree_size; i++) {
            buddy *new_buddy = new buddy(i, starting_n(i), size_n(i));
            debug_early(("init buddy " + std::to_string(i) + " \n").c_str());
            insert(*new_buddy);
       }*/
    }
};

//typedef bi::set<buddy, bi::key_of_value<_pos_int_is_key>> buddy_tree_type;
__attribute__((init_priority((int)init_prio::buddy_tree)))
buddy_tree_type buddy_tree;
//std::atomic<unsigned char> buddy_tree[tree_size] = {};

const auto max_size = upper_vma_limit - lower_vma_limit; // Can be adjusted

void init_buddy_lazy(size_t n) 
{
    if (buddy_tree.find(n) == buddy_tree.end()) {
        auto new_buddy = new buddy(n, starting_n(n), size_n(n));
        //debug_early(("init buddy " + std::to_string(n) + " \n").c_str());
        buddy_tree.insert(*new_buddy);
    }/* else {
        debug_early(("tried init buddy " + std::to_string(n) + " \n").c_str());
    }*/
}

bool is_free(unsigned char val)
{
    return !(val & BUSY);
}

unsigned char clean_coal(unsigned char val, size_t child) {
    return val & !(COAL_LEFT >> child%2);
}

unsigned char mark(unsigned char val, size_t child) { 
    return val | (OCC_LEFT >> child%2);
}

unsigned char unmark(unsigned char val, size_t child) { 
    return val & !((OCC_LEFT | OCC_LEFT) >> child%2);
}

bool is_coal(unsigned char val, size_t child) {
    return val & (OCC_LEFT >> child%2);
}

bool is_occ_buddy(unsigned char val, size_t child) {
    return val & (OCC_RIGHT << child%2);
}

bool is_coal_buddy(unsigned char val, size_t child) {
    return val & (COAL_RIGHT << child%2);
}

void unmark(size_t n, size_t upper_bound)
{
    auto current = n;
    unsigned char cur_val;
    unsigned char new_val;
    size_t child;
    do {
        child = current;
        current = n >> 1;
        do {
            cur_val = buddy_tree.find(current)->_val.load();
            if (!is_coal(cur_val, child)) {
                return;
            }
            new_val = unmark(cur_val, child);
        } while (!buddy_tree.find(current)->_val.compare_exchange_weak(cur_val, new_val));
    } while(level_n(current) > upper_bound && !is_occ_buddy(new_val, child));
}

void freenode(size_t n, size_t upper_bound)
{
    //debug_early(("freenode n " + std::to_string(n) + " \n").c_str());
    //debug_early(("freenode starting_n(n) " + std::to_string(starting_n(n)) + " \n").c_str());
    auto current = n >> 1;
    auto runner = n;
    while (level_n(runner) > upper_bound) {
        auto or_val = COAL_LEFT >> current % 2;
        unsigned char cur_val;
        unsigned char new_val;
        //debug_early(("freenode level_n(runner) " + std::to_string(level_n(runner)) + " \n").c_str());
        //debug_early(("freenode " + std::to_string(n) + " \n").c_str());
        do {
            cur_val = buddy_tree.find(current)->_val.load();
            new_val = cur_val | or_val;
        } while (!buddy_tree.find(current)->_val.compare_exchange_weak(cur_val, new_val));
        //debug_early(("cur_val " + std::to_string(cur_val) + " \n").c_str());
        //debug_early(("new_val " + std::to_string(new_val) + " \n").c_str());
        if (is_occ_buddy(cur_val, runner) && !is_coal_buddy(cur_val, runner)) break;
        runner = current;
        current = current >> 1;
    }
    //debug_early(("freenode n " + std::to_string(n) + " \n").c_str());
    //debug_early(("freenode starting_n(n) " + std::to_string(starting_n(n)) + " \n").c_str());
    buddy_tree.find(n)->_val.exchange(0); //~OCC
    //assert(is_free(buddy_tree.find(n)->_val.load()));
    if (level_n(n) != upper_bound) {
        unmark(n, upper_bound);
    }
}

size_t tryalloc(size_t n, size_t max_level=_max_level) 
{
    unsigned char busy = BUSY;
    unsigned char zero = 0; //~OCC
    if (!buddy_tree.find(n)->_val.compare_exchange_weak(zero, busy)) { //::atomic_compare_exchange_weak(buddy_tree[n]._val.get(), 0, BUSY)
        return n;
    }
    auto current = n;
    while (level_n(current) > max_level) {
        auto child = current;
        current = current >> 1;
        unsigned char cur_val;
        unsigned char new_val;
        init_buddy_lazy(current);
        //debug_early(("current " + std::to_string(current) + " \n").c_str());
        do {
            cur_val = buddy_tree.find(current)->_val.load();
            //debug_early(("cur_val " + std::to_string(cur_val) + " \n").c_str());
            if (cur_val & OCC) {
                freenode(n, level_n(child));
                return current;
            }
            new_val = clean_coal(cur_val, child);
            new_val = mark(new_val, child);
            //debug_early(("new_val " + std::to_string(new_val) + " \n").c_str());
        } while (!buddy_tree.find(current)->_val.compare_exchange_weak(cur_val, new_val));
    }
    return 0;
}

std::pair<uintptr_t, size_t> nballoc(size_t size, uintptr_t hint) 
{
    if (size > max_size) {
        throw make_error(ENOMEM);
    }
    size_t level = std::log2(upper_vma_limit/size);
    if (upper_vma_limit/std::pow(2, level) < size) {
        level--;
    }
    //debug_early(("nballoc level " + std::to_string(level) + " \n").c_str());
    assert(level >= _max_level);
    if (level > depth) {
        level = depth;
    }
    //debug_early(("nballoc level " + std::to_string(level) + " \n").c_str());
    //debug_early(("nballoc hint " + std::to_string(hint) + " \n").c_str());
    size_t start = std::pow(2, level);
    //debug_early(("nballoc start " + std::to_string(start) + " \n").c_str());
    size_t end = std::pow(2, level+1)-1;
    /*debug_early(("nballoc end " + std::to_string(end) + " \n").c_str());
    debug_early(("nballoc hint/upper_vma " + std::to_string((end-start) *((double)hint/(double)upper_vma_limit)) + " \n").c_str());*/
    start = start + ((end-start) * ((double)hint/(double)upper_vma_limit));
    if (starting_n(start) < hint) {
        start++;
    }
    /*debug_early(("nballoc with hint " + std::to_string(start) + " \n").c_str());
    debug_early(("nballoc size " + std::to_string(size) + " \n").c_str());
    debug_early(("nballoc size_n " + std::to_string(size_n(start)) + " \n").c_str());*/
    //debug_early("new nballoc\n");
    size_t i;
    for (i = start; i <= end; i++) {
        init_buddy_lazy(i);
        //debug_early(("nballoc i->_val " + std::to_string(buddy_tree.find(i)->_val.load()) + " \n").c_str());
        if (is_free(buddy_tree.find(i)->_val.load())) {
            //debug_early(("nballoc i->_val is_free " + std::to_string(buddy_tree.find(i)->_val.load()) + " \n").c_str());
            auto failed_at = tryalloc(i);
            if (!failed_at) {
                return {starting_n(i), i}; //buddy_tree[i]._mem_start
            } else {
                auto d =  (1 << (level_n(i) - level_n(failed_at)));
                i = (failed_at + 1) * d;
            }
        } /*else {
            //if (i > start+((end-start)/4)) {
                debug_early(("nballoc level_n " + std::to_string(level_n(i)) + " \n").c_str());
                debug_early(("nballoc i " + std::to_string(i) + " \n").c_str());
                debug_early(("nballoc i->_val " + std::to_string(buddy_tree.find(i)->_val.load()) + " \n").c_str());
            //}
            auto current = i;
            do {
                current = current >> 1;
                init_buddy_lazy(current);
                debug_early(("nballoc else current " + std::to_string(current) + " \n").c_str());
                debug_early(("nballoc else current level " + std::to_string(level_n(current)) + " \n").c_str());
                debug_early(("nballoc else current val " + std::to_string(buddy_tree.find(current)->_val.load()) + " \n").c_str());
            } while (((buddy_tree.find(current)->_val.load() & OCC_RIGHT)) && level_n(current) > _max_level); // | OCC  // is_free(buddy_tree.find(current)->_val.load()) // (buddy_tree.find(current)->_val.load() & (OCC | OCC_RIGHT) //!(is_free(buddy_tree.find(current)->_val.load())
            //debug_early(("nballoc else current after w " + std::to_string(current) + " \n").c_str());
            auto old_i = i;
            if (level_n(current) == _max_level) {
                auto d =  (1 << (level_n(i) - level_n(current)));
                //debug_early(("nballoc else d " + std::to_string(d) + " \n").c_str());
                i = (current + 1) * d;
            } else {
                auto d =  (1 << (level_n(i) - level_n(current*2 + 1)));
                //debug_early(("nballoc else d " + std::to_string(d) + " \n").c_str());
                i = (current*2 + 1) * d;
            }
            //debug_early(("nballoc else i " + std::to_string(i) + " \n").c_str());
            //debug_early(("nballoc else i->_val " + std::to_string(buddy_tree.find(i)->_val.load()) + " \n").c_str());
            if (old_i!=i) i--;
            //init_buddy_lazy(i);
            //if(old_i+1 < i) {
                //debug_early(("nballoc else i " + std::to_string(i) + " old i: " + std::to_string(old_i) + " \n").c_str());
                //debug_early(("nballoc else i->_val " + std::to_string(buddy_tree.find(i)->_val.load()) + " \n").c_str());
            //}
        }*/
    }
    debug_early(("nballoc i " + std::to_string(i) + " \n").c_str());
    throw make_error(ENOMEM);
}

std::pair<int, size_t> nballoc_fixed(uintptr_t start, size_t size) 
{
    if (size > max_size) {
        throw make_error(ENOMEM);
    }
    size_t level = std::log2(upper_vma_limit/size);
    if (upper_vma_limit/std::pow(2, level) < size) {
        level--;
    }
    assert(level >= _max_level);
    if (level > depth) {
        level = depth;
    }
    size_t _start = std::pow(2, level);
    size_t end = std::pow(2, level+1)-1;
    _start = _start + ((end-_start) * ((double)start/(double)upper_vma_limit));
    /*debug_early(("nballoc level " + std::to_string(level) + " \n").c_str());
    debug_early(("nballoc start " + std::to_string(start) + " \n").c_str());
    debug_early(("nballoc start + size " + std::to_string(start + size) + " \n").c_str());
    debug_early(("nballoc size " + std::to_string(size) + " \n").c_str());
    debug_early(("nballoc size_n " + std::to_string(size_n(_start)) + " \n").c_str());
    debug_early(("nballoc _start " + std::to_string(_start) + " \n").c_str());
    debug_early(("nballoc starting_n(_start) " + std::to_string(starting_n(_start)) + " \n").c_str());
    debug_early(("nballoc starting_n(_start+1) " + std::to_string(starting_n(_start+1)) + " \n").c_str());
    debug_early(("nballoc starting_n(end) " + std::to_string(starting_n(_start) + size_n(_start)) + " \n").c_str());
    debug_early(("nballoc starting_n(end+1) " + std::to_string(starting_n(_start+1) + size_n(_start+1)) + " \n").c_str());*/
    if (start >= starting_n(_start) + size_n(_start)) {
        _start++;
    }
    if (starting_n(_start) + size_n(_start) > (start + size)) {
        //debug_early("nballoc use lowest level\n");
        level = depth;
        _start = std::pow(2, level);
        end = std::pow(2, level+1)-1;
        _start = _start + ((end-_start) * ((double)start/(double)upper_vma_limit));
        /*debug_early(("nballoc level " + std::to_string(level) + " \n").c_str());
        debug_early(("nballoc size_n " + std::to_string(size_n(_start)) + " \n").c_str());
        debug_early(("nballoc _start " + std::to_string(starting_n(_start)) + " \n").c_str());
        debug_early(("nballoc starting_n(_start) " + std::to_string(starting_n(_start)) + " \n").c_str());
        debug_early(("nballoc starting_n(end) " + std::to_string(starting_n(_start) + size_n(_start)) + " \n").c_str());*/
    }

    //debug_early("nballoc start alloc\n");
    int count = 0;
    auto group_start = _start;
    while (starting_n(_start) + size_n(_start) < start + size) {
        /*debug_early(("nballoc _start " + std::to_string(_start) + " \n").c_str());
        debug_early(("nballoc starting_n(_start) " + std::to_string(starting_n(_start)) + " \n").c_str());
        debug_early(("nballoc starting_n(_start) + size_n(_start) " + std::to_string(starting_n(_start) + size_n(_start)) + " \n").c_str());*/
        init_buddy_lazy(_start);
        auto failed_at = tryalloc(_start);
        //debug_early(("nballoc failed_at " + std::to_string(failed_at) + " \n").c_str());
        if (!failed_at) {
            count++;
            _start++;
        } /*else if(starting_n(_start+1) + size_n(_start+1) > start + size) {
            break;
        }*/ else {
            throw make_error(ENOMEM);
        }
    }
    return {count, group_start};
    /*assert(starting_n(_start) <= start); //buddy_tree[_start]._mem_start, buddy_tree[_start]._mem_size
    assert(start + size <= starting_n(_start) + size_n(_start));
    auto failed_at = tryalloc(_start);
    if (!failed_at) {
        return {starting_n(_start), group_start};
    }
    throw make_error(ENOMEM);*/
}

void nbfree(size_t i) 
{
    freenode(i, _max_level);
}

split_result nbsplit(std::size_t ref, uintptr_t edge) 
{
    auto level = level_n(ref);
    auto size = size_n(ref);
    auto start = starting_n(ref);
    auto end = start+size;
    /*debug_early(("nbsplit level " + std::to_string(level) + " \n").c_str());
    debug_early(("nbsplit size " + std::to_string(size) + " \n").c_str());
    debug_early(("nbsplit start " + std::to_string(start) + " \n").c_str());
    debug_early(("nbsplit end " + std::to_string(end) + " \n").c_str());
    debug_early(("nbsplit edge " + std::to_string(edge) + " \n").c_str());*/
    if (edge <= start || end <= edge || level >= depth || edge-start < mmu::page_size || end-edge<mmu::page_size) {
        return {false,0,0,0,0};
    }
    auto min_size = std::min(edge-start, end-edge);
    size_t split_level = std::log2(upper_vma_limit/min_size);
    if (split_level > depth) {
        split_level = depth;
    }
    size_t left_start = ref*std::pow(2,split_level-level);
    size_t block_end = (ref+1)*std::pow(2,split_level-level);
    size_t right_start = left_start + ((block_end-left_start) * ((double)(edge-start)/(double)size));
    /*debug_early(("nbsplit min_size " + std::to_string(min_size) + " \n").c_str());
    debug_early(("nbsplit split_level " + std::to_string(split_level) + " \n").c_str());
    debug_early(("nbsplit left_start " + std::to_string(left_start) + " \n").c_str());
    debug_early(("nbsplit block_end " + std::to_string(block_end) + " \n").c_str());
    debug_early(("nbsplit right_start " + std::to_string(right_start) + " \n").c_str());*/
    for (size_t i=left_start; i<block_end; i++) {
        init_buddy_lazy(i);
        tryalloc(i,level-1);
    }
    return {true,left_start,(int)(right_start-left_start),right_start,(int) (block_end-right_start)};
}

split_result nbsplit(std::size_t ref, int buddy_count, uintptr_t edge) 
{
    auto level = level_n(ref);
    auto size = size_n(ref);
    auto start = starting_n(ref);
    auto end = start+(size*buddy_count);
    /*debug_early(("nbsplit level " + std::to_string(level) + " \n").c_str());
    debug_early(("nbsplit size " + std::to_string(size) + " \n").c_str());
    debug_early(("nbsplit start " + std::to_string(start) + " \n").c_str());
    debug_early(("nbsplit end " + std::to_string(end) + " \n").c_str());
    debug_early(("nbsplit edge " + std::to_string(edge) + " \n").c_str());*/
    if (edge <= start || end <= edge || level >= depth || edge-start < mmu::page_size || end-edge<mmu::page_size) {
        return {false,0,0,0,0};
    }
    auto min_size = std::min(edge-start, end-edge);
    size_t split_level = std::log2(upper_vma_limit/min_size);
    if (split_level > depth) {
        split_level = depth;
    }
    size_t left_start = ref*std::pow(2,split_level-level);
    size_t block_end = (ref+1)*std::pow(2,split_level-level);
    size_t right_start = left_start + ((block_end-left_start) * ((double)(edge-start)/(double)size));
    /*debug_early(("nbsplit min_size " + std::to_string(min_size) + " \n").c_str());
    debug_early(("nbsplit split_level " + std::to_string(split_level) + " \n").c_str());
    debug_early(("nbsplit left_start " + std::to_string(left_start) + " \n").c_str());
    debug_early(("nbsplit block_end " + std::to_string(block_end) + " \n").c_str());
    debug_early(("nbsplit right_start " + std::to_string(right_start) + " \n").c_str());*/
    for (size_t i=left_start; i<block_end; i++) {
        init_buddy_lazy(i);
        tryalloc(i,level-1);
    }
    return {true,left_start,(int)(right_start-left_start),right_start,(int) (block_end-right_start)};
}

}