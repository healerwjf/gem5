/*
 * Copyright (c) 2018 Inria
 * Copyright (c) 2013,2016-2018 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 *          Nikos Nikoleris
 *          Daniel Carvalho
 */

/**
 * @file
 * Definitions a fully associative seq tagstore.
 */

#include "mem/cache/tags/fa_seq.hh"

#include <cassert>
#include <sstream>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "mem/cache/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "debug/FASEQ.hh"

std::string
FASEQBlk::print() const
{
    return csprintf("%s inCachesMask: %#x", CacheBlk::print(), inCachesMask);
}

FASEQ::FASEQ(const Params *p)
    : BaseTags(p),
      pageShift(p->page_shift),
      spmTracking(p->min_tracked_cache_size, size, blkSize)
      
{
    if (!isPowerOf2(blkSize))
        fatal("cache block size (in bytes) `%d' must be a power of two",
              blkSize);
    //if (!isPowerOf2(size))
        //fatal("Cache Size must be power of 2 for now");

    //numBlocks = 1536; 

    blks = new FASEQBlk[numBlocks];
}

FASEQ::~FASEQ()
{
    delete[] blks;
}

void
FASEQ::tagsInit()
{

    head = &(blks[0]);
    head->prev = initial;
    head->next = &(blks[1]);
    head->setPosition(0, 0);
    head->data = &dataBlks[0];

    for (unsigned i = 1; i < numBlocks - 1; i++) {
        blks[i].prev = &(blks[i-1]);
        blks[i].next = &(blks[i+1]);
        blks[i].setPosition(0, i);

        // Associate a data chunk to the block
        blks[i].data = &dataBlks[blkSize*i];
    }

    tail = &(blks[numBlocks - 1]);
    tail->prev = &(blks[numBlocks - 2]);
    tail->next = nullptr;
    tail->setPosition(0, numBlocks - 1);
    tail->data = &dataBlks[(numBlocks - 1) * blkSize];

    spmTracking.init(head, tail);

    //tail = nullptr;

}

void
FASEQ::regStats()
{
    BaseTags::regStats();
    spmTracking.regStats(name());
}

void
FASEQ::invalidate(CacheBlk *blk)
{
    // Erase block entry reference in the hash table
    auto num_erased M5_VAR_USED =
        tagHash.erase(std::make_pair(blk->tag, blk->isSecure()));

    if (blk->isValid()) {
        /** 删除对应pageHash中相应的tag */
        Addr temp_page = blk->tag >> pageShift;

        auto& iter = pageHash[temp_page];
        if (iter.size() == 1) {
            pageHash.erase(temp_page);
            pageAccessCount.erase(temp_page);
        } else {
            auto itr = std::find(iter.begin(), iter.end(), blk->tag);
            iter.erase(itr);
        }
        /*
        auto iter = pageHash.find(temp_page);
        if (iter != pageHash.end())
            pageHash.erase(temp_page);
            */
    }

    //DPRINTF(FASEQ, "%s: page: %#x, tag: %#x, number of pageHash: %d.\n", __func__, temp_page, blk->tag, temp_vec.size());


    // Sanity check; only one block reference should be erased
    assert(num_erased == 1);

    // Invalidate block entry. Must be done after the hash is erased
    BaseTags::invalidate(blk);

    // Decrease the number of tags in use
    tagsInUse--;

    // Move the block to the tail to make it the next victim
    moveToTail((FASEQBlk*)blk);
}

void
FASEQ::setUnused(CacheBlk *blk)
{
    if (blk->isValid())
        invalidate(blk);
    if (tail == nullptr) {
        tail = nullptr;
    } else if (tail->prev) {
        tail = tail->prev;
    } else {
        tail = nullptr;
    }
}

void
FASEQ::setUsed(CacheBlk *blk)
{
    if (blk->isValid())
        invalidate(blk);

    if (tail == nullptr) {
        tail = head;
    } else if (tail->next) {
        tail = tail->next;
    }
    
}

CacheBlk*
FASEQ::accessBlock(Addr addr, bool is_secure, Cycles &lat)
{
    return accessBlock(addr, is_secure, lat, 0);
}

CacheBlk*
FASEQ::accessBlock(Addr addr, bool is_secure, Cycles &lat,
                   CachesMask *in_caches_mask)
{
    CachesMask mask = 0;
    FASEQBlk* blk = static_cast<FASEQBlk*>(findBlock(addr, is_secure));

    // If a cache hit
    if (blk && blk->isValid()) {
        mask = blk->inCachesMask;

        /** 将地址所属的page中所有cacheLine移动到头部 */
        Addr temp_page = addr >> pageShift;
        if (!insert_page) {
            std::vector<Addr> temp_vec = pageHash[temp_page];
            pageAccessCount[temp_page] = ticksToCycles(curTick());

            if (temp_vec.size() == (1 << (pageShift - 6))) {
                for (int i = 0; i < temp_vec.size(); ++i) {
                    Addr temp_tag = temp_vec[i];
                    auto iter = tagHash.find(std::make_pair(temp_tag, is_secure));
                    if (iter != tagHash.end()) {
                        // DPRINTF(FASEQ, "%s: moveTohead page: %#x, tag: %#x, size: %d.\n", __func__, temp_page, temp_tag, temp_vec.size());
                        FASEQBlk* temp_blk = static_cast<FASEQBlk*>((*iter).second);
                        moveToHead(temp_blk);
                    }
                }
            }
        }         
    }

    if (in_caches_mask) {
        *in_caches_mask = mask;
    }

    spmTracking.recordAccess(blk);

    // The tag lookup latency is the same for a hit or a miss
    lat = lookupLatency;

    return blk;
}

CacheBlk*
FASEQ::findBlock(Addr addr, bool is_secure) const
{
    FASEQBlk* blk = nullptr;

    Addr tag = extractTag(addr);
    auto iter = tagHash.find(std::make_pair(tag, is_secure));
    if (iter != tagHash.end()) {
        blk = (*iter).second;
    }

    if (blk && blk->isValid()) {
        assert(blk->tag == tag);
        assert(blk->isSecure() == is_secure);
    }

    return blk;
}

ReplaceableEntry*
FASEQ::findBlockBySetAndWay(int set, int way) const
{
    assert(set == 0);
    return &blks[way];
}

CacheBlk*
FASEQ::findVictim(Addr addr, const bool is_secure,
                  std::vector<CacheBlk*>& evict_blks) const
{

    // The victim is always stored on the tail for the FASEQ
     FASEQBlk* victim = tail;
     DPRINTF(FASEQ, "%s: hit page: %#x, tag: %#x.\n", __func__, victim->tag >> pageShift, victim->tag);

    // There is only one eviction for this replacement
    evict_blks.push_back(victim);

    return victim;
}

CacheBlk*
FASEQ::findUsed(Addr addr,
                  std::vector<CacheBlk*>& evict_blks)
{
    FASEQBlk* victim = nullptr;

    if (tail) {
        DPRINTF(FASEQ, "%s: Doning here.\n", __func__);
        // The victim is always stored on the tail for the FASEQ
        victim = tail;

        // There is only one eviction for this replacement
        evict_blks.push_back(victim);
    }

    return victim;
}

CacheBlk*
FASEQ::findUnused(Addr addr,
                  std::vector<CacheBlk*>& evict_blks)
{
    FASEQBlk* victim = nullptr;

    if (tail == nullptr) {

        victim = head;

        evict_blks.push_back(victim);

    } else if (tail->next) {
        // The victim is always stored on the tail for the FASEQ
        victim = tail->next;

        // There is only one eviction for this replacement
        evict_blks.push_back(victim);
    }

    return victim;
}

void
FASEQ::insertBlock(const Addr addr, const bool is_secure,
                   const int src_master_ID, const uint32_t task_ID,
                   CacheBlk *blk)
{
    FASEQBlk* faseqBlk = static_cast<FASEQBlk*>(blk);

    // Make sure block is not present in the cache
    assert(faseqBlk->inCachesMask == 0);

    // Do common block insertion functionality
    BaseTags::insertBlock(addr, is_secure, src_master_ID, task_ID, blk);

    if (usedBlk < numBlocks) {
        usedBlk++;
        //std::cout << usedBlk << " " << numBlocks << std::endl;
    }

    // Increment tag counter
    tagsInUse++;

    // New block is the MRU
    moveToHead(faseqBlk);

    // Insert new block in the hash table
    tagHash[std::make_pair(blk->tag, blk->isSecure())] = faseqBlk;

    Addr temp_page = addr >> pageShift;
    auto iter = pageHash.find(temp_page);
    if (iter != pageHash.end()) {
        DPRINTF(FASEQ, "%s: hit page: %#x, tag: %#x.\n", __func__, temp_page, extractTag(addr));
        pageHash[temp_page].push_back(extractTag(addr));
        pageAccessCount[temp_page] = ticksToCycles(curTick());
    } else {
        DPRINTF(FASEQ, "%s: miss page: %#x, tag: %#x.\n", __func__, temp_page, extractTag(addr));
        std::vector<Addr> vec;
        vec.push_back(extractTag(addr));
        pageHash[temp_page] = vec;
    }
}

void
FASEQ::moveToHead(FASEQBlk *blk)
{
    /*
    // If block is not already head, do the moving
    if (blk != head) {
        spmTracking.moveBlockToHead(blk);
        // If block is tail, set previous block as new tail
        if (blk == tail && blk->next == nullptr){
            assert(blk->next == nullptr);
            tail = blk->prev;
            tail->next = nullptr;
        // Inform block's surrounding blocks that it has been moved
        } else if (blk == tail && blk->next != nullptr) {
            tail = blk->prev;
            tail->next = blk->next;
            blk->next->prev = tail;
        } else {
            blk->prev->next = blk->next;
            blk->next->prev = blk->prev;
        }

        // Swap pointers
        blk->next = head;
        blk->prev = nullptr;
        head->prev = blk;
        head = blk;

        spmTracking.check(head, tail);
    }
    */

    // If block is not already head, do the moving
    if (blk != head) {
        spmTracking.moveBlockToHead(blk);
        // If block is tail, set previous block as new tail
        if (blk == tail){
            assert(blk->next == nullptr);
            tail = blk->prev;
            tail->next = nullptr;
        // Inform block's surrounding blocks that it has been moved
        } else {
            blk->prev->next = blk->next;
            blk->next->prev = blk->prev;
        }

        // Swap pointers
        blk->next = head;
        blk->prev = nullptr;
        head->prev = blk;
        head = blk;

        spmTracking.check(head, tail);
    }
}

void
FASEQ::moveToTail(FASEQBlk *blk)
{
    /*
    // If block is not already tail, do the moving
    if (blk != tail) {
        spmTracking.moveBlockToTail(blk);
        // If block is head, set next block as new head
        if (blk == head){
            assert(blk->prev == nullptr);
            head = blk->next;
            head->prev = nullptr;
        // Inform block's surrounding blocks that it has been moved
        } else {
            blk->prev->next = blk->next;
            blk->next->prev = blk->prev;
        }

        // Swap pointers
        blk->prev = tail;
        if (!tail->next) {
            blk->next = tail->next;
            tail->next->prev = blk;
        } else {
            blk->next = nullptr; 
        }
        
        tail->next = blk;
        tail = blk;

        spmTracking.check(head, tail);
    }
    */
    // If block is not already tail, do the moving
    if (blk != tail) {
        spmTracking.moveBlockToTail(blk);
        // If block is head, set next block as new head
        if (blk == head){
            assert(blk->prev == nullptr);
            head = blk->next;
            head->prev = nullptr;
        // Inform block's surrounding blocks that it has been moved
        } else {
            blk->prev->next = blk->next;
            blk->next->prev = blk->prev;
        }

        // Swap pointers
        blk->prev = tail;
        blk->next = nullptr;
        tail->next = blk;
        tail = blk;

        spmTracking.check(head, tail);
    }
}

FASEQ *
FASEQParams::create()
{
    return new FASEQ(this);
}

void
FASEQ::SPMTracking::check(const FASEQBlk *head, const FASEQBlk *tail) const
{
#ifdef FASEQ_DEBUG
    const FASEQBlk* blk = head;
    unsigned curr_size = 0;
    unsigned tracked_cache_size = minTrackedSize;
    CachesMask in_caches_mask = inAllCachesMask;
    int j = 0;

    while (blk) {
        panic_if(blk->inCachesMask != in_caches_mask, "Expected cache mask "
                 "%x found %x", blk->inCachesMask, in_caches_mask);

        curr_size += blkSize;
        if (curr_size == tracked_cache_size && blk != tail) {
            panic_if(boundaries[j] != blk, "Unexpected boundary for the %d-th "
                     "cache", j);
            tracked_cache_size <<= 1;
            // from this point, blocks fit only in the larger caches
            in_caches_mask &= ~(1U << j);
            ++j;
        }
        blk = blk->next;
    }
#endif // FASEQ_DEBUG
}

void
FASEQ::SPMTracking::init(FASEQBlk *head, FASEQBlk *tail)
{
    // early exit if we are not tracking any extra caches
    FASEQBlk* blk = numTrackedCaches ? head : nullptr;
    unsigned curr_size = 0;
    unsigned tracked_cache_size = minTrackedSize;
    CachesMask in_caches_mask = inAllCachesMask;
    int j = 0;

    while (blk) {
        blk->inCachesMask = in_caches_mask;

        curr_size += blkSize;
        if (curr_size == tracked_cache_size && blk != tail) {
            boundaries[j] = blk;

            tracked_cache_size <<= 1;
            // from this point, blocks fit only in the larger caches
            in_caches_mask &= ~(1U << j);
            ++j;
        }
        blk = blk->next;
    }
}


void
FASEQ::SPMTracking::moveBlockToHead(FASEQBlk *blk)
{
    // Get the mask of all caches, in which the block didn't fit
    // before moving it to the head
    CachesMask update_caches_mask = inAllCachesMask ^ blk->inCachesMask;

    for (int i = 0; i < numTrackedCaches; i++) {
        CachesMask current_cache_mask = 1U << i;
        if (current_cache_mask & update_caches_mask) {
            // if the ith cache didn't fit the block (before it is moved to
            // the head), move the ith boundary 1 block closer to the
            // MRU
            boundaries[i]->inCachesMask &= ~current_cache_mask;
            boundaries[i] = boundaries[i]->prev;
        } else if (boundaries[i] == blk) {
            // Make sure the boundary doesn't point to the block
            // we are about to move
            boundaries[i] = blk->prev;
        }
    }

    // Make block reside in all caches
    blk->inCachesMask = inAllCachesMask;
}

void
FASEQ::SPMTracking::moveBlockToTail(FASEQBlk *blk)
{
    CachesMask update_caches_mask = blk->inCachesMask;

    for (int i = 0; i < numTrackedCaches; i++) {
        CachesMask current_cache_mask = 1U << i;
        if (current_cache_mask & update_caches_mask) {
            // if the ith cache fitted the block (before it is moved to
            // the tail), move the ith boundary 1 block closer to the
            // LRU
            boundaries[i] = boundaries[i]->next;
            if (boundaries[i] == blk) {
                // Make sure the boundary doesn't point to the block
                // we are about to move
                boundaries[i] = blk->next;
            }
            boundaries[i]->inCachesMask |= current_cache_mask;
        }
    }

    // The block now fits only in the actual cache
    blk->inCachesMask = 0;
}

void
FASEQ::SPMTracking::recordAccess(FASEQBlk *blk)
{
    for (int i = 0; i < numTrackedCaches; i++) {
        if (blk && ((1U << i) & blk->inCachesMask)) {
            hits[i]++;
        } else {
            misses[i]++;
        }
    }

    // Record stats for the actual cache too
    if (blk && blk->isValid()) {
        hits[numTrackedCaches]++;
    } else {
        misses[numTrackedCaches]++;
    }

    accesses++;
}


void
printMySize(std::ostream &stream, size_t size)
{
    static const char *SIZES[] = { "B", "kB", "MB", "GB", "TB", "ZB" };
    int div = 0;
    while (size >= 1024 && div < (sizeof SIZES / sizeof *SIZES)) {
        div++;
        size >>= 10;
    }
    stream << size << SIZES[div];
}


void
FASEQ::SPMTracking::regStats(std::string name)
{
    hits
        .init(numTrackedCaches + 1)
        .name(name + ".faseq_hits")
        .desc("The number of hits in each cache size.")
        ;
    misses
        .init(numTrackedCaches + 1)
        .name(name + ".faseq_misses")
        .desc("The number of misses in each cache size.")
        ;
    accesses
        .name(name + ".faseq_accesses")
        .desc("The number of accesses to the FA LRU cache.")
        ;

    for (unsigned i = 0; i < numTrackedCaches + 1; ++i) {
        std::stringstream size_str;
        printMySize(size_str, minTrackedSize << i);
        hits.subname(i, size_str.str());
        hits.subdesc(i, "Hits in a " + size_str.str() + " cache");
        misses.subname(i, size_str.str());
        misses.subdesc(i, "Misses in a " + size_str.str() + " cache");
    }
}
