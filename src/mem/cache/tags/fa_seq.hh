/*
 * Copyright (c) 2012-2013,2016,2018 ARM Limited
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
 */

/**
 * @file
 * Declaration of a fully associative seq tag store.
 */

#ifndef __MEM_CACHE_TAGS_FA_SEQ_HH__
#define __MEM_CACHE_TAGS_FA_SEQ_HH__

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/tags/base.hh"
#include "params/FASEQ.hh"

// Uncomment to enable sanity checks for the FASEQ cache and the
// TrackedCaches class
//#define FASEQ_DEBUG

class BaseCache;
class ReplaceableEntry;

// A bitmask of the caches we are keeping track of. Currently the
// lowest bit is the smallest cache we are tracking, as it is
// specified by the corresponding parameter. The rest of the bits are
// for exponentially growing cache sizes.
typedef uint32_t CachesMask;

/**
 * A fully associative cache block.
 */
class FASEQBlk : public CacheBlk
{
  public:
    FASEQBlk() : CacheBlk(), prev(nullptr), next(nullptr), inCachesMask(0) {}

    /** The previous block in LRU order. */
    FASEQBlk *prev;
    /** The next block in LRU order. */
    FASEQBlk *next;

    /** A bit mask of the caches that fit this block. */
    CachesMask inCachesMask;

    /**
     * Pretty-print inCachesMask and other CacheBlk information.
     *
     * @return string with basic state information
     */
    std::string print() const override;
};

/**
 * A fully associative LRU cache. Keeps statistics for accesses to a number of
 * cache sizes at once.
 */
class FASEQ : public BaseTags
{
  public:
    /** Typedef the block type used in this class. */
    typedef FASEQBlk BlkType;

  protected:
    /** The number of bits worth of in-page address */
    const int pageShift;

    /** The cache blocks. */
    FASEQBlk *blks;

    /** The MRU block. */
    FASEQBlk *head;
    /** The LRU block. */
    FASEQBlk *tail;

    FASEQBlk *initial;

    /** Hash table type mapping addresses to cache block pointers. */
    struct PairHash
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2> &p) const
        {
            return std::hash<T1>()(p.first) ^ std::hash<T2>()(p.second);
        }
    };
    typedef std::pair<Addr, bool> TagHashKey;
    typedef std::unordered_map<TagHashKey, FASEQBlk *, PairHash> TagHash;

    /** The address hash table. */
    TagHash tagHash;

    typedef std::map<Addr, std::vector<Addr> > PageHash;

    /** The page hash table. */
    PageHash pageHash; 

    /** */
    std::map<Addr, Cycles> pageAccessCount;

    /** the number of used blk */
    unsigned usedBlk = 0;

    /**
     * Move a cache block to the MRU position.
     *
     * @param blk The block to promote.
     */
    void moveToHead(FASEQBlk *blk);

    /**
     * Move a cache block to the LRU position.
     *
     * @param blk The block to demote.
     */
    void moveToTail(FASEQBlk *blk);

  public:
    typedef FASEQParams Params;

    /**
     * Construct and initialize this cache tagstore.
     */
    FASEQ(const Params *p);
    ~FASEQ();

    /**
     * the spm is full or not
     */
    bool isFull() { 
        //std::cout << usedBlk << " " << numBlocks << std::endl;
        return usedBlk == numBlocks; 
    }

    /**
     * Initialize blocks as FASEQBlk instances.
     */
    void tagsInit() override;

    /**
     * Register the stats for this object.
     */
    void regStats() override;

    /**
     * Invalidate a cache block.
     * @param blk The block to invalidate.
     */
    void invalidate(CacheBlk *blk) override;

    void setUnused(CacheBlk *blk) override;

    void setUsed(CacheBlk *blk) override;

    /**
     * Access block and update replacement data.  May not succeed, in which
     * case nullptr pointer is returned.  This has all the implications of a
     * cache access and should only be used as such.
     * Returns tag lookup latency and the inCachesMask flags as a side effect.
     *
     * @param addr The address to look for.
     * @param is_secure True if the target memory space is secure.
     * @param lat The latency of the tag lookup.
     * @param in_cache_mask Mask indicating the caches in which the blk fits.
     * @return Pointer to the cache block.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat,
                          CachesMask *in_cache_mask);

    /**
     * Just a wrapper of above function to conform with the base interface.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat) override;

    /**
     * Find the block in the cache, do not update the replacement data.
     * @param addr The address to look for.
     * @param is_secure True if the target memory space is secure.
     * @param asid The address space ID.
     * @return Pointer to the cache block.
     */
    CacheBlk* findBlock(Addr addr, bool is_secure) const override;

    CacheBlk* findLastVictim(Addr addr, bool is_secure);

    /**
     * Find a block given set and way.
     *
     * @param set The set of the block.
     * @param way The way of the block.
     * @return The block.
     */
    ReplaceableEntry* findBlockBySetAndWay(int set, int way) const override;

    /**
     * Find replacement victim based on address. The list of evicted blocks
     * only contains the victim.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param evict_blks Cache blocks to be evicted.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const override;

    CacheBlk* findUsed(Addr addr,
                         std::vector<CacheBlk*>& evict_blks) override;

    CacheBlk* findUnused(Addr addr,
                         std::vector<CacheBlk*>& evict_blks) override;

    /**
     * Insert the new block into the cache and update replacement data.
     *
     * @param addr Address of the block.
     * @param is_secure Whether the block is in secure space or not.
     * @param src_master_ID The source requestor ID.
     * @param task_ID The new task ID.
     * @param blk The block to update.
     */
    void insertBlock(const Addr addr, const bool is_secure,
                     const int src_master_ID, const uint32_t task_ID,
                     CacheBlk *blk) override;

    /**
     * Generate the tag from the addres. For fully associative this is just the
     * block address.
     * @param addr The address to get the tag from.
     * @return The tag.
     */
    Addr extractTag(Addr addr) const override
    {
        return blkAlign(addr);
    }

    /**
     * Regenerate the block address from the tag.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override
    {
        return blk->tag;
    }

    void forEachBlk(std::function<void(CacheBlk &)> visitor) override {
        for (int i = 0; i < numBlocks; i++) {
            visitor(blks[i]);
        }
    }

    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override {
        for (int i = 0; i < numBlocks; i++) {
            if (visitor(blks[i])) {
                return true;
            }
        }
        return false;
    }

    Cycles getPageAccessCount(Addr page) override {
        return pageAccessCount[page];
    }

    void resetPageAccessCount() override {
        // for (auto iter = pageAccessCount.begin(); iter != pageAccessCount.end(); ++iter) {
        //     pageAccessCount[iter->first] = ticksToCycles(curTick());
        // }
        for (int i = 0; i < 16; i++) {
            //FASEQBlk* temp = head;
            moveToTail(head);
        }
    }

  private:
    /**
     * Mechanism that allows us to simultaneously collect miss
     * statistics for multiple caches. Currently, we keep track of
     * caches from a set minimum size of interest up to the actual
     * cache size.
     */
    class SPMTracking
    {
      public:
        SPMTracking(unsigned min_size, unsigned max_size,
                      unsigned block_size)
            : blkSize(block_size),
              minTrackedSize(min_size),
              numTrackedCaches(max_size > min_size ?
                               floorLog2(max_size) - floorLog2(min_size) : 0),
              inAllCachesMask(mask(numTrackedCaches)),
              boundaries(numTrackedCaches)
        {
            fatal_if(numTrackedCaches > sizeof(CachesMask) * 8,
                     "Not enough bits (%s) in type CachesMask type to keep "
                     "track of %d caches\n", sizeof(CachesMask),
                     numTrackedCaches);
        }

        /**
         * Initialiaze cache blocks and the tracking mechanism
         *
         * All blocks in the cache need to be initialized once.
         *
         * @param blk the MRU block
         * @param blk the LRU block
         */
        void init(FASEQBlk *head, FASEQBlk *tail);

        /**
         * Update boundaries as a block will be moved to the MRU.
         *
         * For all caches that didn't fit the block before moving it,
         * we move their boundaries one block closer to the MRU. We
         * also update InCacheMasks as neccessary.
         *
         * @param blk the block that will be moved to the head
         */
        void moveBlockToHead(FASEQBlk *blk);

        /**
         * Update boundaries as a block will be moved to the LRU.
         *
         * For all caches that fitted the block before moving it, we
         * move their boundaries one block closer to the LRU. We
         * also update InCacheMasks as neccessary.
         *
         * @param blk the block that will be moved to the head
         */
        void moveBlockToTail(FASEQBlk *blk);

        /**
         * Notify of a block access.
         *
         * This should be called every time a block is accessed and it
         * updates statistics. If the input block is nullptr then we
         * treat the access as a miss. The block's InCacheMask
         * determines the caches in which the block fits.
         *
         * @param blk the block to record the access for
         */
        void recordAccess(FASEQBlk *blk);

        /**
         * Check that the tracking mechanism is in consistent state.
         *
         * Iterate from the head (MRU) to the tail (LRU) of the list
         * of blocks and assert the inCachesMask and the boundaries
         * are in consistent state.
         *
         * @param head the MRU block of the actual cache
         * @param head the LRU block of the actual cache
         */
        void check(const FASEQBlk *head, const FASEQBlk *tail) const;

        /**
         * Register the stats for this object.
         */
        void regStats(std::string name);

      private:
        /** The size of the cache block */
        const unsigned blkSize;
        /** The smallest cache we are tracking */
        const unsigned minTrackedSize;
        /** The number of different size caches being tracked. */
        const int numTrackedCaches;
        /** A mask for all cache being tracked. */
        const CachesMask inAllCachesMask;
        /** Array of pointers to blocks at the cache boundaries. */
        std::vector<FASEQBlk*> boundaries;

      protected:
        /**
         * @defgroup FASEQStats Fully Associative LRU specific statistics
         * The FA lru stack lets us track multiple cache sizes at once. These
         * statistics track the hits and misses for different cache sizes.
         * @{
         */

        /** Hits in each cache */
        Stats::Vector hits;
        /** Misses in each cache */
        Stats::Vector misses;
        /** Total number of accesses */
        Stats::Scalar accesses;

        /**
         * @}
         */
    };
    SPMTracking spmTracking;
};

#endif // __MEM_CACHE_TAGS_FA_SEQ_HH__