/*
 * Copyright (c) 2012-2018 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *          Dave Greene
 *          Steve Reinhardt
 *          Ron Dreslinski
 *          Andreas Hansson
 */

/**
 * @file
 * Describes a cache
 */

#ifndef __MEM_CACHE_SPM_HH__
#define __MEM_CACHE_SPM_HH__

#include <cstdint>
#include <unordered_set>
#include <vector>
#include <map>
#include <queue>
#include <stdlib.h>
#include <time.h>
#include <list>

#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

//#define random(x) (rand() % x)

class CacheBlk;
struct SPMParams;
class MSHR;

/**
 * A coherent cache that can be arranged in flexible topologies.
 */
class SPM : public BaseCache
{

  protected:

    class CacheSidePort : public MasterPort
    {
      private:

        // a pointer to our specific SPM implementation
        SPM *spm;

      protected:

        virtual void recvTimingSnoopReq(PacketPtr pkt) override;

        virtual bool recvTimingResp(PacketPtr pkt) override;

        virtual Tick recvAtomicSnoop(PacketPtr pkt) override;

        virtual void recvFunctionalSnoop(PacketPtr pkt) override;

        virtual void recvReqRetry() override;
        
        virtual void recvRetrySnoopResp() override;
        
        virtual void recvRangeChange() override;
        
        virtual bool isSnooping() const override {
            return false;
        }

        virtual void recvDmaAction() override;

      public:

        CacheSidePort(const std::string &_name, SPM *_spm,
                      const std::string &_label);

    };

    CacheSidePort cacheSidePort;

  protected:
    /**
     * This cache should allocate a block on a line-sized write miss.
     */
    const bool doFastWrites;

    PacketPtr temp_pkt;

    //Flags temp_flag;
    //MasterID temp_masterid;

    bool re;

    //typedef std::pair<int, bool> pageCountState;
    //typedef std::pair<int, bool> PageState;
    typedef std::map<Addr, int> PageCount;
    typedef std::map<Addr, bool> PageState;

    /** 存放不同page的访问数目 */
    PageCount pageCount;

    PageCount pageAccess;
    /** 存放不同page的访问数目 */
    PageCount pageFreCount;
    /** 存放不同page的miss数目 */
    PageCount pageMissCount;
    PageCount pageHitCount;
    PageCount pageLineSize;
    /** 存放不同page是否映射到SPM中 */
    PageState pageState;

    /** page大小 */
    const unsigned pageShift;

    /** 记录上次触发page映射的时间 */
    Cycles temp_time;

    Cycles reset_time;

    /** 记录接收到cache line的数目 */
    unsigned recvPageNum;

    /** 当前映射的page地址 */
    Addr temp_page;

    /** 当前映射page地址的集合 */
    std::vector<Addr> movePage;

    /** 每个page中cache line的数目 */
    unsigned pageCacheLineNum;

    /** threshold value of the current period */
    signed th_cur;

    /** threshold value of the previous period */
    signed th_next;

    /** number of off-chip memory accesses in the previous period */
    unsigned ma_pre;

    /** number of off-chip memory accesses in the current period */
    unsigned ma_cur;

    /** total miss number */
    unsigned total_miss;

    unsigned th;

    unsigned timeGap;

    unsigned accessFre;

    unsigned accessTh;

    std::list<Addr> candidate;

    unsigned total_acc;

    unsigned monitor_page;

    unsigned monitor_miss;

    std::list<Addr> tlbPageNum;
	
	std::map<Addr, std::queue<int>> accHistory;


    std::vector<Addr> missHistory;
    std::vector<Addr> replHistory;

    std::map<Addr, int> page_threshold;

    Addr recently_page;

    int recently_th = 626;

    // Used to Q-learning parameters
    /**               spm free space  0--full  1--free
                      threshold value  0--0~3  1--4~7  2--8~15  3--16~31  4--32~63  5--64~
                      miss page numbers  0--0~7  1--8~15  2--15~31  3--32~63  4--64~128
                      replacement page numbers 0--0~7  1--8~15  2--16~31  3--32~63  4--64~128
                      actions  0--unchange  1--increase 1  2--reduce 1 */
    typedef std::map<unsigned, float> QValueTable;
    typedef std::map<unsigned, int> MissTable;

    /** Q-value table */
    QValueTable qValueTable;
    MissTable missTable;
    unsigned prevState = 0;
    unsigned prev_act = 0;

    /** reward  */
    float reward;

    int prev_miss;
    std::vector<int> missPage;

    int replPageCount = 0;

    int debug_cnt = 0;

    int my_plan;

    /**
      0-- Am0,   Am1,   Am2,   ..., Amk
      1-- Am0^n, Am1^n, Am2^n, ..., Amk^n
    */
    std::map<Addr, std::pair<int, float> > patternTable;

    std::map<Addr, std::pair<long, bool> > spmPageTypeTable;
    std::map<Addr, float> spmMissRateTable;
    std::map<Addr, float> spmAccCountTable;

    std::map<Addr, std::vector<int> > setTable;

    std::map<Addr, std::pair<long, bool> > pageTypeTable;

    std::vector<Addr> storeSeqPage;

    int miss_num = 0;

    std::vector<Addr> missPageNum;

    int record_miss = 0;

    std::map<Addr, int> pageAccState;

    int cnt_16 = 0;

    bool repl_type = false;

    std::vector<Addr> victPageTable;

    int low = 0;
    int high = 0;

    int N_repl = 0;
    int N_alloc = 0;

    int pre_repl = 0;
    int pre_alloc = 0;

    std::queue<PacketPtr> sendPackets;

    Tick curDMATick = 0;

    Tick dmaDelay = 0;

    int e_cache;
    int e_spm;
    int e_dram;





    /**
     * Store the outstanding requests that we are expecting snoop
     * responses from so we can determine which snoop responses we
     * generated and which ones were merely forwarded.
     */
    std::unordered_set<RequestPtr> outstandingSnoop;

    virtual bool handleTimingResp(PacketPtr pkt)
    {
        //handlePartitionWithRand(pkt);
        
        if (!isBlocked(Blocked_DMA)) {
          handlePartitionWithMiss(pkt);
        }
        
        
        return cpuSidePort.sendTimingResp(pkt);
    }

    virtual bool recvTimingReqSPM(PacketPtr pkt) override;

    virtual void handleTimingSnoopReq(PacketPtr pkt)
    {
        cpuSidePort.sendTimingSnoopReq(pkt);
    }

    virtual Tick handleAtomicSnoop(PacketPtr pkt)
    {
        return cpuSidePort.sendAtomicSnoop(pkt);
    }

    virtual void handleFunctionalSnoop(PacketPtr pkt)
    {
        cpuSidePort.sendFunctionalSnoop(pkt);
    }


    void handleReqRetry()
    {
        cpuSidePort.sendRetryReq();
    }

    void handleRetrySnoopResp()
    {
        cpuSidePort.sendRetrySnoopResp();
    }

    void sendRangeChange() const
    {
        //cpuSidePort.sendRangeChange();
    }

  protected:
    /**
     * Turn line-sized writes into WriteInvalidate transactions.
     */
    void promoteWholeLineWrites(PacketPtr pkt);

    bool access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                PacketList &writebacks) override;

    void handleTimingReqHit(PacketPtr pkt, CacheBlk *blk,
                            Tick request_time) override;

    void handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk,
                             Tick forward_time,
                             Tick request_time) override;

    void recvTimingReq(PacketPtr pkt) override;

    void doWritebacks(PacketList& writebacks, Tick forward_time) override;

    void doWritebacksAtomic(PacketList& writebacks) override;

    void serviceMSHRTargets(MSHR *mshr, const PacketPtr pkt,
                            CacheBlk *blk) override;

    void recvTimingSnoopReq(PacketPtr pkt) override;

    void recvTimingSnoopResp(PacketPtr pkt) override;

    Cycles handleAtomicReqMiss(PacketPtr pkt, CacheBlk *&blk,
                               PacketList &writebacks) override;

    Tick recvAtomic(PacketPtr pkt) override;

    Tick recvAtomicSnoop(PacketPtr pkt) override;

    void satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                        bool deferred_response = false,
                        bool pending_downgrade = false) override;

    void doTimingSupplyResponse(PacketPtr req_pkt, const uint8_t *blk_data,
                                bool already_copied, bool pending_inval);

    /**
     * Perform an upward snoop if needed, and update the block state
     * (possibly invalidating the block). Also create a response if required.
     *
     * @param pkt Snoop packet
     * @param blk Cache block being snooped
     * @param is_timing Timing or atomic for the response
     * @param is_deferred Is this a deferred snoop or not?
     * @param pending_inval Do we have a pending invalidation?
     *
     * @return The snoop delay incurred by the upwards snoop
     */
    uint32_t handleSnoop(PacketPtr pkt, CacheBlk *blk,
                         bool is_timing, bool is_deferred, bool pending_inval);

    M5_NODISCARD PacketPtr evictBlock(CacheBlk *blk) override;

    /**
     * Create a CleanEvict request for the given block.
     *
     * @param blk The block to evict.
     * @return The CleanEvict request for the block.
     */
    PacketPtr cleanEvictBlk(CacheBlk *blk);

    PacketPtr createMissPacket(PacketPtr cpu_pkt, CacheBlk *blk,
                               bool needs_writable,
                               bool is_whole_line_write) const override;

    /**
     * Send up a snoop request and find cached copies. If cached copies are
     * found, set the BLOCK_CACHED flag in pkt.
     */
    bool isCachedAbove(PacketPtr pkt, bool is_timing = true);

    Addr maxCount(PageCount p) {
        auto iter = p.begin();
        int max = 0;
        Addr page = 0;
        while (iter != p.end()) {
            if ((*iter).second > max) {
                page = (*iter).first;
            }
            iter++;
        }
        return page;
    }

    CacheBlk* allocateBlock(const PacketPtr pkt, PacketList &writebacks) override;

    void handleSendPacket(PacketPtr pkt, Addr paddr, int n=0);

    /**
     * 一定时间间隔内利用计数器记录不同page的访问次数，
     * 当访问次数达到一定阈值时，触发SPM数据映射。
     */
    void handlePartitionWithFre(PacketPtr pkt);

    /**
     * 记录不同page的miss数目（片外访存），达到一定阈值触发SPM映射
     */
    void handlePartitionWithMiss(PacketPtr pkt);

    /**
     * 动态调整阈值
     */
    void handleThresholdAdjust();

    bool handleAllocSpace(PacketPtr pkt) override;

    bool handlePatternReg(Addr page, int set);

    void handleDmaAction();



  public:

    BaseMasterPort &getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    /** Instantiates a basic cache object. */
    SPM(const SPMParams *p);

    /**
     * Take an MSHR, turn it into a suitable downstream packet, and
     * send it out. This construct allows a queue entry to choose a suitable
     * approach based on its type.
     *
     * @param mshr The MSHR to turn into a packet and send
     * @return True if the port is waiting for a retry
     */
    bool sendMSHRQueuePacket(MSHR* mshr) override;

};

#endif // __MEM_CACHE_CACHE_HH__
