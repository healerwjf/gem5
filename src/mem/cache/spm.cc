/*
 * Copyright (c) 2010-2018 ARM Limited
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
 * Copyright (c) 2010,2015 Advanced Micro Devices, Inc.
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
 *          Nathan Binkert
 *          Steve Reinhardt
 *          Ron Dreslinski
 *          Andreas Sandberg
 *          Nikos Nikoleris
 */

/**
 * @file
 * Cache definitions.
 */

#include "mem/cache/spm.hh"

#include <cassert>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <bitset>

#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/SPM.hh"
#include "debug/SPMTags.hh"
#include "debug/SPMVerbose.hh"
#include "debug/SPMRepl.hh"
#include "enums/Clusivity.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/write_queue_entry.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/request.hh"
#include "params/SPM.hh"

SPM::SPM(const SPMParams *p)
    : BaseCache(p, p->system->cacheLineSize()),
      cacheSidePort(p->name + ".cache_side", this, "CacheSidePort"),
      doFastWrites(true),
      temp_pkt(nullptr),
      re(true),
      pageShift(p->pageShift),
      temp_time(0),
      recvPageNum(0),
      temp_page(0),
      th_cur(64), th_next(32), 
      ma_pre(0), ma_cur(0),
      total_miss(0),
      th(p->th),
      timeGap(p->timeGap),
      accessFre(p->accessFre),
      accessTh(p->accessTh),
      my_plan(p->myPlan),
      e_cache(p->eCache),
      e_spm(p->eSPM),
      e_dram(p->eDRAM)
{
    pageCacheLineNum = 1 << (pageShift - 6);
    srand((unsigned)time(NULL));
    setIsSPM();
}

BaseMasterPort &
SPM::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cache_side") {
        return cacheSidePort;
    }  else {
        return BaseCache::getMasterPort(if_name, idx);
    }
}

void
SPM::satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                      bool deferred_response, bool pending_downgrade)
{
    BaseCache::satisfyRequest(pkt, blk);

    if (pkt->isRead()) {
        // determine if this read is from a (coherent) cache or not
        if (pkt->fromCache()) {
            assert(pkt->getSize() == blkSize);
            // special handling for coherent block requests from
            // upper-level caches
            if (pkt->needsWritable()) {
                // sanity check
                assert(pkt->cmd == MemCmd::ReadExReq ||
                       pkt->cmd == MemCmd::SCUpgradeFailReq);
                assert(!pkt->hasSharers());

                // if we have a dirty copy, make sure the recipient
                // keeps it marked dirty (in the modified state)
                if (blk->isDirty()) {
                    pkt->setCacheResponding();
                    blk->status &= ~BlkDirty;
                }
            } else if (blk->isWritable() && !pending_downgrade &&
                       !pkt->hasSharers() &&
                       pkt->cmd != MemCmd::ReadCleanReq) {
                // we can give the requester a writable copy on a read
                // request if:
                // - we have a writable copy at this level (& below)
                // - we don't have a pending snoop from below
                //   signaling another read request
                // - no other cache above has a copy (otherwise it
                //   would have set hasSharers flag when
                //   snooping the packet)
                // - the read has explicitly asked for a clean
                //   copy of the line
                if (blk->isDirty()) {
                    // special considerations if we're owner:
                    if (!deferred_response) {
                        // respond with the line in Modified state
                        // (cacheResponding set, hasSharers not set)
                        pkt->setCacheResponding();

                        // if this cache is mostly inclusive, we
                        // keep the block in the Exclusive state,
                        // and pass it upwards as Modified
                        // (writable and dirty), hence we have
                        // multiple caches, all on the same path
                        // towards memory, all considering the
                        // same block writable, but only one
                        // considering it Modified

                        // we get away with multiple caches (on
                        // the same path to memory) considering
                        // the block writeable as we always enter
                        // the cache hierarchy through a cache,
                        // and first snoop upwards in all other
                        // branches
                        blk->status &= ~BlkDirty;
                    } else {
                        // if we're responding after our own miss,
                        // there's a window where the recipient didn't
                        // know it was getting ownership and may not
                        // have responded to snoops correctly, so we
                        // have to respond with a shared line
                        pkt->setHasSharers();
                    }
                }
            } else {
                // otherwise only respond with a shared copy
                pkt->setHasSharers();
            }
        }
    }
}

/////////////////////////////////////////////////////
//
// Access path: requests coming in from the CPU side
//
/////////////////////////////////////////////////////

bool
SPM::access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
              PacketList &writebacks)
{

    if (pkt->req->isUncacheable()) {
        assert(pkt->isRequest());

        chatty_assert(!(isReadOnly && pkt->isWrite()),
                      "Should never see a write in a read-only cache %s\n",
                      name());

        DPRINTF(SPM, "%s for %s\n", __func__, pkt->print());

        // flush and invalidate any existing block
        CacheBlk *old_blk(tags->findBlock(pkt->getAddr(), pkt->isSecure()));
        if (old_blk && old_blk->isValid()) {
            BaseCache::evictBlock(old_blk, writebacks);
        }

        blk = nullptr;
        // lookupLatency is the latency in case the request is uncacheable.
        lat = lookupLatency;
        return false;
    }

    return BaseCache::access(pkt, blk, lat, writebacks);
}

void
SPM::doWritebacks(PacketList& writebacks, Tick forward_time)
{
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
        // We use forwardLatency here because we are copying writebacks to
        // write buffer.

        // Call isCachedAbove for Writebacks, CleanEvicts and
        // WriteCleans to discover if the block is cached above.
        if (isCachedAbove(wbPkt)) {
            if (wbPkt->cmd == MemCmd::CleanEvict) {
                // Delete CleanEvict because cached copies exist above. The
                // packet destructor will delete the request object because
                // this is a non-snoop request packet which does not require a
                // response.
                delete wbPkt;
            } else if (wbPkt->cmd == MemCmd::WritebackClean) {
                // clean writeback, do not send since the block is
                // still cached above
                assert(writebackClean);
                delete wbPkt;
            } else {
                assert(wbPkt->cmd == MemCmd::WritebackDirty ||
                       wbPkt->cmd == MemCmd::WriteClean);
                // Set BLOCK_CACHED flag in Writeback and send below, so that
                // the Writeback does not reset the bit corresponding to this
                // address in the snoop filter below.
                wbPkt->setBlockCached();
                allocateWriteBuffer(wbPkt, forward_time);
            }
        } else {
            // If the block is not cached above, send packet below. Both
            // CleanEvict and Writeback with BLOCK_CACHED flag cleared will
            // reset the bit corresponding to this address in the snoop filter
            // below.
            allocateWriteBuffer(wbPkt, forward_time);
        }
        writebacks.pop_front();
    }

    if (recvPageNum != 0)
        DPRINTF(SPMRepl, "writeBuffer is empty: %d\n", writeBuffer.theSize());
}

void
SPM::doWritebacksAtomic(PacketList& writebacks)
{
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
        // Call isCachedAbove for both Writebacks and CleanEvicts. If
        // isCachedAbove returns true we set BLOCK_CACHED flag in Writebacks
        // and discard CleanEvicts.
        if (isCachedAbove(wbPkt, false)) {
            if (wbPkt->cmd == MemCmd::WritebackDirty ||
                wbPkt->cmd == MemCmd::WriteClean) {
                // Set BLOCK_CACHED flag in Writeback and send below,
                // so that the Writeback does not reset the bit
                // corresponding to this address in the snoop filter
                // below. We can discard CleanEvicts because cached
                // copies exist above. Atomic mode isCachedAbove
                // modifies packet to set BLOCK_CACHED flag
                memSidePort.sendAtomic(wbPkt);
            }
        } else {
            // If the block is not cached above, send packet below. Both
            // CleanEvict and Writeback with BLOCK_CACHED flag cleared will
            // reset the bit corresponding to this address in the snoop filter
            // below.
            memSidePort.sendAtomic(wbPkt);
        }
        writebacks.pop_front();
        // In case of CleanEvicts, the packet destructor will delete the
        // request object because this is a non-snoop request packet which
        // does not require a response.
        delete wbPkt;
    }
}


void
SPM::recvTimingSnoopResp(PacketPtr pkt)
{
    DPRINTF(SPM, "%s for %s\n", __func__, pkt->print());

    // determine if the response is from a snoop request we created
    // (in which case it should be in the outstandingSnoop), or if we
    // merely forwarded someone else's snoop request
    const bool forwardAsSnoop = outstandingSnoop.find(pkt->req) ==
        outstandingSnoop.end();

    if (!forwardAsSnoop) {
        // the packet came from this cache, so sink it here and do not
        // forward it
        assert(pkt->cmd == MemCmd::HardPFResp);

        outstandingSnoop.erase(pkt->req);

        DPRINTF(SPM, "Got prefetch response from above for addr "
                "%#llx (%s)\n", pkt->getAddr(), pkt->isSecure() ? "s" : "ns");
        recvTimingResp(pkt);
        return;
    }

    // forwardLatency is set here because there is a response from an
    // upper level cache.
    // To pay the delay that occurs if the packet comes from the bus,
    // we charge also headerDelay.
    Tick snoop_resp_time = clockEdge(forwardLatency) + pkt->headerDelay;
    // Reset the timing of the packet.
    pkt->headerDelay = pkt->payloadDelay = 0;
    memSidePort.schedTimingSnoopResp(pkt, snoop_resp_time);
}

void
SPM::promoteWholeLineWrites(PacketPtr pkt)
{
    // Cache line clearing instructions
    if (doFastWrites && (pkt->cmd == MemCmd::WriteReq) &&
        (pkt->getSize() == blkSize) && (pkt->getOffset(blkSize) == 0)) {
        pkt->cmd = MemCmd::WriteLineReq;
        DPRINTF(SPM, "packet promoted from Write to WriteLineReq\n");
    }
}

void
SPM::handleTimingReqHit(PacketPtr pkt, CacheBlk *blk, Tick request_time)
{
    // should never be satisfying an uncacheable access as we
    // flush and invalidate any existing block as part of the
    // lookup
    assert(!pkt->req->isUncacheable());

    BaseCache::handleTimingReqHit(pkt, blk, request_time);
}

void
SPM::handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk, Tick forward_time,
                           Tick request_time)
{
    if (pkt->req->isUncacheable()) {
        // ignore any existing MSHR if we are dealing with an
        // uncacheable request

        // should have flushed and have no valid block
        assert(!blk || !blk->isValid());

        mshr_uncacheable[pkt->cmdToIndex()][pkt->req->masterId()]++;

        if (pkt->isWrite()) {
            allocateWriteBuffer(pkt, forward_time);
        } else {
            assert(pkt->isRead());

            // uncacheable accesses always allocate a new MSHR

            // Here we are using forward_time, modelling the latency of
            // a miss (outbound) just as forwardLatency, neglecting the
            // lookupLatency component.
            allocateMissBuffer(pkt, forward_time);
        }

        return;
    }

    Addr blk_addr = pkt->getBlockAddr(blkSize);

    MSHR *mshr = mshrQueue.findMatch(blk_addr, pkt->isSecure());

    // Software prefetch handling:
    // To keep the core from waiting on data it won't look at
    // anyway, send back a response with dummy data. Miss handling
    // will continue asynchronously. Unfortunately, the core will
    // insist upon freeing original Packet/Request, so we have to
    // create a new pair with a different lifecycle. Note that this
    // processing happens before any MSHR munging on the behalf of
    // this request because this new Request will be the one stored
    // into the MSHRs, not the original.

    BaseCache::handleTimingReqMiss(pkt, mshr, blk, forward_time, request_time);
}

bool
SPM::recvTimingReqSPM(PacketPtr pkt)
{

    if (isBlocked()) {
        cpuSidePort.tryTiming(pkt);
        return false;
    } else {
        bool recvSuccess = false;

        handleThresholdAdjust();

        promoteWholeLineWrites(pkt);

        if (pkt->cacheResponding()) {
            // a cache above us (but not where the packet came from) is
            // responding to the request, in other words it has the line
            // in Modified or Owned state
            DPRINTF(SPM, "Cache above responding to %s: not responding\n",
                    pkt->print());

            // if the packet needs the block to be writable, and the cache
            // that has promised to respond (setting the cache responding
            // flag) is not providing writable (it is in Owned rather than
            // the Modified state), we know that there may be other Shared
            // copies in the system; go out and invalidate them all
            assert(pkt->needsWritable() && !pkt->responderHadWritable());

            // an upstream cache that had the line in Owned state
            // (dirty, but not writable), is responding and thus
            // transferring the dirty line from one branch of the
            // cache hierarchy to another

            // send out an express snoop and invalidate all other
            // copies (snooping a packet that needs writable is the
            // same as an invalidation), thus turning the Owned line
            // into a Modified line, note that we don't invalidate the
            // block in the current cache or any other cache on the
            // path to memory

            // create a downstream express snoop with cleared packet
            // flags, there is no need to allocate any data as the
            // packet is merely used to co-ordinate state transitions
            Packet *snoop_pkt = new Packet(pkt, true, false);

            // also reset the bus time that the original packet has
            // not yet paid for
            snoop_pkt->headerDelay = snoop_pkt->payloadDelay = 0;

            // make this an instantaneous express snoop, and let the
            // other caches in the system know that the another cache
            // is responding, because we have found the authorative
            // copy (Modified or Owned) that will supply the right
            // data
            snoop_pkt->setExpressSnoop();
            snoop_pkt->setCacheResponding();

            // this express snoop travels towards the memory, and at
            // every crossbar it is snooped upwards thus reaching
            // every cache in the system
            bool M5_VAR_USED success = memSidePort.sendTimingReq(snoop_pkt);
            // express snoops always succeed
            assert(success);

            // main memory will delete the snoop packet

            // queue for deletion, as opposed to immediate deletion, as
            // the sending cache is still relying on the packet
            pendingDelete.reset(pkt);

            // no need to take any further action in this particular cache
            // as an upstram cache has already committed to responding,
            // and we have already sent out any express snoops in the
            // section above to ensure all other copies in the system are
            // invalidated
            return true;
        }

    // anything that is merely forwarded pays for the forward latency and
    // the delay provided by the crossbar
    Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;

    Cycles lat;
    CacheBlk *blk = nullptr;
    bool satisfied = false;
    {
        PacketList writebacks;
        // Note that lat is passed by reference here. The function
        // access() will set the lat value.
        satisfied = access(pkt, blk, lat, writebacks);

        // copy writebacks to write buffer here to ensure they logically
        // precede anything happening below
        doWritebacks(writebacks, forward_time);
    }

    // Here we charge the headerDelay that takes into account the latencies
    // of the bus, if the packet comes from it.
    // The latency charged is just the value set by the access() function.
    // In case of a hit we are neglecting response latency.
    // In case of a miss we are neglecting forward latency.
    Tick request_time = clockEdge(lat) + pkt->headerDelay;
    // Here we reset the timing of the packet.
    pkt->headerDelay = pkt->payloadDelay = 0;
    
    if (!satisfied && isBlocked() && pkt->getAddr() >> pageShift == temp_page) {
        DPRINTF(Cache, "satisfied: %d.\n", satisfied);
        cpuSidePort.tryTiming(pkt);
        return false;
    } else if (satisfied) {
        // notify before anything else as later handleTimingReqHit might turn
        // the packet in a response
        ppHit->notify(pkt);

        pkt->headerDelay = pkt->payloadDelay = 0;

        if (prefetcher && blk && blk->wasPrefetched()) {
            blk->status &= ~BlkHWPrefetched;
        }
        spmAccCountTable[pkt->getAddr() >> pageShift]++;

        recvSuccess = true;

        handleTimingReqHit(pkt, blk, request_time);

    } else {
        if (cacheSidePort.sendTimingReq(pkt)) {
            recvSuccess = true;
        }
        ppMiss->notify(pkt);
    }

    // if (prefetcher) {
    //     // track time of availability of next prefetch, if any
    //     Tick next_pf_time = prefetcher->nextPrefetchReadyTime();
    //     if (next_pf_time != MaxTick) {
    //         schedMemSideSendEvent(next_pf_time);
    //     }
    // }
    return recvSuccess;
    }

}




void
SPM::recvTimingReq(PacketPtr pkt)
{

    promoteWholeLineWrites(pkt);

    if (pkt->cacheResponding()) {
        // a cache above us (but not where the packet came from) is
        // responding to the request, in other words it has the line
        // in Modified or Owned state
        DPRINTF(SPM, "Cache above responding to %s: not responding\n",
                pkt->print());

        // if the packet needs the block to be writable, and the cache
        // that has promised to respond (setting the cache responding
        // flag) is not providing writable (it is in Owned rather than
        // the Modified state), we know that there may be other Shared
        // copies in the system; go out and invalidate them all
        assert(pkt->needsWritable() && !pkt->responderHadWritable());

        // an upstream cache that had the line in Owned state
        // (dirty, but not writable), is responding and thus
        // transferring the dirty line from one branch of the
        // cache hierarchy to another

        // send out an express snoop and invalidate all other
        // copies (snooping a packet that needs writable is the
        // same as an invalidation), thus turning the Owned line
        // into a Modified line, note that we don't invalidate the
        // block in the current cache or any other cache on the
        // path to memory

        // create a downstream express snoop with cleared packet
        // flags, there is no need to allocate any data as the
        // packet is merely used to co-ordinate state transitions
        Packet *snoop_pkt = new Packet(pkt, true, false);

        // also reset the bus time that the original packet has
        // not yet paid for
        snoop_pkt->headerDelay = snoop_pkt->payloadDelay = 0;

        // make this an instantaneous express snoop, and let the
        // other caches in the system know that the another cache
        // is responding, because we have found the authorative
        // copy (Modified or Owned) that will supply the right
        // data
        snoop_pkt->setExpressSnoop();
        snoop_pkt->setCacheResponding();

        // this express snoop travels towards the memory, and at
        // every crossbar it is snooped upwards thus reaching
        // every cache in the system
        bool M5_VAR_USED success = memSidePort.sendTimingReq(snoop_pkt);
        // express snoops always succeed
        assert(success);

        // main memory will delete the snoop packet

        // queue for deletion, as opposed to immediate deletion, as
        // the sending cache is still relying on the packet
        pendingDelete.reset(pkt);

        // no need to take any further action in this particular cache
        // as an upstram cache has already committed to responding,
        // and we have already sent out any express snoops in the
        // section above to ensure all other copies in the system are
        // invalidated
        return;
    }

    BaseCache::recvTimingReq(pkt);
};

PacketPtr
SPM::createMissPacket(PacketPtr cpu_pkt, CacheBlk *blk,
                        bool needsWritable,
                        bool is_whole_line_write) const
{
    // should never see evictions here
    assert(!cpu_pkt->isEviction());

    bool blkValid = blk && blk->isValid();

    if (cpu_pkt->req->isUncacheable() ||
        (!blkValid && cpu_pkt->isUpgrade()) ||
        cpu_pkt->cmd == MemCmd::InvalidateReq || cpu_pkt->isClean()) {
        // uncacheable requests and upgrades from upper-level caches
        // that missed completely just go through as is
        return nullptr;
    }

    assert(cpu_pkt->needsResponse());

    MemCmd cmd;
    // @TODO make useUpgrades a parameter.
    // Note that ownership protocols require upgrade, otherwise a
    // write miss on a shared owned block will generate a ReadExcl,
    // which will clobber the owned copy.
    const bool useUpgrades = true;
    assert(cpu_pkt->cmd != MemCmd::WriteLineReq || is_whole_line_write);
    if (is_whole_line_write) {
        assert(!blkValid || !blk->isWritable());
        // forward as invalidate to all other caches, this gives us
        // the line in Exclusive state, and invalidates all other
        // copies
        cmd = MemCmd::InvalidateReq;
    } else if (blkValid && useUpgrades) {
        // only reason to be here is that blk is read only and we need
        // it to be writable
        assert(needsWritable);
        assert(!blk->isWritable());
        cmd = cpu_pkt->isLLSC() ? MemCmd::SCUpgradeReq : MemCmd::UpgradeReq;
    } else if (cpu_pkt->cmd == MemCmd::SCUpgradeFailReq ||
               cpu_pkt->cmd == MemCmd::StoreCondFailReq) {
        // Even though this SC will fail, we still need to send out the
        // request and get the data to supply it to other snoopers in the case
        // where the determination the StoreCond fails is delayed due to
        // all caches not being on the same local bus.
        cmd = MemCmd::SCUpgradeFailReq;
    } else {
        // block is invalid

        // If the request does not need a writable there are two cases
        // where we need to ensure the response will not fetch the
        // block in dirty state:
        // * this cache is read only and it does not perform
        //   writebacks,
        // * this cache is mostly exclusive and will not fill (since
        //   it does not fill it will have to writeback the dirty data
        //   immediately which generates uneccesary writebacks).
        bool force_clean_rsp = isReadOnly || clusivity == Enums::mostly_excl;
        needsWritable = true;
        cmd = needsWritable ? MemCmd::ReadExReq :
            (force_clean_rsp ? MemCmd::ReadCleanReq : MemCmd::ReadSharedReq);
    }
    PacketPtr pkt = new Packet(cpu_pkt->req, cmd, blkSize);

    // if there are upstream caches that have already marked the
    // packet as having sharers (not passing writable), pass that info
    // downstream
    if (cpu_pkt->hasSharers() && !needsWritable) {
        // note that cpu_pkt may have spent a considerable time in the
        // MSHR queue and that the information could possibly be out
        // of date, however, there is no harm in conservatively
        // assuming the block has sharers
        pkt->setHasSharers();
        DPRINTF(SPM, "%s: passing hasSharers from %s to %s\n",
                __func__, cpu_pkt->print(), pkt->print());
    }

    // the packet should be block aligned
    assert(pkt->getAddr() == pkt->getBlockAddr(blkSize));

    pkt->allocate();
    DPRINTF(SPM, "%s: created %s from %s\n", __func__, pkt->print(),
            cpu_pkt->print());
    return pkt;
}


Cycles
SPM::handleAtomicReqMiss(PacketPtr pkt, CacheBlk *&blk,
                           PacketList &writebacks)
{
    // deal with the packets that go through the write path of
    // the cache, i.e. any evictions and writes
    if (pkt->isEviction() || pkt->cmd == MemCmd::WriteClean ||
        (pkt->req->isUncacheable() && pkt->isWrite())) {
        Cycles latency = ticksToCycles(memSidePort.sendAtomic(pkt));

        // at this point, if the request was an uncacheable write
        // request, it has been satisfied by a memory below and the
        // packet carries the response back
        assert(!(pkt->req->isUncacheable() && pkt->isWrite()) ||
               pkt->isResponse());

        return latency;
    }

    // only misses left

    PacketPtr bus_pkt = createMissPacket(pkt, blk, true/*pkt->needsWritable()*/,
                                         pkt->isWholeLineWrite(blkSize));

    bool is_forward = (bus_pkt == nullptr);

    if (is_forward) {
        // just forwarding the same request to the next level
        // no local cache operation involved
        bus_pkt = pkt;
    }

    DPRINTF(SPM, "%s: Sending an atomic %s\n", __func__,
            bus_pkt->print());

#if TRACING_ON
    CacheBlk::State old_state = blk ? blk->status : 0;
#endif

    Cycles latency = ticksToCycles(memSidePort.sendAtomic(bus_pkt));

    bool is_invalidate = bus_pkt->isInvalidate();

    // We are now dealing with the response handling
    DPRINTF(SPM, "%s: Receive response: %s in state %i\n", __func__,
            bus_pkt->print(), old_state);

    // If packet was a forward, the response (if any) is already
    // in place in the bus_pkt == pkt structure, so we don't need
    // to do anything.  Otherwise, use the separate bus_pkt to
    // generate response to pkt and then delete it.
    if (!is_forward) {
        if (pkt->needsResponse()) {
            assert(bus_pkt->isResponse());
            if (bus_pkt->isError()) {
                pkt->makeAtomicResponse();
                pkt->copyError(bus_pkt);
            } else if (pkt->isWholeLineWrite(blkSize)) {
                // note the use of pkt, not bus_pkt here.

                // write-line request to the cache that promoted
                // the write to a whole line
                const bool allocate = allocOnFill(pkt->cmd) &&
                    (!writeAllocator || writeAllocator->allocate());
                blk = handleFill(bus_pkt, blk, writebacks, allocate);
                assert(blk != NULL);
                is_invalidate = false;
                satisfyRequest(pkt, blk);
            } else if (bus_pkt->isRead() ||
                       bus_pkt->cmd == MemCmd::UpgradeResp) {
                // we're updating cache state to allow us to
                // satisfy the upstream request from the cache
                blk = handleFill(bus_pkt, blk, writebacks,
                                 allocOnFill(pkt->cmd));
                satisfyRequest(pkt, blk);
                maintainClusivity(pkt->fromCache(), blk);
            } else {
                // we're satisfying the upstream request without
                // modifying cache state, e.g., a write-through
                pkt->makeAtomicResponse();
            }
        }
        delete bus_pkt;
    }

    if (is_invalidate && blk && blk->isValid()) {
        invalidateBlock(blk);
    }

    return latency;
}

Tick
SPM::recvAtomic(PacketPtr pkt)
{
    promoteWholeLineWrites(pkt);

    // follow the same flow as in recvTimingReq, and check if a cache
    // above us is responding
    if (pkt->cacheResponding()) {
        assert(!pkt->req->isCacheInvalidate());
        DPRINTF(SPM, "Cache above responding to %s: not responding\n",
                pkt->print());

        // if a cache is responding, and it had the line in Owned
        // rather than Modified state, we need to invalidate any
        // copies that are not on the same path to memory
        assert(pkt->needsWritable() && !pkt->responderHadWritable());

        return memSidePort.sendAtomic(pkt);
    }

    return BaseCache::recvAtomic(pkt);
}


/////////////////////////////////////////////////////
//
// Response handling: responses from the memory side
//
/////////////////////////////////////////////////////


void
SPM::serviceMSHRTargets(MSHR *mshr, const PacketPtr pkt, CacheBlk *blk)
{
    MSHR::Target *initial_tgt = mshr->getTarget();
    // First offset for critical word first calculations
    const int initial_offset = initial_tgt->pkt->getOffset(blkSize);

    const bool is_error = pkt->isError();
    // allow invalidation responses originating from write-line
    // requests to be discarded
    bool is_invalidate = pkt->isInvalidate() &&
        !mshr->wasWholeLineWrite;

    MSHR::TargetList targets = mshr->extractServiceableTargets(pkt);
    for (auto &target: targets) {
        Packet *tgt_pkt = target.pkt;
        switch (target.source) {
          case MSHR::Target::FromCPU:
            Tick completion_time;
            // Here we charge on completion_time the delay of the xbar if the
            // packet comes from it, charged on headerDelay.
            completion_time = pkt->headerDelay;

            // Software prefetch handling for cache closest to core
            if (tgt_pkt->cmd.isSWPrefetch()) {
                // a software prefetch would have already been ack'd
                // immediately with dummy data so the core would be able to
                // retire it. This request completes right here, so we
                // deallocate it.
                delete tgt_pkt;
                if (isBlocked(Blocked_DMA)) {
                    if ((recvPageNum == pageCacheLineNum - 1)) { // pageCacheLineNum
                        //if (repl_type)
                            // tags->resetPageAccessCount();
                        repl_type = false;
                        clearBlocked(Blocked_DMA);
                        tags->insert_page = false;
                        // tags->resetPageAccessCount();
                        mshr_miss_latency[tgt_pkt->cmdToIndex()][tgt_pkt->req->masterId()] +=
                            dmaDelay;
                        DPRINTF(SPMRepl, "%s: recvPageNum: %d, pageCacheLineNum: %d.\n", __func__, recvPageNum, pageCacheLineNum);
                        recvPageNum = 0;
                    } else if (recvPageNum < pageCacheLineNum - 1) {
                        DPRINTF(SPMRepl, "%s: recvPageNum: %d, pageCacheLineNum: %d.\n", __func__, recvPageNum, pageCacheLineNum);
                        recvPageNum++;
                        if (!sendPackets.empty()) {
                            PacketPtr temp = sendPackets.front();
                            sendPackets.pop();
                            recvTimingReq(temp);
                            // printf("sendPacket\n");
                        }
                    }
                }
                break; // skip response
            }

            // unlike the other packet flows, where data is found in other
            // caches or memory and brought back, write-line requests always
            // have the data right away, so the above check for "is fill?"
            // cannot actually be determined until examining the stored MSHR
            // state. We "catch up" with that logic here, which is duplicated
            // from above.
            if (tgt_pkt->cmd == MemCmd::WriteLineReq) {
                assert(!is_error);
                assert(blk);
                assert(blk->isWritable());
            }

            if (blk && blk->isValid() && !mshr->isForward) {
                satisfyRequest(tgt_pkt, blk, true, mshr->hasPostDowngrade());

                // How many bytes past the first request is this one
                int transfer_offset =
                    tgt_pkt->getOffset(blkSize) - initial_offset;
                if (transfer_offset < 0) {
                    transfer_offset += blkSize;
                }

                // If not critical word (offset) return payloadDelay.
                // responseLatency is the latency of the return path
                // from lower level caches/memory to an upper level cache or
                // the core.
                completion_time += clockEdge(responseLatency) +
                    (transfer_offset ? pkt->payloadDelay : 0);

                assert(!tgt_pkt->req->isUncacheable());

                assert(tgt_pkt->req->masterId() < system->maxMasters());
                missLatency[tgt_pkt->cmdToIndex()][tgt_pkt->req->masterId()] +=
                    completion_time - target.recvTime;
            } else if (pkt->cmd == MemCmd::UpgradeFailResp) {
                // failed StoreCond upgrade
                assert(tgt_pkt->cmd == MemCmd::StoreCondReq ||
                       tgt_pkt->cmd == MemCmd::StoreCondFailReq ||
                       tgt_pkt->cmd == MemCmd::SCUpgradeFailReq);
                // responseLatency is the latency of the return path
                // from lower level caches/memory to an upper level cache or
                // the core.
                completion_time += clockEdge(responseLatency) +
                    pkt->payloadDelay;
                tgt_pkt->req->setExtraData(0);
            } else {
                // We are about to send a response to a cache above
                // that asked for an invalidation; we need to
                // invalidate our copy immediately as the most
                // up-to-date copy of the block will now be in the
                // cache above. It will also prevent this cache from
                // responding (if the block was previously dirty) to
                // snoops as they should snoop the caches above where
                // they will get the response from.
                if (is_invalidate && blk && blk->isValid()) {
                    invalidateBlock(blk);
                }
                // not a cache fill, just forwarding response
                // responseLatency is the latency of the return path
                // from lower level cahces/memory to the core.
                completion_time += clockEdge(responseLatency) +
                    pkt->payloadDelay;
                if (pkt->isRead() && !is_error) {
                    // sanity check
                    assert(pkt->getAddr() == tgt_pkt->getAddr());
                    assert(pkt->getSize() >= tgt_pkt->getSize());

                    tgt_pkt->setData(pkt->getConstPtr<uint8_t>());
                }
            }
            tgt_pkt->makeTimingResponse();
            // if this packet is an error copy that to the new packet
            if (is_error)
                tgt_pkt->copyError(pkt);
            if (tgt_pkt->cmd == MemCmd::ReadResp &&
                (is_invalidate || mshr->hasPostInvalidate())) {
                // If intermediate cache got ReadRespWithInvalidate,
                // propagate that.  Response should not have
                // isInvalidate() set otherwise.
                tgt_pkt->cmd = MemCmd::ReadRespWithInvalidate;
                DPRINTF(SPM, "%s: updated cmd to %s\n", __func__,
                        tgt_pkt->print());
            }
            // Reset the bus additional time as it is now accounted for
            tgt_pkt->headerDelay = tgt_pkt->payloadDelay = 0;
            delete tgt_pkt;

            break;

          case MSHR::Target::FromPrefetcher:
            assert(tgt_pkt->cmd == MemCmd::HardPFReq);
            if (blk)
                blk->status |= BlkHWPrefetched;
            delete tgt_pkt;
            break;

          case MSHR::Target::FromSnoop:
            // I don't believe that a snoop can be in an error state
            assert(!is_error);
            // response to snoop request
            DPRINTF(SPM, "processing deferred snoop...\n");
            // If the response is invalidating, a snooping target can
            // be satisfied if it is also invalidating. If the reponse is, not
            // only invalidating, but more specifically an InvalidateResp and
            // the MSHR was created due to an InvalidateReq then a cache above
            // is waiting to satisfy a WriteLineReq. In this case even an
            // non-invalidating snoop is added as a target here since this is
            // the ordering point. When the InvalidateResp reaches this cache,
            // the snooping target will snoop further the cache above with the
            // WriteLineReq.
            assert(!is_invalidate || pkt->cmd == MemCmd::InvalidateResp ||
                   pkt->req->isCacheMaintenance() ||
                   mshr->hasPostInvalidate());
            handleSnoop(tgt_pkt, blk, true, true, mshr->hasPostInvalidate());
            break;

          default:
            panic("Illegal target->source enum %d\n", target.source);
        }
    }

    maintainClusivity(targets.hasFromCache, blk);

    if (blk && blk->isValid()) {
        // an invalidate response stemming from a write line request
        // should not invalidate the block, so check if the
        // invalidation should be discarded
        if (is_invalidate || mshr->hasPostInvalidate()) {
            invalidateBlock(blk);
        } else if (mshr->hasPostDowngrade()) {
            blk->status &= ~BlkWritable;
        }
    }
}

PacketPtr
SPM::evictBlock(CacheBlk *blk)
{
    PacketPtr pkt = (blk->isDirty() || writebackClean) ?
        writebackBlk(blk) : cleanEvictBlk(blk);

    invalidateBlock(blk);

    return pkt;
}

PacketPtr
SPM::cleanEvictBlk(CacheBlk *blk)
{
    assert(!writebackClean);
    assert(blk && blk->isValid() && !blk->isDirty());

    // Creating a zero sized write, a message to the snoop filter
    RequestPtr req = std::make_shared<Request>(
        regenerateBlkAddr(blk), blkSize, 0, Request::wbMasterId);

    if (blk->isSecure())
        req->setFlags(Request::SECURE);

    req->taskId(blk->task_id);

    PacketPtr pkt = new Packet(req, MemCmd::CleanEvict);
    pkt->allocate();
    DPRINTF(SPM, "Create CleanEvict %s\n", pkt->print());

    return pkt;
}

/////////////////////////////////////////////////////
//
// Snoop path: requests coming in from the memory side
//
/////////////////////////////////////////////////////

void
SPM::doTimingSupplyResponse(PacketPtr req_pkt, const uint8_t *blk_data,
                              bool already_copied, bool pending_inval)
{
    // sanity check
    assert(req_pkt->isRequest());
    assert(req_pkt->needsResponse());

    DPRINTF(SPM, "%s: for %s\n", __func__, req_pkt->print());
    // timing-mode snoop responses require a new packet, unless we
    // already made a copy...
    PacketPtr pkt = req_pkt;
    if (!already_copied)
        // do not clear flags, and allocate space for data if the
        // packet needs it (the only packets that carry data are read
        // responses)
        pkt = new Packet(req_pkt, false, req_pkt->isRead());

    assert(req_pkt->req->isUncacheable() || req_pkt->isInvalidate() ||
           pkt->hasSharers());
    pkt->makeTimingResponse();
    if (pkt->isRead()) {
        pkt->setDataFromBlock(blk_data, blkSize);
    }
    if (pkt->cmd == MemCmd::ReadResp && pending_inval) {
        // Assume we defer a response to a read from a far-away cache
        // A, then later defer a ReadExcl from a cache B on the same
        // bus as us. We'll assert cacheResponding in both cases, but
        // in the latter case cacheResponding will keep the
        // invalidation from reaching cache A. This special response
        // tells cache A that it gets the block to satisfy its read,
        // but must immediately invalidate it.
        pkt->cmd = MemCmd::ReadRespWithInvalidate;
    }
    // Here we consider forward_time, paying for just forward latency and
    // also charging the delay provided by the xbar.
    // forward_time is used as send_time in next allocateWriteBuffer().
    Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;
    // Here we reset the timing of the packet.
    pkt->headerDelay = pkt->payloadDelay = 0;
    DPRINTF(SPMVerbose, "%s: created response: %s tick: %lu\n", __func__,
            pkt->print(), forward_time);
    memSidePort.schedTimingSnoopResp(pkt, forward_time, true);
}

uint32_t
SPM::handleSnoop(PacketPtr pkt, CacheBlk *blk, bool is_timing,
                   bool is_deferred, bool pending_inval)
{
    DPRINTF(SPMVerbose, "%s: for %s\n", __func__, pkt->print());
    // deferred snoops can only happen in timing mode
    assert(!(is_deferred && !is_timing));
    // pending_inval only makes sense on deferred snoops
    assert(!(pending_inval && !is_deferred));
    assert(pkt->isRequest());

    // the packet may get modified if we or a forwarded snooper
    // responds in atomic mode, so remember a few things about the
    // original packet up front
    bool invalidate = pkt->isInvalidate();
    bool M5_VAR_USED needs_writable = pkt->needsWritable();

    // at the moment we could get an uncacheable write which does not
    // have the invalidate flag, and we need a suitable way of dealing
    // with this case
    panic_if(invalidate && pkt->req->isUncacheable(),
             "%s got an invalidating uncacheable snoop request %s",
             name(), pkt->print());

    uint32_t snoop_delay = 0;

    if (forwardSnoops) {
        // first propagate snoop upward to see if anyone above us wants to
        // handle it.  save & restore packet src since it will get
        // rewritten to be relative to cpu-side bus (if any)
        bool alreadyResponded = pkt->cacheResponding();
        if (is_timing) {
            // copy the packet so that we can clear any flags before
            // forwarding it upwards, we also allocate data (passing
            // the pointer along in case of static data), in case
            // there is a snoop hit in upper levels
            Packet snoopPkt(pkt, true, true);
            snoopPkt.setExpressSnoop();
            // the snoop packet does not need to wait any additional
            // time
            snoopPkt.headerDelay = snoopPkt.payloadDelay = 0;
            cpuSidePort.sendTimingSnoopReq(&snoopPkt);

            // add the header delay (including crossbar and snoop
            // delays) of the upward snoop to the snoop delay for this
            // cache
            snoop_delay += snoopPkt.headerDelay;

            if (snoopPkt.cacheResponding()) {
                // cache-to-cache response from some upper cache
                assert(!alreadyResponded);
                pkt->setCacheResponding();
            }
            // upstream cache has the block, or has an outstanding
            // MSHR, pass the flag on
            if (snoopPkt.hasSharers()) {
                pkt->setHasSharers();
            }
            // If this request is a prefetch or clean evict and an upper level
            // signals block present, make sure to propagate the block
            // presence to the requester.
            if (snoopPkt.isBlockCached()) {
                pkt->setBlockCached();
            }
            // If the request was satisfied by snooping the cache
            // above, mark the original packet as satisfied too.
            if (snoopPkt.satisfied()) {
                pkt->setSatisfied();
            }
        } else {
            cpuSidePort.sendAtomicSnoop(pkt);
            if (!alreadyResponded && pkt->cacheResponding()) {
                // cache-to-cache response from some upper cache:
                // forward response to original requester
                assert(pkt->isResponse());
            }
        }
    }

    bool respond = false;
    bool blk_valid = blk && blk->isValid();
    if (pkt->isClean()) {
        if (blk_valid && blk->isDirty()) {
            DPRINTF(SPMVerbose, "%s: packet (snoop) %s found block: %s\n",
                    __func__, pkt->print(), blk->print());
            PacketPtr wb_pkt = writecleanBlk(blk, pkt->req->getDest(), pkt->id);
            PacketList writebacks;
            writebacks.push_back(wb_pkt);

            if (is_timing) {
                // anything that is merely forwarded pays for the forward
                // latency and the delay provided by the crossbar
                Tick forward_time = clockEdge(forwardLatency) +
                    pkt->headerDelay;
                doWritebacks(writebacks, forward_time);
            } else {
                doWritebacksAtomic(writebacks);
            }
            pkt->setSatisfied();
        }
    } else if (!blk_valid) {
        DPRINTF(SPMVerbose, "%s: snoop miss for %s\n", __func__,
                pkt->print());
        if (is_deferred) {
            // we no longer have the block, and will not respond, but a
            // packet was allocated in MSHR::handleSnoop and we have
            // to delete it
            assert(pkt->needsResponse());

            // we have passed the block to a cache upstream, that
            // cache should be responding
            assert(pkt->cacheResponding());

            delete pkt;
        }
        return snoop_delay;
    } else {
        DPRINTF(SPM, "%s: snoop hit for %s, old state is %s\n", __func__,
                pkt->print(), blk->print());

        // We may end up modifying both the block state and the packet (if
        // we respond in atomic mode), so just figure out what to do now
        // and then do it later. We respond to all snoops that need
        // responses provided we have the block in dirty state. The
        // invalidation itself is taken care of below. We don't respond to
        // cache maintenance operations as this is done by the destination
        // xbar.
        respond = blk->isDirty() && pkt->needsResponse();

        chatty_assert(!(isReadOnly && blk->isDirty()), "Should never have "
                      "a dirty block in a read-only cache %s\n", name());
    }

    // Invalidate any prefetch's from below that would strip write permissions
    // MemCmd::HardPFReq is only observed by upstream caches.  After missing
    // above and in it's own cache, a new MemCmd::ReadReq is created that
    // downstream caches observe.
    if (pkt->mustCheckAbove()) {
        DPRINTF(SPM, "Found addr %#llx in upper level cache for snoop %s "
                "from lower cache\n", pkt->getAddr(), pkt->print());
        pkt->setBlockCached();
        return snoop_delay;
    }

    if (pkt->isRead() && !invalidate) {
        // reading without requiring the line in a writable state
        assert(!needs_writable);
        pkt->setHasSharers();

        // if the requesting packet is uncacheable, retain the line in
        // the current state, otherwhise unset the writable flag,
        // which means we go from Modified to Owned (and will respond
        // below), remain in Owned (and will respond below), from
        // Exclusive to Shared, or remain in Shared
        if (!pkt->req->isUncacheable())
            blk->status &= ~BlkWritable;
        DPRINTF(SPM, "new state is %s\n", blk->print());
    }

    if (respond) {
        // prevent anyone else from responding, cache as well as
        // memory, and also prevent any memory from even seeing the
        // request
        pkt->setCacheResponding();
        if (!pkt->isClean() && blk->isWritable()) {
            // inform the cache hierarchy that this cache had the line
            // in the Modified state so that we avoid unnecessary
            // invalidations (see Packet::setResponderHadWritable)
            pkt->setResponderHadWritable();

            // in the case of an uncacheable request there is no point
            // in setting the responderHadWritable flag, but since the
            // recipient does not care there is no harm in doing so
        } else {
            // if the packet has needsWritable set we invalidate our
            // copy below and all other copies will be invalidates
            // through express snoops, and if needsWritable is not set
            // we already called setHasSharers above
        }

        // if we are returning a writable and dirty (Modified) line,
        // we should be invalidating the line
        panic_if(!invalidate && !pkt->hasSharers(),
                 "%s is passing a Modified line through %s, "
                 "but keeping the block", name(), pkt->print());

        if (is_timing) {
            doTimingSupplyResponse(pkt, blk->data, is_deferred, pending_inval);
        } else {
            pkt->makeAtomicResponse();
            // packets such as upgrades do not actually have any data
            // payload
            if (pkt->hasData())
                pkt->setDataFromBlock(blk->data, blkSize);
        }
    }

    if (!respond && is_deferred) {
        assert(pkt->needsResponse());
        delete pkt;
    }

    // Do this last in case it deallocates block data or something
    // like that
    if (blk_valid && invalidate) {
        invalidateBlock(blk);
        DPRINTF(SPM, "new state is %s\n", blk->print());
    }

    return snoop_delay;
}


void
SPM::recvTimingSnoopReq(PacketPtr pkt)
{
    DPRINTF(SPMTags, "%s: for %s\n", __func__, pkt->print());
    DPRINTF(SPM, "recvTimingSnoopReq\n");

    return;

    // no need to snoop requests that are not in range
    if (!inRange(pkt->getAddr())) {
        return;
    }

    bool is_secure = pkt->isSecure();
    CacheBlk *blk = tags->findBlock(pkt->getAddr(), is_secure);

    Addr blk_addr = pkt->getBlockAddr(blkSize);
    MSHR *mshr = mshrQueue.findMatch(blk_addr, is_secure);

    // Update the latency cost of the snoop so that the crossbar can
    // account for it. Do not overwrite what other neighbouring caches
    // have already done, rather take the maximum. The update is
    // tentative, for cases where we return before an upward snoop
    // happens below.
    pkt->snoopDelay = std::max<uint32_t>(pkt->snoopDelay,
                                         lookupLatency * clockPeriod());

    // Inform request(Prefetch, CleanEvict or Writeback) from below of
    // MSHR hit, set setBlockCached.
    if (mshr && pkt->mustCheckAbove()) {
        DPRINTF(SPM, "Setting block cached for %s from lower cache on "
                "mshr hit\n", pkt->print());
        pkt->setBlockCached();
        return;
    }

    // Bypass any existing cache maintenance requests if the request
    // has been satisfied already (i.e., the dirty block has been
    // found).
    if (mshr && pkt->req->isCacheMaintenance() && pkt->satisfied()) {
        return;
    }

    // Let the MSHR itself track the snoop and decide whether we want
    // to go ahead and do the regular cache snoop
    if (mshr && mshr->handleSnoop(pkt, order++)) {
        DPRINTF(SPM, "Deferring snoop on in-service MSHR to blk %#llx (%s)."
                "mshrs: %s\n", blk_addr, is_secure ? "s" : "ns",
                mshr->print());

        if (mshr->getNumTargets() > numTarget)
            warn("allocating bonus target for snoop"); //handle later
        return;
    }

    //We also need to check the writeback buffers and handle those
    WriteQueueEntry *wb_entry = writeBuffer.findMatch(blk_addr, is_secure);
    if (wb_entry) {
        DPRINTF(SPM, "Snoop hit in writeback to addr %#llx (%s)\n",
                pkt->getAddr(), is_secure ? "s" : "ns");
        // Expect to see only Writebacks and/or CleanEvicts here, both of
        // which should not be generated for uncacheable data.
        assert(!wb_entry->isUncacheable());
        // There should only be a single request responsible for generating
        // Writebacks/CleanEvicts.
        assert(wb_entry->getNumTargets() == 1);
        PacketPtr wb_pkt = wb_entry->getTarget()->pkt;
        assert(wb_pkt->isEviction() || wb_pkt->cmd == MemCmd::WriteClean);

        if (pkt->isEviction()) {
            // if the block is found in the write queue, set the BLOCK_CACHED
            // flag for Writeback/CleanEvict snoop. On return the snoop will
            // propagate the BLOCK_CACHED flag in Writeback packets and prevent
            // any CleanEvicts from travelling down the memory hierarchy.
            pkt->setBlockCached();
            DPRINTF(SPM, "%s: Squashing %s from lower cache on writequeue "
                    "hit\n", __func__, pkt->print());
            return;
        }

        // conceptually writebacks are no different to other blocks in
        // this cache, so the behaviour is modelled after handleSnoop,
        // the difference being that instead of querying the block
        // state to determine if it is dirty and writable, we use the
        // command and fields of the writeback packet
        bool respond = wb_pkt->cmd == MemCmd::WritebackDirty &&
            pkt->needsResponse();
        bool have_writable = !wb_pkt->hasSharers();
        bool invalidate = pkt->isInvalidate();

        if (!pkt->req->isUncacheable() && pkt->isRead() && !invalidate) {
            assert(!pkt->needsWritable());
            pkt->setHasSharers();
            wb_pkt->setHasSharers();
        }

        if (respond) {
            pkt->setCacheResponding();

            if (have_writable) {
                pkt->setResponderHadWritable();
            }

            doTimingSupplyResponse(pkt, wb_pkt->getConstPtr<uint8_t>(),
                                   false, false);
        }

        if (invalidate && wb_pkt->cmd != MemCmd::WriteClean) {
            // Invalidation trumps our writeback... discard here
            // Note: markInService will remove entry from writeback buffer.
            markInService(wb_entry);
            delete wb_pkt;
        }
    }

    // If this was a shared writeback, there may still be
    // other shared copies above that require invalidation.
    // We could be more selective and return here if the
    // request is non-exclusive or if the writeback is
    // exclusive.
    uint32_t snoop_delay = handleSnoop(pkt, blk, true, false, false);

    // Override what we did when we first saw the snoop, as we now
    // also have the cost of the upwards snoops to account for
    pkt->snoopDelay = std::max<uint32_t>(pkt->snoopDelay, snoop_delay +
                                         lookupLatency * clockPeriod());
}

Tick
SPM::recvAtomicSnoop(PacketPtr pkt)
{
    // no need to snoop requests that are not in range.
    if (!inRange(pkt->getAddr())) {
        return 0;
    }

    CacheBlk *blk = tags->findBlock(pkt->getAddr(), pkt->isSecure());
    uint32_t snoop_delay = handleSnoop(pkt, blk, false, false, false);
    return snoop_delay + lookupLatency * clockPeriod();
}

bool
SPM::isCachedAbove(PacketPtr pkt, bool is_timing)
{
    if (!forwardSnoops)
        return false;
    // Mirroring the flow of HardPFReqs, the cache sends CleanEvict and
    // Writeback snoops into upper level caches to check for copies of the
    // same block. Using the BLOCK_CACHED flag with the Writeback/CleanEvict
    // packet, the cache can inform the crossbar below of presence or absence
    // of the block.
    if (is_timing) {
        Packet snoop_pkt(pkt, true, false);
        snoop_pkt.setExpressSnoop();
        // Assert that packet is either Writeback or CleanEvict and not a
        // prefetch request because prefetch requests need an MSHR and may
        // generate a snoop response.
        assert(pkt->isEviction() || pkt->cmd == MemCmd::WriteClean);
        snoop_pkt.senderState = nullptr;
        cpuSidePort.sendTimingSnoopReq(&snoop_pkt);
        // Writeback/CleanEvict snoops do not generate a snoop response.
        assert(!(snoop_pkt.cacheResponding()));
        return snoop_pkt.isBlockCached();
    } else {
        cpuSidePort.sendAtomicSnoop(pkt);
        return pkt->isBlockCached();
    }
}

bool
SPM::sendMSHRQueuePacket(MSHR* mshr)
{
    assert(mshr);

    // use request from 1st target
    PacketPtr tgt_pkt = mshr->getTarget()->pkt;

    if (tgt_pkt->cmd == MemCmd::HardPFReq && forwardSnoops) {
        DPRINTF(SPM, "%s: MSHR %s\n", __func__, tgt_pkt->print());

        // we should never have hardware prefetches to allocated
        // blocks
        assert(!tags->findBlock(mshr->blkAddr, mshr->isSecure));

        // We need to check the caches above us to verify that
        // they don't have a copy of this block in the dirty state
        // at the moment. Without this check we could get a stale
        // copy from memory that might get used in place of the
        // dirty one.
        Packet snoop_pkt(tgt_pkt, true, false);
        snoop_pkt.setExpressSnoop();
        // We are sending this packet upwards, but if it hits we will
        // get a snoop response that we end up treating just like a
        // normal response, hence it needs the MSHR as its sender
        // state
        snoop_pkt.senderState = mshr;
        cpuSidePort.sendTimingSnoopReq(&snoop_pkt);

        // Check to see if the prefetch was squashed by an upper cache (to
        // prevent us from grabbing the line) or if a Check to see if a
        // writeback arrived between the time the prefetch was placed in
        // the MSHRs and when it was selected to be sent or if the
        // prefetch was squashed by an upper cache.

        // It is important to check cacheResponding before
        // prefetchSquashed. If another cache has committed to
        // responding, it will be sending a dirty response which will
        // arrive at the MSHR allocated for this request. Checking the
        // prefetchSquash first may result in the MSHR being
        // prematurely deallocated.
        if (snoop_pkt.cacheResponding()) {
            auto M5_VAR_USED r = outstandingSnoop.insert(snoop_pkt.req);
            assert(r.second);

            // if we are getting a snoop response with no sharers it
            // will be allocated as Modified
            bool pending_modified_resp = !snoop_pkt.hasSharers();
            markInService(mshr, pending_modified_resp);

            DPRINTF(SPM, "Upward snoop of prefetch for addr"
                    " %#x (%s) hit\n",
                    tgt_pkt->getAddr(), tgt_pkt->isSecure()? "s": "ns");
            return false;
        }

        if (snoop_pkt.isBlockCached()) {
            DPRINTF(SPM, "Block present, prefetch squashed by cache.  "
                    "Deallocating mshr target %#x.\n",
                    mshr->blkAddr);

            // Deallocate the mshr target
            if (mshrQueue.forceDeallocateTarget(mshr)) {
                // Clear block if this deallocation resulted freed an
                // mshr when all had previously been utilized
                clearBlocked(Blocked_NoMSHRs);
            }

            // given that no response is expected, delete Request and Packet
            delete tgt_pkt;

            return false;
        }
    }

    return BaseCache::sendMSHRQueuePacket(mshr);
}

///////////////
//
// MemSidePort
//
///////////////
void
SPM::CacheSidePort::recvTimingSnoopReq(PacketPtr pkt)
{
    spm->handleTimingSnoopReq(pkt);
}

bool
SPM::CacheSidePort::recvTimingResp(PacketPtr pkt)
{
    return spm->handleTimingResp(pkt);
}

Tick
SPM::CacheSidePort::recvAtomicSnoop(PacketPtr pkt)
{
    return spm->handleAtomicSnoop(pkt);
}

void
SPM::CacheSidePort::recvFunctionalSnoop(PacketPtr pkt)
{
    spm->handleFunctionalSnoop(pkt);
}

void
SPM::CacheSidePort::recvReqRetry()
{
    spm->handleReqRetry();
}

void
SPM::CacheSidePort::recvRetrySnoopResp()
{
    spm->handleRetrySnoopResp();
}

void
SPM::CacheSidePort::recvRangeChange()
{
    spm->sendRangeChange();
}

void
SPM::CacheSidePort::recvDmaAction()
{
    spm->handleDmaAction();
}

SPM::CacheSidePort::CacheSidePort(const std::string &_name,
                                  SPM *_spm,
                                  const std::string &_label)
    : MasterPort(_name, _spm), spm(_spm)
{
}

////////////////////////
//
// partition function
//
////////////////////////
CacheBlk*
SPM::allocateBlock(const PacketPtr pkt, PacketList &writebacks)
{
    // Get address
    const Addr addr = pkt->getAddr();

    // Get secure bit
    const bool is_secure = pkt->isSecure();

    // Find replacement victim
    std::vector<CacheBlk*> evict_blks;
    CacheBlk *victim = tags->findVictim(addr, is_secure, evict_blks);

    // It is valid to return nullptr if there is no victim
    if (!victim)
        return nullptr;

    // Print victim block's information
    DPRINTF(SPMRepl, "Replacement victim: %s\n", victim->print());
    if (recvPageNum == pageCacheLineNum - 1) {
        //auto iter = pageState.find((victim->tag) >> pageShift);
        //if (iter != pageState.end()) {
            DPRINTF(SPMRepl, "Done page evict.\n");
            //(*iter).second = false;
            if (victim->isValid()) {
                // pageMissCount.erase((victim->tag) >> pageShift);
                pageState.erase((victim->tag) >> pageShift);
                spmPageTypeTable.erase((victim->tag) >> pageShift);
                spmAccCountTable.erase((victim->tag) >> pageShift);
                // std::cout << ((victim->tag) >> pageShift) << std::endl;
            }
            
        //}
    }

    // Check for transient state allocations. If any of the entries listed
    // for eviction has a transient state, the allocation fails
    for (const auto& blk : evict_blks) {
        if (blk->isValid()) {
            Addr repl_addr = regenerateBlkAddr(blk);
            MSHR *repl_mshr = mshrQueue.findMatch(repl_addr, blk->isSecure());
            if (repl_mshr) {
                // must be an outstanding upgrade or clean request
                // on a block we're about to replace...
                assert((!blk->isWritable() && repl_mshr->needsWritable()) ||
                       repl_mshr->isCleaning());

                // too hard to replace block with transient state
                // allocation failed, block not inserted
                return nullptr;
            }
        }
    }

    // The victim will be replaced by a new entry, so increase the replacement
    // counter if a valid block is being replaced
    if (victim->isValid()) {
        DPRINTF(SPM, "replacement: replacing %#llx (%s) with %#llx "
                "(%s): %s\n", regenerateBlkAddr(victim),
                victim->isSecure() ? "s" : "ns",
                addr, is_secure ? "s" : "ns",
                victim->isDirty() ? "writeback" : "clean");

        replacements++;
    }

    // Evict valid blocks associated to this victim block
    for (const auto& blk : evict_blks) {
        if (blk->isValid()) {
            if (blk->wasPrefetched()) {
                unusedPrefetches++;
            }

            BaseCache::evictBlock(blk, writebacks);
        }
    }

    // Insert new block at victimized entry
    tags->insertBlock(addr, is_secure, pkt->req->masterId(),
                      pkt->req->taskId(), victim);

    return victim;
}


void 
SPM::handleSendPacket(PacketPtr pkt, Addr paddr, int n)
{  
    //RequestPtr req = RequestPtr(new Request(paddr, blkSize, pkt->req->getFlags(), pkt->req->masterId()));
    //PacketPtr my_pkt = new Packet(pkt->req, MemCmd::ReadReq);
    //my_pkt->allocate();
    PacketPtr pf = nullptr;
    RequestPtr req = std::make_shared<Request>(paddr,
                                               blkSize,
                                               pkt->req->getFlags(),
                                               pkt->req->masterId());
    pf = new Packet(req, MemCmd::SoftPFReq);
    pf->allocate();

    incMissCount(pf);
    pf->headerDelay = Cycles(500*500+n);


    DPRINTF(SPM, "%s: offset: %#x, size: %#x, the time:%llu\n", __func__, pf->getOffset(blkSize), pf->getSize(), clockEdge(forwardLatency) + pf->headerDelay);
                
    recvTimingReq(pf);
}

void
SPM::handlePartitionWithFre(PacketPtr pkt)
{    
    Addr page = pkt->getAddr() >> pageShift;

    DPRINTF(SPM, "%s, page: %#x.\n", __func__, page);

    auto iter = pageFreCount.find(page);

    if (iter != pageFreCount.end()) {
        pageFreCount[page]++; 
    } else {
        pageFreCount[page] = 1;
        pageState[page] = false;
    }

    if (ticksToCycles(curTick()) - temp_time >= 100000000) {
        temp_time = ticksToCycles(curTick());
        page = maxCount(pageFreCount);
        //temp_page = page;
        //int i = 0;
        //while (iter != temp.end() && i < 4) {
            if (pageFreCount[page] >= 10 && !pageState[page]) {
                temp_page = page;
                pageState[page] = true;
                setBlocked(Blocked_DMA);
                Addr paddr = page << pageShift;
                handleSendPacket(pkt, paddr);
            }
            //iter++;
            //++i;
        //}
        auto iter = pageFreCount.begin();
        while (iter != pageFreCount.end()) {
            (*iter).second = 0;
            iter++;
        }
    }
}

void
SPM::handlePartitionWithMiss(PacketPtr pkt)
{
    Addr page = pkt->getAddr() >> pageShift;

    if (pkt->cacheMiss) {
        total_miss++;
        if (pageMissCount.find(page) != pageMissCount.end())
            pageMissCount[page]++;
        else
            pageMissCount[page] = 1;
        pageState[page] = false;

        // 新方案添加
        if (my_plan) {
            if (pageLineSize.find(page) == pageLineSize.end()) {
                pageLineSize[page] = 32;
            }

            int set = (pkt->getAddr() & ((1 << pageShift) - 1)) >> 6;
            handlePatternReg(page, set);
        }
        
    } else {
        if (pageHitCount.find(page) != pageHitCount.end())
            pageHitCount[page]++;
        else
            pageHitCount[page] = 1;
    }


    bool repl = false;
    if (my_plan) {
        if (pkt->cacheMiss) {
            if ((pageTypeTable[page].first & 0x3) > 0 ) {
                repl = true;
            }
        }
    }

    
    if (!isBlocked() && pkt->cacheMiss && pageMissCount[page] >= th_next && !pageState[page]) {
        DPRINTF(SPMRepl, "%s, page: %#x, miss num: %d.\n", __func__, page, pageMissCount[page]);
        if (!repl && my_plan) {
            return;
        }

        if (my_plan) {
            int n_hit = 0;
            if (pageHitCount.find(page) != pageHitCount.end())
                n_hit = pageHitCount[page];
            int n_miss = pageMissCount[page];
            int CacheEnergy = e_cache * n_hit + (e_cache + e_dram) * n_miss;
            int SPMEnergy = e_spm * (n_hit + n_miss) + 16 * e_dram;

            std::vector<CacheBlk*> evict_blks;
            CacheBlk *victim = tags->findVictim(page, true, evict_blks);
            int temp = 0;
            if (victim->isValid()) {
                Addr victPage = (victim->tag) >> pageShift;
                temp = ticksToCycles(curTick()) - tags->getPageAccessCount(victPage);
            } else {
                temp = 50000;
            }
            if (temp < 50000) {
                return;
            }

            if (CacheEnergy < SPMEnergy && tags->isFull() && th_next > 4) {
                //std::cout << "energy" << std::endl;
                return;
            }
        }


        // 新方案添加
        if (my_plan) {
            spmPageTypeTable[page] = pageTypeTable[page];
            if (pageHitCount.find(page) != pageHitCount.end())
                spmMissRateTable[page] = ((float)pageMissCount[page] / ((float)pageHitCount[page] + (float)pageMissCount[page]));
            else
                spmMissRateTable[page] = 1.0;
        }


        if (my_plan) {

            for (auto iter = pageMissCount.begin(); iter != pageMissCount.end(); ++iter) {
                if (iter->first != page){
                    if ((pageTypeTable[iter->first].first & 0x3) == 0x1 && (spmPageTypeTable[iter->first].first & 0x3) == 0x1) {
                        pageMissCount[iter->first] = 0;
                        pageLineSize[iter->first] = pageLineSize[iter->first] >> 1;

                        if (pageHitCount.find(iter->first) != pageHitCount.end())
                                pageHitCount[iter->first] = 0;
                    }
                }             
            }
        }

        std::vector<CacheBlk*> evict_blks;
        CacheBlk *victim = tags->findVictim(page, true, evict_blks);
        Addr victPage = (victim->tag) >> pageShift;
        float n_acc = (float)spmAccCountTable[victPage];
        float mu = spmMissRateTable[victPage];
        float before_vic = n_acc * (float)e_spm + 16.0 * e_dram;
        float after_vic = n_acc * (float)e_cache + mu * n_acc * (float)e_dram;
        if (before_vic > after_vic )
            N_repl += 1;
            N_alloc+= 1;

        if ((pageTypeTable[page].first & 0x3) == 0x1)
            pre_repl++;
        


        Addr paddr = page << pageShift;

        missReplacements++;
        replPageCount++;
        movePage.push_back(page);
        temp_page = page;
        pageState[page] = true;
        pageMissCount.erase(page);
        if (pageHitCount.find(page) != pageHitCount.end())
            pageHitCount.erase(page);
        pageLineSize.erase(page);
        if (my_plan) {
            if (pageTypeTable[page].first == 1)
                repl_type = true;
            pageTypeTable.erase(page);
        }
        tags->insert_page = true;
        setBlocked(Blocked_DMA);

        curDMATick = curTick();
        for (int i = 0; i < pageCacheLineNum; ++i) {
            total_miss++;
            paddr = temp_page << pageShift | (i << 6);
            PacketPtr pf = nullptr;
            RequestPtr req = std::make_shared<Request>(paddr,
                                                       blkSize,
                                                       pkt->req->getFlags(),
                                                       pkt->req->masterId());
            pf = new Packet(req, MemCmd::SoftPFReq);
            pf->allocate();
            if (i == 0)
                pf->headerDelay = Cycles(350*500);
            sendPackets.push(pf);
        }

        handleDmaAction();
    }

}

bool
SPM::handlePatternReg(Addr page, int set)
{
    if (pageTypeTable.find(page) != pageTypeTable.end()) {

        // 提取items
        long BB = (pageTypeTable[page].first >> 18 ) & 0xffff;
        long LB = (pageTypeTable[page].first >> 14 ) & 0xf;
        long LD = (pageTypeTable[page].first >> 10 ) & 0xf;
        long DS = (pageTypeTable[page].first >> 4 ) & 0x3f;
        long RC = (pageTypeTable[page].first >> 2 ) & 0x3;
        long PT = pageTypeTable[page].first & 0x3;

        // 判断block是否被访问
        // 1--已被访问则计数器+1
        // 2--未被访问则相应bit置1
        if (((BB >> set) & 0x1) == 1) {
            if (RC != 3) {
                RC++;
            }
        } else {
            BB = BB ^ (1 << set);
        }
        long delta = 0;

        // 计算当前offset和上一次offset的差值
        if ((set - LB) < 0 )
            delta = LB - set;
        else
            delta = set - LB;
        long del = 0;

        // 若delta=0或stride为1,则del为1表示连续访问
        if (delta == 0 || delta == LD)
            del = 1;

        // 更新相应的delta sequence/last block offset和last delta
        DS = ((DS & 0x1f) << 1) ^ del;
        LB = set;
        if (delta != 0)
            LD = delta;

        if (PT == 0) {
            if ((RC > 2) && (DS == 0x3f)) {
                PT = 1;
                RC = 1;
            }
            else if (RC > 1) {
                PT = 2;
                RC = 1;
            }
            else if (DS == 0x3f) {
                PT = 3;
                DS = DS >> 1; 
            }
        } else if (PT == 1){
            if (DS != 0x3f || RC < 2) {
                if (DS == 0x3f) {
                    PT = 3;
                    DS = DS >> 1; 
                }
                else if (RC >1) {
                    PT = 2;
                    RC = 1;
                }
            }
        } else if (PT == 2) {
            if ((RC > 2) && (DS == 0x3f)){ //&& (BB == 0x1 || BB == 0x3 || BB == 0x7)) {
                PT = 1;
                RC = 1;
            }
            else if (DS == 0x3f) {
                PT = 3;
                DS = DS >> 1;
            }
        } else if (PT == 3) {
            if ((RC > 2) && (DS == 0x3f)){ //&& (BB == 0x1 || BB == 0x3 || BB == 0x7)) {
                PT = 1;
                RC = 1;
            }
            else if (RC > 1) {
                PT = 2;
                RC = 1;
            }
        }
        // if (delta != 0)
        //     RC = 0;
        //std::cout << BB << " " << LB << " " << LD << " " << DS << " " << RC << " " << PT << std::endl;
        long temp = (BB << 18) + (LB << 14) + (LD << 10) + (DS << 4) + (RC << 2) + PT;
        //std::cout << temp << std:: endl;
        pageTypeTable[page] = std::make_pair(temp, false);
    } else {
         long BB = (1 << set);
         long LB = set;
         long temp = (BB << 18) + (LB << 14) + (0 << 10) + (0 << 4) + (0 << 2) + 0;
        pageTypeTable[page] = std::make_pair(temp, false);
    }
    return true;
}

void
SPM::handleDmaAction()
{
    if (!sendPackets.empty()) {
        PacketPtr temp = sendPackets.front();
        sendPackets.pop();
        // std::cout << ticksToCycles(curTick()) << ":" << " sendPackets" << std::endl;
        // printf("sendPacket\n");
        recvTimingReq(temp);
        dmaDelay = curTick() - curDMATick;
    }
}


void
SPM::handleThresholdAdjust()
{
    if (ticksToCycles(curTick()) - reset_time >= 1000000) {
        reset_time = ticksToCycles(curTick());
        miss_num = 0;
        if (my_plan) {
            // std::cout << 1000000 << std::endl;
            for (auto iter = pageMissCount.begin(); iter != pageMissCount.end(); ++iter) {
                pageTypeTable.erase(iter->first);
                // pageMissCount.erase(iter->first);
                pageMissCount[iter->first] = 0;
                pageLineSize.erase(iter->first);
                if (pageHitCount.find(iter->first) != pageHitCount.end())
                    pageHitCount.erase(iter->first);             
            }

            ma_pre = ma_cur;
            //signed th_pre = th_cur;
            th_cur = th_next;
            ma_cur = total_miss;
            total_miss = 0;
            if (movePage.size() <= 0 && !tags->isFull()) {
                th_next = th_cur >> 1;
                if (th_next < 4)
                    th_next = 4;
            } else if (N_alloc == 0) {
                if (ma_cur < ma_pre) {
                    th_next = th_cur;
                } else {
                    th_next = th_cur >> 1;
                    if (th_next < 4)
                         th_next = 4;
                }
            } else {
                if (N_repl == 0) {
                    if (pre_repl == N_alloc) {
                        th_next = th_next >> 1;
                                    if (th_next < 4)
                    th_next = 4;
                    }
                    else
                        th_next = th_cur;
                } else {
                    th_next = th_cur * 2;
                }
            }

            N_alloc = 0;
            N_repl = 0;
            pre_repl = 0;
            movePage.clear();

        }
        else {
            ma_pre = ma_cur;
            unsigned th_pre = th_cur;
            th_cur = th_next;
            ma_cur = total_miss;
            total_miss = 0;
            if (movePage.size() <= 0 && !tags->isFull()) {
                th_next = th_cur >> 1;
                // if (th_next < 1)
                //     th_next = 2;
            } else if (ma_cur < ma_pre) {
                if (th_cur > th_pre) {
                    th_next = th_cur << 1;
                    // if (th_next > 2048)
                    //     th_next = 2048;
                } else {
                    th_next = th_cur >> 1;
                    if (th_next < 1)
                        th_next = 2;
                }
            } else {
                th_next = th_pre;
            } 
      
            movePage.clear();
            if (my_plan == 0) {
                auto iter = pageMissCount.begin();
                while (iter != pageMissCount.end()) {
                    (*iter).second = 0;
                    iter++;
                }

                auto iter1 = pageHitCount.begin();
                while (iter1 != pageHitCount.end()) {
                    (*iter1).second = 0;
                    iter1++;
                }
            }
            

            auto iter_access = pageAccess.begin();
            while (iter_access != pageAccess.end()) {
                (*iter_access).second = 0;
                iter_access++;
            }
        }
    }
}



bool
SPM::handleAllocSpace(PacketPtr pkt)
{
    DPRINTF(SPM, "%s: alloc space, incSpace: %d.\n", __func__, pkt->incSpace);

    Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;

    //Addr addr = (rand() % (p->size / (p->system->cacheLineSize * p->assoc))) << floorLog2(p->system->cacheLineSize);

    if (pkt->isAllocSpace) {
        if (!pkt->incSpace) {
            // Find replacement victim
            std::vector<CacheBlk*> evict_blks;
            CacheBlk *victim = nullptr;

            Addr addr = pkt->getAddr();
            victim = tags->findUsed(addr, evict_blks);

            if (!victim) {
                return false;
            }

            //CacheBlk *victim = tags->findUsed(addr, evict_blks);

            // Print victim block's information
            DPRINTF(SPMRepl, "Unused victim: %s\n", victim->print());

            PacketList writebacks;

            // Evict valid blocks associated to this victim block
            for (const auto& blk : evict_blks) {
                if (blk->isValid()) {

                    BaseCache::evictBlock(blk, writebacks);
                }
                tags->setUnused(blk);
            }

            doWritebacks(writebacks, forward_time);
        } else {
            // Find replacement victim
            std::vector<CacheBlk*> evict_blks;
            CacheBlk *victim = nullptr;

            Addr addr = pkt->getAddr();
            victim = tags->findUnused(addr, evict_blks);

            if (!victim) {
                return false;
            }

            //CacheBlk *victim = tags->findUnused(addr, evict_blks);

            PacketList writebacks;

            // Print victim block's information
            DPRINTF(SPMRepl, "Used victim: %s\n", victim->print());

            // Evict valid blocks associated to this victim block
            for (const auto& blk : evict_blks) {
                if (blk->isValid()) {

                    BaseCache::evictBlock(blk, writebacks);
                }
                tags->setUsed(blk);
            }

            doWritebacks(writebacks, forward_time);
        }

    } else {
        // Get address
        const Addr addr = pkt->getAddr();

        // Get secure bit
        const bool is_secure = pkt->isSecure();

        // Find replacement victim
        std::vector<CacheBlk*> evict_blks;
        CacheBlk *victim = tags->findVictim(addr, is_secure, evict_blks);
        // It is valid to return nullptr if there is no victim
        if (!victim) {
            return false;
        } else if (victim->isValid() && pageMissCount[((victim->tag) >> pageShift)] >= accessFre) {
            std::vector<CacheBlk*> evict_blks;
            victim = nullptr;

            Addr addr = pkt->getAddr();
            victim = tags->findUnused(addr, evict_blks);

            if (victim != nullptr) {
                return false;
            } else {
                return true;
            }
        }
    }

    return true;
}

SPM*
SPMParams::create()
{
    assert(tags);
    assert(replacement_policy);

    return new SPM(this);
}
