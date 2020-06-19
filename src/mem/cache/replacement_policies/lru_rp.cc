/**
 * Copyright (c) 2018 Inria
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
 * Authors: Daniel Carvalho
 */

#include "mem/cache/replacement_policies/lru_rp.hh"

#include <cassert>
#include <memory>

#include "params/LRURP.hh"

LRURP::LRURP(const Params *p)
    : BaseReplacementPolicy(p)
{
}

void
LRURP::setUnused(const std::shared_ptr<ReplacementData>& replacement_data)
{
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->isUsed = false;
    invalidate(replacement_data);
}

void
LRURP::setUsed(const std::shared_ptr<ReplacementData>& replacement_data)
{
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->isUsed = true;
    invalidate(replacement_data);
}

void
LRURP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
const
{
    // Reset last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = Tick(0);
}

void
LRURP::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Update last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = curTick();
}

void
LRURP::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = curTick();
}

ReplaceableEntry*
LRURP::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    //ReplaceableEntry* victim = candidates[0];
    ReplaceableEntry* victim = nullptr;
    for (const auto& candidate : candidates) {
        if (std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->isUsed) {
            victim = candidate;
            break;
        }
    }
    
    for (const auto& candidate : candidates) {
        // Update victim entry if necessary
        if ((std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->lastTouchTick <
                std::static_pointer_cast<LRUReplData>(
                    victim->replacementData)->lastTouchTick) && 
            std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->isUsed == true)  {
            victim = candidate;
        }
    }

    return victim;
}

ReplaceableEntry*
LRURP::getUsed(const ReplacementCandidates& candidates)
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    //ReplaceableEntry* victim = candidates[0];
    ReplaceableEntry* victim = nullptr;

    int usedNum = 0;
    for (const auto& candidate : candidates) {
        if (std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->isUsed) {
            usedNum++;
        }
    }

    if (usedNum <= 1)
        return victim;

    for (const auto& candidate : candidates) {
        if (std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->isUsed) {
            victim = candidate;
            break;
        }
    }
    
    for (const auto& candidate : candidates) {
        // Update victim entry if necessary
        if ((std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->lastTouchTick <
                std::static_pointer_cast<LRUReplData>(
                    victim->replacementData)->lastTouchTick) && 
            std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->isUsed == true)  {
            victim = candidate;
        }
    }

    return victim;
}

ReplaceableEntry*
LRURP::getUnused(const ReplacementCandidates& candidates)
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    //ReplaceableEntry* victim = candidates[0];
    ReplaceableEntry* victim = nullptr;
    for (const auto& candidate : candidates) {
        if (!std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->isUsed) {
            victim = candidate;
            break;
        }
    }

    return victim;
}

std::shared_ptr<ReplacementData>
LRURP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new LRUReplData());
}

LRURP*
LRURPParams::create()
{
    return new LRURP(this);
}
