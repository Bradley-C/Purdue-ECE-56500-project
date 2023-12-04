/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010 The University of Edinburgh
 * Copyright (c) 2012 Mark D. Hill and David A. Wood
 * All rights reserved
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 */

#include "cpu/lvpu/lvpu.hh"

#include <algorithm>

#include "arch/generic/pcstate.hh"
#include "base/compiler.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/Load.hh"

namespace gem5
{

namespace load_value_prediction
{

LVPredUnit::LVPredUnit(const Params &params)
    : SimObject(params),
      numThreads(params.numThreads),
      loadPredHist(numThreads),
      loadValuePredTable(params.numEntries,
                        params.instShiftAmt,
                        params.numThreads),
      stats(this),
      instShiftAmt(params.instShiftAmt)
{}

LVPredUnit::LVPredUnitStats::LVPredUnitStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(LVPTLookups, statistics::units::Count::get(),
               "Number of LVPT lookups"),
      ADD_STAT(predictedIncorrect, statistics::units::Count::get(),
               "Number of incorrect predictions"),

{}

probing::PMUUPtr
LVPredUnit::pmuProbePoint(const char *name)
{
    probing::PMUUPtr ptr;
    ptr.reset(new probing::PMU(getProbeManager(), name));

    return ptr;
}

void
LVPredUnit::regProbePoints()
{
    ppLoads = pmuProbePoint("Loads");
    ppMisses = pmuProbePoint("Misses");
}

void
LVPredUnit::drainSanityCheck() const
{
    // We shouldn't have any outstanding requests when we resume from
    // a drained system.
    for ([[maybe_unused]] const auto& ph : loadPredHist)
        assert(ph.empty());
}

void
LVPredUnit::update(const InstSeqNum &done_sn, ThreadID tid)
{
    DPRINTF(Load, "[tid:%i] Committing loads until "
            "sn:%llu]\n", tid, done_sn);

    while (!loadPredHist[tid].empty() &&
           loadPredHist[tid].back().loadSeqNum <= done_sn) {
        // Update the load value predictor with the correct results.
        lctUpdate(tid, loadPredHist[tid].back().pc,
                    loadPredHist[tid].back().correct,
                    false,
                    loadPredHist[tid].back().inst,
                    loadPredHist[tid].back().data);

        loadPredHist[tid].pop_back();
    }
}

void
LVPredUnit::squash(const InstSeqNum &squashed_sn, ThreadID tid)
{
    History &load_pred_hist = loadPredHist[tid];

    while (!load_pred_hist.empty() &&
           load_pred_hist.front().loadSeqNum > squashed_sn) {

        DPRINTF(Load, "[tid:%i] [squash sn:%llu] "
                "Removing history for [sn:%llu] "
                "PC %#x\n", tid, squashed_sn,
                load_pred_hist.front().loadSeqNum,
                load_pred_hist.front().pc);

        load_pred_hist.pop_front();

        DPRINTF(Load, "[tid:%i] [squash sn:%llu] loadPredHist.size(): %i\n",
                tid, squashed_sn, loadPredHist[tid].size());
    }
}

void
LVPredUnit::squash(const InstSeqNum &squashed_sn,
                  const uint64_t &corr_data, ThreadID tid)
{
    // Now that we know that a load was mispredicted, we need to undo
    // all the loads that have been seen up until this load and
    // fix up everything.
    // NOTE: This should be call conceivably in only 1 scenario:
    //  After a load is executed, it updates its status in the ROB in the
    //  commit stage then checks the ROB update and sends a signal to
    //  the fetch stage to squash the history after the mispredict.

    History &load_pred_hist = loadPredHist[tid];

    ++stats.predictedIncorrect;
    ppMisses->notify(1);

    DPRINTF(Load, "[tid:%i] Squashing from sequence number %i, "
            "setting data to %s\n", tid, squashed_sn, corr_data);

    // Squash All Branches AFTER this mispredicted branch
    squash(squashed_sn, tid);

    // If there's a squash due to a syscall, there may not be an entry
    // corresponding to the squash.  In that case, don't bother trying to
    // fix up the entry.
    if (!load_pred_hist.empty()) {

        auto hist_it = load_pred_hist.begin();
        //HistoryIt hist_it = find(load_pred_hist.begin(),
        //                        load_pred_hist.end(),
        //                        squashed_sn);

        //assert(hist_it != load_pred_hist.end());
        if (load_pred_hist.front().loadSeqNum != squashed_sn) {
            DPRINTF(Load, "Front sn %i != Squash sn %i\n",
                    load_pred_hist.front().loadSeqNum, squashed_sn);

            assert(load_pred_hist.front().loadSeqNum == squashed_sn);
        }

        // There are separate functions for in-order and out-of-order
        // branch prediction, but not for update. Therefore, this
        // call should take into account that the mispredicted branch may
        // be on the wrong path (i.e., OoO execution), and that the counter
        // table(s) should not be updated. Thus, this call should restore the
        // state of the underlying predictor, for instance the local/global
        // histories. The counter tables will be updated when the branch
        // actually commits.

        // Remember the correct direction for the update at commit.
        load_pred_hist.front().predTaken = actually_taken;
        load_pred_hist.front().target = corr_target.instAddr();

        lctUpdate(tid, (*hist_it).pc, correct, true,
               load_pred_hist.front().inst,
               corr_data);

        if (!correct) {
            DPRINTF(Load,"[tid:%i] [squash sn:%llu] "
                    "LVPT Update called for [sn:%llu] "
                    "PC %#x\n", tid, squashed_sn,
                    hist_it->loadSeqNum, hist_it->pc);

            LVPT.update(hist_it->pc, corr_data, tid);
        } else {
           // prediction was incorrect
        }
    } else {
        DPRINTF(Load, "[tid:%i] [sn:%llu] load_pred_hist empty, can't "
                "update\n", tid, squashed_sn);
    }
}

void
LVPredUnit::lctUpdate(ThreadID tid, Addr load_addr, bool correct,
                bool squashed, const StaticInstPtr &inst, uint8_t *corrData)
{
    unsigned local_predictor_idx;

    // No state to restore, and we do not update on the wrong
    // path.
    if (squashed) {
        return;
    }

    // Update the local predictor.
    local_predictor_idx = getLCTIndex(load_addr);

    DPRINTF(Fetch, "Looking up index %#x\n", local_predictor_idx);

    if (correct) {
        DPRINTF(Fetch, "LCT entry incremented.\n");
        loadClassTable[local_predictor_idx]++;
    } else {
        DPRINTF(Fetch, "LCT entry decremented.\n");
        loadClassTable[local_predictor_idx]--;
    }
}

inline
LVPredUnit::LoadClass
LVPredUnit::getLoadClass(uint8_t &count)
{
    switch(count) {
      case 0:
        return LoadClass::UnpredictableStrong;
      case 1:
        return LoadClass::UnpredictableWeak;
      case 2:
        return LoadClass::Predictable;
      default:
        return LoadClass::Constant;
    }
}

inline
unsigned
LVPredUnit::getLCTIndex(Addr &load_addr)
{
    return (load_addr >> instShiftAmt) & indexMask;
}

void
LVPredUnit::dump()
{
    int i = 0;
    for (const auto& ph : loadPredHist) {
        if (!ph.empty()) {
            auto pred_hist_it = ph.begin();

            cprintf("loadPredHist[%i].size(): %i\n", i++, ph.size());

            while (pred_hist_it != ph.end()) {
                cprintf("sn:%llu], PC:%#x, tid:%i, predTaken:%i, "
                        "bpHistory:%#x\n",
                        pred_hist_it->loadSeqNum, pred_hist_it->pc,
                        pred_hist_it->tid, pred_hist_it->predTaken,
                        pred_hist_it->bpHistory);
                pred_hist_it++;
            }

            cprintf("\n");
        }
    }
}

} // namespace branch_prediction
} // namespace gem5
