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
      LVPT(params.numEntries,
                        params.instShiftAmt,
                        params.numThreads),
      stats(this),
      instShiftAmt(params.instShiftAmt),
      lctSize(params.lctSize),
      lctBits(params.lctBits),
      lctSets(lctSize / lctBits),
      loadClassTable(lctSets, SatCounter8(lctBits)),
      lctIndexMask(lctSets - 1)
{
    if (!isPowerOf2(lctSize)) {
        fatal("Invalid LCT size.\n");
    }
    if (!isPowerOf2(lctSets)) {
        fatal("Invalid number of LCT sets. Check lctBits.\n");
    }

    DPRINTF(Fetch, "index mask: %#x\n", lctIndexMask);
    DPRINTF(Fetch, "LCT size: %i\n", lctSize);
    DPRINTF(Fetch, "LCT counter bits: %i\n", lctBits);
    DPRINTF(Fetch, "instruction shift amount: %i\n", instShiftAmt);
}

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

bool
LVPredUnit::getPrediction(const StaticInstPtr &inst,
                          const InstSeqNum &loadSeqNum,
                          PCStateBase &pc, LVPResult &result, ThreadID tid)
{
    ++stats.lvpuLookups;
    ppLoads->notify(1);

    // Get the load classification and the current value in the LVPT.
    LoadClass loadClass = getLoadClass(tid, pc.instAddr(), load_history);
    result.loadClass = loadClass;
    result.data = lvptLookup(pc.instAddr());
    bool predictable = (loadClass == Predictable) || (loadClass == Constant);
    if (predictable) ++stats.predicted;

    DPRINTF(Load, "[tid:%i] [sn:%llu] "
            "Load predictor predicted %i for PC %s\n",
            tid, loadSeqNum, predictable, pc);

    // append a record to the load prediction history table
    DPRINTF(Load,
            "[tid:%i] [sn:%llu] Creating prediction history for PC %s\n",
            tid, loadSeqNum, pc);
    PredictorHistory load_predict_record(loadSeqNum, pc.instAddr(),
                                         predictable, loadClass,
                                         tid, inst, result.data);
    loadPredHist[tid].push_front(load_predict_record);

    DPRINTF(Load,
            "[tid:%i] [sn:%llu] History entry added. "
            "loadPredHist.size(): %i\n",
            tid, loadSeqNum, loadPredHist[tid].size());

    return predictable;
}

void
LVPredUnit::update(const InstSeqNum &done_sn, ThreadID tid)
{
    DPRINTF(Load, "[tid:%i] Committing loads until "
            "sn:%llu]\n", tid, done_sn);

    while (!loadPredHist[tid].empty() &&
           loadPredHist[tid].back().loadSeqNum <= done_sn) {
        // Update the load value predictor with the correct results.
        lctUpdate(loadPredHist[tid].back().pc,
                    loadPredHist[tid].back().correct,
                    loadPredHist[tid].back().data);
        LVPT.update(loadPredHist[tid].back().pc,
                    loadPredHist[tid].back().data,
                    tid)
        loadPredHist[tid].pop_back();
    }
}

void
LVPredUnit::lctUpdate(const Addr load_addr, const bool correct,
                      const uint64_t &corrData)
{
    unsigned lctIndex;

    // Update the local predictor.
    lctIndex = getLCTIndex(load_addr);

    DPRINTF(Fetch, "Looking up index %#x\n", lctIndex);

    if (correct) {
        DPRINTF(Fetch, "LCT entry incremented.\n");
        loadClassTable[lctIndex]++;
    } else {
        DPRINTF(Fetch, "LCT entry decremented.\n");
        loadClassTable[lctIndex]--;
    }
}


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

std::string
getLoadClassString(LoadClass loadClass)
{
    std::string loadClassString;
    switch(loadClass) {
        case UnpredictableStrong:
            loadClassString = "UnpredictableStrong";
            return loadClassString;
        case UnpredictableWeak:
            loadClassString = "UnpredictableWeak";
            return loadClassString;
        case Predictable:
            loadClassString = "Predictable";
            return loadClassString;
        case Constant:
            loadClassString = "Constant";
            return loadClassString;
        default:
            DPRINTF(LVPU, "Invalid load class: %d. Values must use LoadClass
                    enumeration. Returning null terminator.", loadClass);
            return "\0";
    }
}

inline
unsigned
LVPredUnit::getLCTIndex(Addr &load_addr)
{
    return (load_addr >> instShiftAmt) & lctIndexMask;
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

} // namespace load_value_prediction
} // namespace gem5
