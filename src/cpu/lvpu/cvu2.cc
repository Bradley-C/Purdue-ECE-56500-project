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

#include "cpu/lvpu/cvu2.hh"

#include <algorithm>

#include "arch/generic/pcstate.hh"
#include "base/compiler.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"

//#include "debug/Execute.hh"

//#include "debug/Load.hh"

namespace gem5
{

namespace load_value_prediction
{

ConstantVUnit::ConstantVUnit(const Params &params)
    : SimObject(params),
      numThreads(params.numThreads),
      predHist(numThreads),
      cvu(params.CVUEntries, std::vector<CVUEntry>(params.numAddrHold)),
      stats(this),
      instShiftAmt(params.instShiftAmt),
      numEntries(params.CVUEntries),
      numAddrperEntry(params.numAddrHold)
{
    if (!isPowerOf2(numEntries)) {
        fatal("Invalid CVU Entry size.\n");
    }
    if (!isPowerOf2(numAddrperEntry)) {
        fatal("Invalid number of CVU sets. Check numAddrHold.\n");
    }

    for (unsigned i = 0; i < numEntries; i++) {
        for (unsigned j = 0; j < numAddrperEntry; j++) {
            cvu[i][j].valid = false;
        }
    }
  //  DPRINTF(Execute, "CVU size: %i\n", numEntries);
   // DPRINTF(Execute, "CVU addrs size: %i\n", numAddrperEntry);
   // DPRINTF(Execute, "instruction shift amount: %i\n", instShiftAmt);
}

ConstantVUnit::CVUUnitStats::CVUUnitStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(cvuLoadLookups, statistics::units::Count::get(),
               "Number of CVU Load Lookups"),
      ADD_STAT(cvuStoreLookups, statistics::units::Count::get(),
               "Number of CVU Store Lookups"),
      ADD_STAT(loadMatched, statistics::units::Count::get(),
               "Number of loads that matched"),
      ADD_STAT(loadMatchedIncorrect, statistics::units::Count::get(),
               "Number of loads that did not matched")
{}

probing::PMUUPtr
ConstantVUnit::pmuProbePoint(const char *name)
{
    probing::PMUUPtr ptr;
    ptr.reset(new probing::PMU(getProbeManager(), name));

    return ptr;
}

void
ConstantVUnit::regProbePoints()
{
    ppLoads = pmuProbePoint("Loads");
    ppMisses = pmuProbePoint("Misses");
}

void
ConstantVUnit::drainSanityCheck() const
{
    // We shouldn't have any outstanding requests when we resume from
    // a drained system.
    for ([[maybe_unused]] const auto& ph : predHist)
        assert(ph.empty());
}

inline
unsigned
ConstantVUnit::getIndexCVU(Addr instPC, ThreadID tid)
{
    // Need to shift PC over by the word offset.
    return ((instPC >> instShiftAmt) & (numEntries-1));
}

bool
ConstantVUnit::checkValid(Addr instPC, ThreadID tid)
{
    unsigned cvu_idx = getIndexCVU(instPC, tid);

    assert(cvu_idx < numEntries);
    for (unsigned j = 0; j < numAddrperEntry; j++) {
        if (!cvu[cvu_idx][j].valid ||
        (!cvu[cvu_idx][j].tid == tid)) {
            return false;
        }
    }
    return true;
}

void
ConstantVUnit::updateEntry(PCStateBase &pc,
uint64_t new_value, ThreadID tid)
{
    unsigned cvu_idx = getIndexCVU( pc.instAddr(), tid);

    assert(cvu_idx < numEntries);

    for (unsigned j = 0; j < numAddrperEntry; j++) {
        cvu[cvu_idx][j].tid = tid;
        cvu[cvu_idx][j].valid = true;
        cvu[cvu_idx][j].value = new_value;
    }

   // PredictorHistory cvu_record(seqNum, pc.instAddr(), cvu, tid,
  //  inst, new_value);
   //predHist[tid].push_front(cvu_record);
}

ConstantVUnit::CVUReturn
ConstantVUnit::storeClear(PCStateBase &inst_pc,
 uint64_t store_addr, uint64_t new_value, ThreadID tid){
    CVUReturn return_data;
    unsigned cvu_idx = getIndexCVU(inst_pc.instAddr(), tid);
    assert(cvu_idx < numEntries);
    ++stats.cvuStoreLookups;

    return_data = (CVUReturn){.pc=inst_pc.instAddr(),
    .addr=cvu[cvu_idx][0].value,
    .clear=true, .update=true};
    for (unsigned i = 0; i < numEntries; i++) {
        for (unsigned j = 0; j < numAddrperEntry; j++) {
            //std::cout << "cvu entry: " << i
           // << " cvu value: " << cvu[i][j].value
           // << " cvu valid: " << cvu[i][j].valid
           // << std::endl;
            if (cvu[i][j].value == store_addr && cvu[i][j].valid){
                // Figure out how to pass value being stored
                //to here as
                // well
                return_data = (CVUReturn)
                {.pc=inst_pc.instAddr(), .value=new_value,
                .clear=true,  .update=true};
            }

        }
    }

    return return_data;
}

ConstantVUnit::CVUReturn
ConstantVUnit::addrMatch(PCStateBase& inst_pc, uint64_t data_addr,
ThreadID tid) {
    unsigned cvu_idx = getIndexCVU(inst_pc.instAddr(), tid);

    assert(cvu_idx < numEntries);
    ++stats.cvuLoadLookups;

    CVUReturn return_data;
    return_data = (CVUReturn){.pc=inst_pc.instAddr(), .addr=data_addr,
    .clear=true, .update=true};
    for (unsigned j = 0; j < numAddrperEntry; j++) {
        // std::cout << "CVU Mem Addr: " << cvu[cvu_idx][j].value
        // << " CVU valid bit: " << cvu[cvu_idx][j].valid
        // << " Load Mem Addr: " << data_addr
        // << std::endl;
        if (cvu[cvu_idx][j].value == data_addr && cvu[cvu_idx][j].valid){
            // Figure out how to pass value being stored to here as
            // well
            return_data = (CVUReturn){.pc=inst_pc.instAddr(), .addr=data_addr,
            .clear=false, .update=true};
            ++stats.loadMatched;
        }
        else{
            if (j==numAddrperEntry-1){
            ++stats.loadMatchedIncorrect;}
        }
    }

    return return_data;
 }

void
ConstantVUnit::update(const InstSeqNum &done_sn, uint64_t new_value,
 ThreadID tid)
{

    int i =0;/*
    while (!predHist[tid].empty() && predHist[tid].back().seqNum <= done_sn) {

        updateEntry(PredHist[tid].back().pc, new_value ,tid);
        CVU.update(PredHist[tid].back().pc, new_value, tid);
        predHist[tid].pop_back();
    }*/
}

void
ConstantVUnit::squash(const InstSeqNum &squashed_sn, ThreadID tid)
{

    int i =0;/*
    History &pred_hist = predHist[tid];

  while (!pred_hist.empty() && pred_hist.front().seqNum > squashed_sn) {
        pred_hist.pop_front();
    }*/
}


void
ConstantVUnit::dump()
{
    int i = 0;
    /*
    for (const auto& ph : predHist) {
        if (!ph.empty()) {
            auto pred_hist_it = ph.begin();

            cprintf("predHist[%i].size(): %i\n", i++, ph.size());

            while (pred_hist_it != ph.end()) {
                /*cprintf("sn:%llu], PC:%#x, tid:%i, predTaken:%i, "
                        "predHist:%#x\n",
                        pred_hist_it->loadSeqNum, pred_hist_it->pc,
                        pred_hist_it->tid, pred_hist_it->predTaken,
                        pred_hist_it->bpHistory);
                pred_hist_it++;
            }

            cprintf("\n");
        }
    }*/
}



} // namespace load_value_prediction
} // namespace gem5
