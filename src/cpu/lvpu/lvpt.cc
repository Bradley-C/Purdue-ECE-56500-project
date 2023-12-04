/*
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

#include "cpu/lvpu/lvpt.hh"

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"

namespace gem5
{

namespace load_value_prediction
{

LoadValuePredictionTable::LoadValuePredictionTable(unsigned _numEntries,
                       unsigned _instShiftAmt,
                       unsigned _num_threads)
    : numEntries(_numEntries),
      instShiftAmt(_instShiftAmt),
      log2NumThreads(floorLog2(_num_threads))
{
    DPRINTF(Fetch, "LVPT: Creating LVPT object.\n");

    if (!isPowerOf2(numEntries)) {
        fatal("LVPT entries is not a power of 2!");
    }

    lvpt.resize(numEntries);

    for (unsigned i = 0; i < numEntries; ++i) {
        lvpt[i].valid = false;
    }

    idxMask = numEntries - 1;
}

void
LoadValuePredictionTable::reset()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        lvpt[i].valid = false;
    }
}

inline
unsigned
LoadValuePredictionTable::getIndex(Addr instPC, ThreadID tid)
{
    // Need to shift PC over by the word offset.
    return ((instPC >> instShiftAmt)
            ^ (tid << (instShiftAmt - log2NumThreads)))
            & idxMask;
}

bool
LoadValuePredictionTable::valid(Addr instPC, ThreadID tid)
{
    unsigned lvpt_idx = getIndex(instPC, tid);

    assert(lvpt_idx < numEntries);

    if (lvpt[lvpt_idx].valid
        && lvpt[lvpt_idx].tid == tid) {
        return true;
    } else {
        return false;
    }
}

// @todo Create some sort of return struct that has both whether or not the
// value is valid, and also the value.  For now will just use
// value = 0 to represent invalid entry.
const uint8_t* *
LoadValuePredictionTable::lookup(Addr inst_pc, ThreadID tid)
{
    unsigned lvpt_idx = getIndex(inst_pc, tid);

    assert(lvpt_idx < numEntries);

    if (lvpt[lvpt_idx].valid
        && lvpt[lvpt_idx].tid == tid) {
        return &lvpt[lvpt_idx].value.get();
    } else {
        return nullptr;
    }
}

void
LoadValuePredictionTable::update(Addr inst_pc, const uint8_t *value,
                                 ThreadID tid)
{
    unsigned lvpt_idx = getIndex(inst_pc, tid);

    assert(lvpt_idx < numEntries);

    lvpt[lvpt_idx].tid = tid;
    lvpt[lvpt_idx].valid = true;
    set(lvpt[lvpt_idx].value, *value);
}

} // namespace branch_prediction
} // namespace gem5
