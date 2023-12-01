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

#include "cpu/lvpu/cvu.hh"

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/Execute.hh"

namespace gem5
{

namespace load_value_prediction
{

ConstantVerificationUnit::ConstantVerificationUnit(unsigned _numEntries,
                       unsigned _tagBits,
                       unsigned _instShiftAmt,
                       unsigned _num_threads)
    : numEntries(_numEntries),
      tagBits(_tagBits),
      instShiftAmt(_instShiftAmt),
      log2NumThreads(floorLog2(_num_threads))
{

    if (!isPowerOf2(numEntries)) {
        fatal("CVU entries is not a power of 2!");
    }

    cvu.resize(numEntries);

    for (unsigned i = 0; i < numEntries; ++i) {
        cvu[i].valid = false;
    }

    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    tagShiftAmt = instShiftAmt + floorLog2(numEntries);
}

void
ConstantVerificationUnit::reset()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        cvu[i].valid = false;
    }
}

inline
unsigned
ConstantVerificationUnit::getIndex(Addr instPC, ThreadID tid)
{
    // Need to shift PC over by the word offset.
    return ((instPC >> instShiftAmt)
            ^ (tid << (tagShiftAmt - instShiftAmt - log2NumThreads)))
            & idxMask;
}

inline
Addr
ConstantVerificationUnit::getTag(Addr instPC)
{
    return (instPC >> tagShiftAmt) & tagMask;
}

bool
ConstantVerificationUnit::valid(Addr instPC, ThreadID tid)
{
    unsigned cvu_idx = getIndex(instPC, tid);

    Addr inst_tag = getTag(instPC);

    assert(cvu_idx < numEntries);

    if (cvu[cvu_idx].valid
        && inst_tag == cvu[cvu_idx].tag
        && cvu[cvu_idx].tid == tid) {
        return true;
    } else {
        return false;
    }
}


const PCStateBase *
ConstantVerificationUnit::lookup(Addr inst_pc, ThreadID tid)
{
    unsigned cvu_idx = getIndex(inst_pc, tid);

    Addr inst_tag = getTag(inst_pc);

    assert(cvu_idx < numEntries);

    if (cvu[cvu_idx].valid
        && inst_tag == cvu[cvu_idx].tag
        && cvu[cvu_idx].tid == tid) {
        return cvu[cvu_idx].target.get();
    } else {
        return nullptr;
    }
}

void
ConstantVerificationUnit::update(Addr inst_pc, const PCStateBase &target,
                                 ThreadID tid)
{
    unsigned cvu_idx = getIndex(inst_pc, tid);

    assert(cvu_idx < numEntries);

    cvu[cvu_idx].tid = tid;
    cvu[cvu_idx].valid = true;
    set(cvu[cvu_idx].target, target);
    cvu[cvu_idx].tag = getTag(inst_pc);
}
    void clear(Addr inst_pc, const uint64_t &data_addr, ThreadID tid);


//Need to add logic here
ConstantVerificationUnit::clear(Addr inst_pc, const PCStateBase &target,
                                 ThreadID tid)
{

}
} // namespace constant_value_unit
} // namespace gem5
