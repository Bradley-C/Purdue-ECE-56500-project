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

// namespace gem5
// {

// namespace load_value_prediction
// {

// LoadValuePredictionTable::LoadValuePredictionTable(unsigned _numEntries,
//                        unsigned _tagBits,
//                        unsigned _instShiftAmt,
//                        unsigned _num_threads)
//     : numEntries(_numEntries),
//       tagBits(_tagBits),
//       instShiftAmt(_instShiftAmt),
//       log2NumThreads(floorLog2(_num_threads))
// {
//     DPRINTF(Fetch, "BTB: Creating BTB object.\n");

//     if (!isPowerOf2(numEntries)) {
//         fatal("BTB entries is not a power of 2!");
//     }

//     lvpt.resize(numEntries);

//     for (unsigned i = 0; i < numEntries; ++i) {
//         lvpt[i].valid = false;
//     }

//     idxMask = numEntries - 1;

//     tagMask = (1 << tagBits) - 1;

//     tagShiftAmt = instShiftAmt + floorLog2(numEntries);
// }

// void
// LoadValuePredictionTable::reset()
// {
//     for (unsigned i = 0; i < numEntries; ++i) {
//         lvpt[i].valid = false;
//     }
// }

// inline
// unsigned
// LoadValuePredictionTable::getIndex(Addr instPC, ThreadID tid)
// {
//     // Need to shift PC over by the word offset.
//     return ((instPC >> instShiftAmt)
//             ^ (tid << (tagShiftAmt - instShiftAmt - log2NumThreads)))
//             & idxMask;
// }

// inline
// Addr
// LoadValuePredictionTable::getTag(Addr instPC)
// {
//     return (instPC >> tagShiftAmt) & tagMask;
// }

// bool
// LoadValuePredictionTable::valid(Addr instPC, ThreadID tid)
// {
//     unsigned lvpt_idx = getIndex(instPC, tid);

//     Addr inst_tag = getTag(instPC);

//     assert(lvpt_idx < numEntries);

//     if (lvpt[lvpt_idx].valid
//         && inst_tag == lvpt[lvpt_idx].tag
//         && lvpt[lvpt_idx].tid == tid) {
//         return true;
//     } else {
//         return false;
//     }
// }

// // @todo Create some sort of return struct that has both whether or not the
// // address is valid, and also the address.  For now will just use
// // addr = 0 to
// // represent invalid entry.
// const PCStateBase *
// LoadValuePredictionTable::lookup(Addr inst_pc, ThreadID tid)
// {
//     unsigned lvpt_idx = getIndex(inst_pc, tid);

//     Addr inst_tag = getTag(inst_pc);

//     assert(lvpt_idx < numEntries);

//     if (lvpt[lvpt_idx].valid
//         && inst_tag == btb[lvpt_idx].tag
//         && btb[btb_idx].tid == tid) {
//         return btb[btb_idx].target.get();
//     } else {
//         return nullptr;
//     }
// }

// void
// LoadValuePredictionTable::update(Addr inst_pc, const PCStateBase &target,
//                                  ThreadID tid)
// {
//     unsigned btb_idx = getIndex(inst_pc, tid);

//     assert(btb_idx < numEntries);

//     btb[btb_idx].tid = tid;
//     btb[btb_idx].valid = true;
//     set(btb[btb_idx].target, target);
//     btb[btb_idx].tag = getTag(inst_pc);
// }

} // namespace branch_prediction
} // namespace gem5
